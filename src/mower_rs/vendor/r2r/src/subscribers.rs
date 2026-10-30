use futures::{channel::mpsc, stream::Stream};
use std::ffi::CString;
// mower_rs patch 3: RawSubscription's drop counter
use std::sync::{
    atomic::{AtomicU64, Ordering},
    Arc,
};
use std::{
    pin::Pin,
    task::{Context, Poll},
};

use crate::{error::*, msg_types::*, qos::QosProfile};
use r2r_rcl::*;
use std::ffi::{c_void, CStr};

/// mower_rs patch (vendor/r2r/VENDORED.md): what `Node::subscribe` returns,
/// the stream of the subscription's messages, which
/// `Node::destroy_subscription` takes back to take the subscription off the
/// graph.
///
/// Dropping it instead is upstream's way: the subscription is destroyed only
/// at its next message, and until then stays on the graph, matched with
/// every writer of its topic.
pub struct Subscription<T> {
    pub(crate) messages: mpsc::Receiver<T>,
    // The node's subscriber, compared with theirs and never dereferenced.
    // While this stream lives, so does its subscriber: the node removes one
    // on its own only once its stream is dropped.
    pub(crate) handle: *const rcl_subscription_t,
}

// The stream `subscribe` returned before was Send + Sync for a Send `T`.
unsafe impl<T: Send> Send for Subscription<T> {}
unsafe impl<T: Send> Sync for Subscription<T> {}

impl<T> Stream for Subscription<T> {
    type Item = T;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<T>> {
        Pin::new(&mut self.messages).poll_next(cx)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.messages.size_hint()
    }
}

/// mower_rs patch 3 (vendor/r2r/VENDORED.md): one message of
/// `Node::subscribe_raw_with_info`, its bytes as rmw took them and what
/// `rcl_take_serialized_message` said about it in its `rmw_message_info_t`
/// (upstream's `subscribe_raw` throws that away). What rosbag2 records: the
/// bytes, `received_timestamp` as the log time, `source_timestamp` as the
/// publish time.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RawMessage {
    /// The CDR, its 4-byte encapsulation header included.
    pub data: Vec<u8>,
    /// When the writer published it: nanoseconds since the epoch, on the
    /// writer's clock (`rmw_message_info_t::source_timestamp`).
    pub source_timestamp: i64,
    /// When this reader's rmw received it: nanoseconds since the epoch
    /// (`rmw_message_info_t::received_timestamp`).
    pub received_timestamp: i64,
    /// `rmw_message_info_t::publisher_gid.data`: the same for every message
    /// of one writer, different between writers. rmw_cyclonedds puts the
    /// writer's DDS instance handle in the reader's participant there (8
    /// bytes, then zeros), not the GUID `TopicEndpointInfo::endpoint_gid`
    /// shows for it; rmw_fastrtps the GUID.
    pub publisher_gid: [u8; RMW_GID_STORAGE_SIZE as usize],
}

/// mower_rs patch 3: how many messages a [`RawSubscription`] dropped because
/// its queue was full. A clone counts the same messages, so the caller can
/// keep one after the stream has moved into a task.
#[derive(Clone, Debug, Default)]
pub struct DropCounter(Arc<AtomicU64>);

impl DropCounter {
    /// The messages dropped so far.
    pub fn get(&self) -> u64 {
        self.0.load(Ordering::Relaxed)
    }
}

/// mower_rs patch 3: what `Node::subscribe_raw_with_info` returns, the stream
/// of the subscription's [`RawMessage`]s, which
/// `Node::destroy_raw_subscription` takes back.
///
/// Its queue holds the `capacity` the caller chose, where upstream's
/// `subscribe_raw` holds 11 (a channel of 10) and drops beyond that with a
/// debug log. A message that finds the queue full is dropped too, and counted
/// ([`RawSubscription::drops`]).
pub struct RawSubscription {
    pub(crate) messages: mpsc::Receiver<RawMessage>,
    // The node's subscriber, compared and never dereferenced (as for
    // `Subscription`).
    pub(crate) handle: *const rcl_subscription_t,
    pub(crate) drops: DropCounter,
}

unsafe impl Send for RawSubscription {}
unsafe impl Sync for RawSubscription {}

impl RawSubscription {
    /// A counter of the messages dropped so far, and from now on.
    pub fn drops(&self) -> DropCounter {
        self.drops.clone()
    }
}

impl Stream for RawSubscription {
    type Item = RawMessage;

    fn poll_next(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Option<RawMessage>> {
        Pin::new(&mut self.messages).poll_next(cx)
    }

    fn size_hint(&self) -> (usize, Option<usize>) {
        self.messages.size_hint()
    }
}

pub trait Subscriber_ {
    fn handle(&self) -> &rcl_subscription_t;
    /// Returns true if the subscriber stream has been dropped.
    fn handle_incoming(&mut self) -> bool;
    fn destroy(&mut self, node: &mut rcl_node_t);
}

pub struct TypedSubscriber<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub sender: mpsc::Sender<T>,
}

pub struct NativeSubscriber<T>
where
    T: WrappedTypesupport,
{
    pub rcl_handle: rcl_subscription_t,
    pub sender: mpsc::Sender<WrappedNativeMsg<T>>,
}

pub struct UntypedSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub topic_type: String,
    pub sender: mpsc::Sender<Result<serde_json::Value>>,
}

pub struct RawSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub msg_buf: rcl_serialized_message_t,
    pub sender: mpsc::Sender<Vec<u8>>,
}

/// mower_rs patch 3: the subscriber of `Node::subscribe_raw_with_info`.
pub struct RawInfoSubscriber {
    pub rcl_handle: rcl_subscription_t,
    pub msg_buf: rcl_serialized_message_t,
    pub sender: mpsc::Sender<RawMessage>,
    pub drops: DropCounter,
}

impl<T: 'static> Subscriber_ for TypedSubscriber<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsg::<T>::new();
        let ret = unsafe {
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            let msg = T::from_native(&msg);
            if let Err(e) = self.sender.try_send(msg) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                log::debug!("error {:?}", e)
            }
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl<T: 'static> Subscriber_ for NativeSubscriber<T>
where
    T: WrappedTypesupport,
{
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let msg = unsafe {
            if rcl_subscription_can_loan_messages(&self.rcl_handle) {
                let mut loaned_msg: *mut c_void = std::ptr::null_mut();
                let ret = rcl_take_loaned_message(
                    &self.rcl_handle,
                    &mut loaned_msg,
                    &mut msg_info,
                    std::ptr::null_mut(),
                );
                if ret != RCL_RET_OK as i32 {
                    return false;
                }
                let handle_box = Box::new(self.rcl_handle);
                let deallocator = Box::new(|msg: *mut T::CStruct| {
                    let handle_ptr = Box::into_raw(handle_box);
                    let ret =
                        rcl_return_loaned_message_from_subscription(handle_ptr, msg as *mut c_void);
                    if ret == RCL_RET_OK as i32 {
                        drop(Box::from_raw(handle_ptr));
                    } else {
                        let topic_str = rcl_subscription_get_topic_name(handle_ptr);
                        let topic = CStr::from_ptr(topic_str).to_str().expect("to_str() call failed").to_owned();
                        drop(Box::from_raw(handle_ptr));

                        let err_str = rcutils_get_error_string();
                        let err_str_ptr = &(err_str.str_) as *const std::os::raw::c_char;
                        let error_msg = CStr::from_ptr(err_str_ptr);

                        // Returning a loan shouldn't fail unless one of the handles or pointers
                        // is invalid, both of which indicate a severe bug. Panicking is therefore
                        // more appropriate than leaking the loaned message.
                        panic!(
                            "rcl_return_loaned_message_from_subscription() \
                            failed for subscription on topic {}: {}",
                            topic,
                            error_msg.to_str().expect("to_str() call failed")
                        );
                    }
                });
                WrappedNativeMsg::<T>::from_loaned(loaned_msg as *mut T::CStruct, deallocator)
            } else {
                let mut new_msg = WrappedNativeMsg::<T>::new();
                let ret = rcl_take(
                    &self.rcl_handle,
                    new_msg.void_ptr_mut(),
                    &mut msg_info,
                    std::ptr::null_mut(),
                );
                if ret != RCL_RET_OK as i32 {
                    return false;
                }
                new_msg
            }
        };
        if let Err(e) = self.sender.try_send(msg) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            log::error!("error {:?}", e)
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl Subscriber_ for UntypedSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let mut msg = WrappedNativeMsgUntyped::new_from(&self.topic_type)
            .unwrap_or_else(|_| panic!("no typesupport for {}", self.topic_type));
        let ret = unsafe {
            rcl_take(&self.rcl_handle, msg.void_ptr_mut(), &mut msg_info, std::ptr::null_mut())
        };
        if ret == RCL_RET_OK as i32 {
            let json = msg.to_json();
            if let Err(e) = self.sender.try_send(json) {
                if e.is_disconnected() {
                    // user dropped the handle to the stream, signal removal.
                    return true;
                }
                log::debug!("error {:?}", e)
            }
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
        }
    }
}

impl Subscriber_ for RawSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default(); // we dont care for now
        let ret = unsafe {
            rcl_take_serialized_message(
                &self.rcl_handle,
                &mut self.msg_buf as *mut rcl_serialized_message_t,
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            log::error!("failed to take serialized message");
            return false;
        }

        let data_bytes = if self.msg_buf.buffer == std::ptr::null_mut() {
            Vec::new()
        } else {
            unsafe {
                std::slice::from_raw_parts(self.msg_buf.buffer, self.msg_buf.buffer_length).to_vec()
            }
        };

        if let Err(e) = self.sender.try_send(data_bytes) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            log::debug!("error {:?}", e)
        }

        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
            rcutils_uint8_array_fini(&mut self.msg_buf as *mut rcl_serialized_message_t);
        }
    }
}

/// mower_rs patch 3: `RawSubscriber`'s take, keeping the message info, and a
/// full queue counted instead of logged at debug level.
impl Subscriber_ for RawInfoSubscriber {
    fn handle(&self) -> &rcl_subscription_t {
        &self.rcl_handle
    }

    fn handle_incoming(&mut self) -> bool {
        let mut msg_info = rmw_message_info_t::default();
        let ret = unsafe {
            rcl_take_serialized_message(
                &self.rcl_handle,
                &mut self.msg_buf as *mut rcl_serialized_message_t,
                &mut msg_info,
                std::ptr::null_mut(),
            )
        };
        if ret != RCL_RET_OK as i32 {
            // RCL_RET_SUBSCRIPTION_TAKE_FAILED: a wakeup with nothing to take
            return false;
        }
        let data = if self.msg_buf.buffer.is_null() {
            Vec::new()
        } else {
            unsafe { std::slice::from_raw_parts(self.msg_buf.buffer, self.msg_buf.buffer_length).to_vec() }
        };
        let message = RawMessage {
            data,
            source_timestamp: msg_info.source_timestamp,
            received_timestamp: msg_info.received_timestamp,
            publisher_gid: msg_info.publisher_gid.data,
        };
        if let Err(e) = self.sender.try_send(message) {
            if e.is_disconnected() {
                // user dropped the handle to the stream, signal removal.
                return true;
            }
            self.drops.0.fetch_add(1, Ordering::Relaxed);
        }
        false
    }

    fn destroy(&mut self, node: &mut rcl_node_t) {
        unsafe {
            rcl_subscription_fini(&mut self.rcl_handle, node);
            rcutils_uint8_array_fini(&mut self.msg_buf as *mut rcl_serialized_message_t);
        }
    }
}

pub fn create_subscription_helper(
    node: &mut rcl_node_t, topic: &str, ts: *const rosidl_message_type_support_t,
    qos_profile: QosProfile,
) -> Result<rcl_subscription_t> {
    let mut subscription_handle = unsafe { rcl_get_zero_initialized_subscription() };
    let topic_c_string = CString::new(topic).map_err(|_| Error::RCL_RET_INVALID_ARGUMENT)?;

    let result = unsafe {
        let mut subscription_options = rcl_subscription_get_default_options();
        subscription_options.qos = qos_profile.into();
        rcl_subscription_init(
            &mut subscription_handle,
            node,
            ts,
            topic_c_string.as_ptr(),
            &subscription_options,
        )
    };
    if result == RCL_RET_OK as i32 {
        Ok(subscription_handle)
    } else {
        Err(Error::from_rcl_error(result))
    }
}
