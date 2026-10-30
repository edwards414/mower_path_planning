//! A bag as `ros2 bag record -s mcap` writes it (rosbag2 Jazzy 0.26.11):
//! the directory, `<dir>_<n>.mcap` files, the two `rosbag2` metadata records
//! of each file and `metadata.yaml`, with the split rules, per-message zstd
//! and snapshot mode. The source it follows: rosbag2_cpp
//! `SequentialWriter`, rosbag2_compression `SequentialCompressionWriter`,
//! rosbag2_storage_mcap `MCAPStorage`, rosbag2_cpp `CircularMessageCache`.
//!
//! What rosbag2 does on threads is done here in line, in the order the
//! messages are given: rosbag2's file order is the order its cache and
//! compression threads finish in (E5 design A5), and its size splits fall a
//! message or more later than here (the E5-0 fixture F2S), so a caller
//! replaying a bag passes the messages in file order and forces those
//! splits with [`BagWriter::split`].

use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::{self, BufWriter};
use std::path::{Path, PathBuf};

use mower_schema::Definition;

use crate::meta::{BagMetadata, FileInformation, Qos, TopicInformation, TopicMetadata};
use crate::stdmap::{hash_bytes, StdOrder};
use crate::writer::{zstd_compress_message, ChunkCompression, McapWriter, WriterOptions, DEFAULT_CHUNK_SIZE};

/// `--storage-preset-profile` (rosbag2_storage_mcap `SetOptionsForPreset`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Preset {
    /// none: uncompressed chunks of 768 KiB
    None,
    /// `zstd_fast`: zstd chunks at libmcap's `Fastest` (-5), 768 KiB
    ZstdFast,
}

/// What a `ros2 bag record` command line sets.
#[derive(Debug, Clone)]
pub struct BagOptions {
    pub preset: Preset,
    /// `--compression-mode message --compression-format zstd`
    pub message_compression: bool,
    /// `--max-bag-size` in bytes, 0 = no split
    pub max_bagfile_size: u64,
    /// `--max-bag-duration` in seconds, 0 = no split
    pub max_bagfile_duration: u64,
    /// `--snapshot-mode --max-cache-size <bytes>`
    pub snapshot_cache_size: Option<u64>,
    /// The MCAP header's `library` (D1: the writer's own name).
    pub library: String,
    /// `ros_distro` (rosbag2 reads `ROS_DISTRO`; D10: `jazzy`).
    pub ros_distro: String,
}

impl BagOptions {
    pub fn new(library: &str) -> BagOptions {
        BagOptions {
            preset: Preset::None,
            message_compression: false,
            max_bagfile_size: 0,
            max_bagfile_duration: 0,
            snapshot_cache_size: None,
            library: library.to_string(),
            ros_distro: "jazzy".to_string(),
        }
    }
}

/// One open `<dir>_<n>.mcap` (`MCAPStorage` opened for writing).
struct Storage {
    writer: McapWriter<BufWriter<File>>,
    schema_ids: HashMap<String, u16>,
    channel_ids: HashMap<String, u16>,
}

impl Storage {
    fn open(path: &Path, preset: Preset, library: &str) -> io::Result<Storage> {
        let compression = match preset {
            Preset::None => ChunkCompression::None,
            Preset::ZstdFast => ChunkCompression::Zstd(-5),
        };
        let opts = WriterOptions { profile: "ros2".into(), library: library.into(), chunk_size: DEFAULT_CHUNK_SIZE, compression };
        let f = File::create(path)?;
        Ok(Storage { writer: McapWriter::new(BufWriter::new(f), opts)?, schema_ids: HashMap::new(), channel_ids: HashMap::new() })
    }

    /// `MCAPStorage::create_topic`: the schema the first time its type is
    /// seen in this file, then the channel.
    fn create_topic(&mut self, t: &TopicMetadata, def: &Definition) {
        if self.channel_ids.contains_key(&t.name) {
            return;
        }
        let schema_id = match self.schema_ids.get(&t.type_name) {
            Some(id) => *id,
            None => {
                let id = self.writer.add_schema(&t.type_name, def.encoding, def.text.as_bytes());
                self.schema_ids.insert(t.type_name.clone(), id);
                id
            }
        };
        let metadata = vec![
            ("offered_qos_profiles".to_string(), Qos::dump_list(&t.offered_qos_profiles)),
            ("topic_type_hash".to_string(), t.type_description_hash.clone()),
        ];
        let id = self.writer.add_channel(schema_id, &t.name, &t.serialization_format, metadata);
        self.channel_ids.insert(t.name.clone(), id);
    }

    fn write(&mut self, topic: &str, recv: i64, send: i64, data: &[u8]) -> io::Result<()> {
        let id = *self.channel_ids.get(topic).ok_or_else(|| other(format!("unknown topic {topic}")))?;
        self.writer.write_message(id, 0, recv as u64, send as u64, data)
    }

    fn update_metadata(&mut self, m: &BagMetadata) -> io::Result<()> {
        self.writer.write_metadata("rosbag2", &[("serialized_metadata".to_string(), m.to_record_yaml())])
    }

    fn close(self) -> io::Result<()> {
        self.writer.finish().map(|_| ())
    }
}

fn other(m: String) -> io::Error {
    io::Error::other(m)
}

struct Topic {
    metadata: TopicMetadata,
    definition: Definition,
    message_count: u64,
}

struct Cached {
    topic: String,
    recv: i64,
    send: i64,
    data: Vec<u8>,
}

/// A bag being written. Topics first ([`BagWriter::create_topic`]), then
/// messages; [`BagWriter::close`] finalizes.
pub struct BagWriter {
    dir: PathBuf,
    base: String,
    opts: BagOptions,
    storage: Option<Storage>,
    metadata: BagMetadata,
    order: StdOrder<String>,
    topics: HashMap<String, Topic>,
    is_first_message: bool,
    ring: VecDeque<Cached>,
    ring_bytes: u64,
}

impl BagWriter {
    /// Creates `dir` (it must not exist, as for rosbag2) and its first file,
    /// with the open-time metadata record (`SequentialWriter::open`).
    pub fn create(dir: &Path, opts: BagOptions) -> io::Result<BagWriter> {
        std::fs::create_dir(dir)?;
        let base = dir.file_name().and_then(|n| n.to_str()).ok_or_else(|| other("a bag dir needs a name".into()))?.to_string();
        let rel = format!("{base}_0.mcap");
        let storage = Storage::open(&dir.join(&rel), opts.preset, &opts.library)?;
        let (compression_format, compression_mode) =
            if opts.message_compression { ("zstd", "MESSAGE") } else { ("", "") };
        let metadata = BagMetadata {
            version: 9,
            storage_identifier: "mcap".into(),
            duration: 0,
            starting_time: i64::MAX,
            message_count: 0,
            topics_with_message_count: Vec::new(),
            compression_format: compression_format.into(),
            compression_mode: compression_mode.into(),
            relative_file_paths: vec![rel.clone()],
            files: vec![FileInformation { path: rel, starting_time: i64::MAX, duration: 0, message_count: 0 }],
            custom_data: Vec::new(),
            ros_distro: opts.ros_distro.clone(),
        };
        let mut w = BagWriter {
            dir: dir.to_path_buf(),
            base,
            opts,
            storage: Some(storage),
            metadata,
            order: StdOrder::new(),
            topics: HashMap::new(),
            is_first_message: true,
            ring: VecDeque::new(),
            ring_bytes: 0,
        };
        let m = w.metadata.clone();
        w.storage_mut()?.update_metadata(&m)?;
        Ok(w)
    }

    fn storage_mut(&mut self) -> io::Result<&mut Storage> {
        self.storage.as_mut().ok_or_else(|| other("the bag is closed".into()))
    }

    /// `SequentialWriter::create_topic`, with mower_schema's definition of
    /// the type (rosbag2's empty one for a type it does not know).
    pub fn create_topic(&mut self, topic: TopicMetadata) -> io::Result<()> {
        let def = mower_schema::rosbag2_definition(&topic.type_name);
        self.create_topic_with_definition(topic, def)
    }

    pub fn create_topic_with_definition(&mut self, topic: TopicMetadata, definition: Definition) -> io::Result<()> {
        if self.topics.contains_key(&topic.name) {
            return Ok(());
        }
        self.order.insert(topic.name.clone(), hash_bytes(topic.name.as_bytes()));
        self.storage_mut()?.create_topic(&topic, &definition);
        self.metadata.topics_with_message_count.push(TopicInformation { topic_metadata: topic.clone(), message_count: 0 });
        self.topics.insert(topic.name.clone(), Topic { metadata: topic, definition, message_count: 0 });
        Ok(())
    }

    /// One message: its receive time (`log_time`), its source time
    /// (`publish_time`), its CDR (compressed here when the bag compresses
    /// messages). `SequentialWriter::write`.
    pub fn write(&mut self, topic: &str, recv: i64, send: i64, cdr: &[u8]) -> io::Result<()> {
        if !self.topics.contains_key(topic) {
            return Err(other(format!("Failed to write on topic '{topic}'. Call create_topic() before first write.")));
        }
        let data = if self.opts.message_compression { zstd_compress_message(cdr)? } else { cdr.to_vec() };
        if self.is_first_message {
            self.metadata.starting_time = recv;
            self.is_first_message = false;
        }
        if self.opts.snapshot_cache_size.is_none() && self.should_split(recv) {
            self.split()?;
            self.metadata.files.last_mut().unwrap().starting_time = recv;
        }
        let m = &mut self.metadata;
        m.starting_time = m.starting_time.min(recv);
        let f = m.files.last_mut().unwrap();
        f.starting_time = f.starting_time.min(recv);
        m.duration = m.duration.max(recv - m.starting_time);
        f.duration = f.duration.max(recv - f.starting_time);
        f.message_count += 1;
        match self.opts.snapshot_cache_size {
            None => self.write_messages(vec![Cached { topic: topic.to_string(), recv, send, data }]),
            Some(max) => {
                // MessageCacheCircularBuffer::push
                let len = data.len() as u64;
                if len > max {
                    return Ok(());
                }
                while self.ring_bytes > max - len {
                    let old = self.ring.pop_front().unwrap();
                    self.ring_bytes -= old.data.len() as u64;
                }
                self.ring_bytes += len;
                self.ring.push_back(Cached { topic: topic.to_string(), recv, send, data });
                Ok(())
            }
        }
    }

    /// `SequentialWriter::write_messages` (the cache consumer's callback).
    fn write_messages(&mut self, batch: Vec<Cached>) -> io::Result<()> {
        if batch.is_empty() {
            return Ok(());
        }
        for c in &batch {
            self.storage_mut()?.write(&c.topic, c.recv, c.send, &c.data)?;
        }
        if self.opts.snapshot_cache_size.is_some() {
            let f = self.metadata.files.last_mut().unwrap();
            f.starting_time = batch[0].recv;
            f.duration = batch[batch.len() - 1].recv - batch[0].recv;
            f.message_count = batch.len() as u64;
        }
        self.metadata.message_count += batch.len() as u64;
        for c in &batch {
            if let Some(t) = self.topics.get_mut(&c.topic) {
                t.message_count += 1;
            }
        }
        Ok(())
    }

    fn should_split(&self, now: i64) -> bool {
        let mut split = false;
        if self.opts.max_bagfile_size != 0 {
            split = self.storage.as_ref().map_or(0, |s| s.writer.written()) >= self.opts.max_bagfile_size;
        }
        if self.opts.max_bagfile_duration != 0 {
            let max = self.opts.max_bagfile_duration as i64 * 1_000_000_000;
            split = split || now.saturating_sub(self.metadata.files.last().unwrap().starting_time) > max;
        }
        split
    }

    /// Starts the next file now (`switch_to_next_storage`): the current
    /// file gets the metadata as it stands, the new one its open-time copy,
    /// then every topic again, in rosbag2's map order.
    pub fn split(&mut self) -> io::Result<()> {
        let m = self.metadata.clone();
        let old = self.storage.take().ok_or_else(|| other("the bag is closed".into()))?;
        let mut old = old;
        old.update_metadata(&m)?;
        let rel = format!("{}_{}.mcap", self.base, self.metadata.relative_file_paths.len());
        let storage = Storage::open(&self.dir.join(&rel), self.opts.preset, &self.opts.library)?;
        old.close()?;
        self.storage = Some(storage);
        self.metadata.files.push(FileInformation { path: rel.clone(), starting_time: i64::MAX, duration: 0, message_count: 0 });
        self.metadata.relative_file_paths.push(rel);
        let m = self.metadata.clone();
        let names: Vec<String> = self.order.keys().cloned().collect();
        let st = self.storage.as_mut().unwrap();
        st.update_metadata(&m)?;
        for n in names {
            let t = &self.topics[&n];
            st.create_topic(&t.metadata, &t.definition);
        }
        Ok(())
    }

    /// Snapshot mode: writes the buffer to the current file, then starts the
    /// next one (`take_snapshot`); the buffer starts empty again. `false`
    /// when the bag is not in snapshot mode.
    pub fn take_snapshot(&mut self) -> io::Result<bool> {
        if self.opts.snapshot_cache_size.is_none() {
            return Ok(false);
        }
        let batch: Vec<Cached> = self.ring.drain(..).collect();
        self.ring_bytes = 0;
        self.write_messages(batch)?;
        self.split()?;
        Ok(true)
    }

    /// Finalizes (`SequentialWriter::close`): the topics with their counts in
    /// rosbag2's map order, the last file's final record and summary, then
    /// `metadata.yaml`. What a snapshot buffer still holds is dropped, as
    /// rosbag2 drops it.
    pub fn close(mut self) -> io::Result<BagMetadata> {
        self.ring.clear();
        let mut topics = Vec::new();
        let mut total = 0;
        for n in self.order.keys() {
            let t = &self.topics[n];
            topics.push(TopicInformation { topic_metadata: t.metadata.clone(), message_count: t.message_count });
            total += t.message_count;
        }
        self.metadata.topics_with_message_count = topics;
        self.metadata.message_count = total;
        let m = self.metadata.clone();
        let mut st = self.storage.take().ok_or_else(|| other("the bag is closed".into()))?;
        st.update_metadata(&m)?;
        st.close()?;
        std::fs::write(self.dir.join("metadata.yaml"), m.to_metadata_yaml())?;
        Ok(m)
    }
}
