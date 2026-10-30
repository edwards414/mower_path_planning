//! An MCAP writer that writes what libmcap 1.3.1's `McapWriter` writes with
//! rosbag2's options (rosbag2_storage_mcap: chunked, `noChunkCRC`, no data
//! section CRC, the summary CRC on), record for record:
//!
//! - **Schema and Channel records** go into the chunk just before a
//!   channel's first message, once per file (`write(const Message&)`); the
//!   summary repeats every schema and channel added, with messages or not.
//!   Statistics count only those written (`schemaCount`, `channelCount`).
//! - **A chunk closes** after the message that brings its records to
//!   `chunk_size` or more (768 KiB by default), and before a Metadata record.
//!   Chunks of 1 KiB or more are compressed, and the compressed form is kept
//!   only if it saves 2 % (`writeChunk`); chunk CRCs stay 0.
//! - **The message indexes** follow their chunk, one per channel with
//!   messages in it, in libstdc++'s order for `currentMessageIndex_`; the
//!   chunk index lists their offsets in its own map's order, and Statistics
//!   its counts in `channelMessageCounts`' ([`crate::stdmap`]).
//! - **The summary:** schemas, channels, Statistics, chunk indexes, metadata
//!   indexes, then a summary offset for each non-empty group (Statistics
//!   always), and the footer with the CRC of everything after DataEnd.
//!
//! Only the header's `library` string is the caller's (D1).

use std::collections::{HashMap, HashSet};
use std::io::{self, Write};

use crate::crc32::Crc32;
use crate::records::{self, op, Body, Channel, ChunkIndex, Schema, Statistics, MAGIC};
use crate::stdmap::{channel_order, StdOrder};

/// libmcap's `DefaultChunkSize`.
pub const DEFAULT_CHUNK_SIZE: u64 = 768 * 1024;

/// How chunks are compressed (`McapWriterOptions::compression`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ChunkCompression {
    None,
    /// zstd at this level: libmcap maps `Fastest` to -5, `Default` to 1,
    /// `Slowest` to 19 (`ZStdCompressionLevel`).
    Zstd(i32),
}

#[derive(Debug, Clone)]
pub struct WriterOptions {
    pub profile: String,
    pub library: String,
    pub chunk_size: u64,
    pub compression: ChunkCompression,
}

pub struct McapWriter<W: Write> {
    out: W,
    written: u64,
    opts: WriterOptions,
    schemas: Vec<Schema>,
    channels: Vec<Channel>,
    written_schemas: HashSet<u16>,
    stats: Statistics,
    counts_order: StdOrder<u16>,
    counts: HashMap<u16, u64>,
    chunk: Vec<u8>,
    chunk_start: u64,
    chunk_end: u64,
    index_order: StdOrder<u16>,
    index: HashMap<u16, Vec<(u64, u64)>>,
    chunk_indexes: Vec<ChunkIndex>,
    metadata_indexes: Vec<(u64, u64, String)>,
    closed: bool,
}

impl<W: Write> McapWriter<W> {
    /// Writes the magic and the header (`McapWriter::open`).
    pub fn new(out: W, opts: WriterOptions) -> io::Result<Self> {
        let mut w = McapWriter {
            out,
            written: 0,
            opts,
            schemas: Vec::new(),
            channels: Vec::new(),
            written_schemas: HashSet::new(),
            stats: Statistics::default(),
            counts_order: StdOrder::new(),
            counts: HashMap::new(),
            chunk: Vec::new(),
            chunk_start: u64::MAX,
            chunk_end: 0,
            index_order: StdOrder::new(),
            index: HashMap::new(),
            chunk_indexes: Vec::new(),
            metadata_indexes: Vec::new(),
            closed: false,
        };
        w.emit(MAGIC)?;
        let mut b = Body::default();
        b.str(&w.opts.profile.clone()).str(&w.opts.library.clone());
        w.emit(&b.record(op::HEADER))?;
        Ok(w)
    }

    fn emit(&mut self, bytes: &[u8]) -> io::Result<()> {
        self.out.write_all(bytes)?;
        self.written += bytes.len() as u64;
        Ok(())
    }

    /// Bytes written to the output so far: the open chunk is not in them
    /// (libmcap's `dataSink()->size()`, what rosbag2's size split reads).
    pub fn written(&self) -> u64 {
        self.written
    }

    /// `addSchema`: the next id, from 1.
    pub fn add_schema(&mut self, name: &str, encoding: &str, data: &[u8]) -> u16 {
        let id = self.schemas.len() as u16 + 1;
        self.schemas.push(Schema { id, name: name.into(), encoding: encoding.into(), data: data.to_vec() });
        id
    }

    /// `addChannel`: the next id, from 1.
    pub fn add_channel(&mut self, schema_id: u16, topic: &str, encoding: &str, metadata: Vec<(String, String)>) -> u16 {
        let id = self.channels.len() as u16 + 1;
        self.channels.push(Channel { id, schema_id, topic: topic.into(), message_encoding: encoding.into(), metadata });
        id
    }

    pub fn write_message(&mut self, channel: u16, sequence: u32, log_time: u64, publish_time: u64, data: &[u8]) -> io::Result<()> {
        let idx = channel as usize;
        if idx == 0 || idx > self.channels.len() {
            return Err(io::Error::new(io::ErrorKind::InvalidInput, format!("invalid channel id {channel}")));
        }
        if !self.counts.contains_key(&channel) {
            let ch = self.channels[idx - 1].clone();
            if ch.schema_id != 0 && !self.written_schemas.contains(&ch.schema_id) {
                let rec = self.schemas[ch.schema_id as usize - 1].record();
                self.chunk.extend_from_slice(&rec);
                self.written_schemas.insert(ch.schema_id);
                self.stats.schema_count += 1;
            }
            self.chunk.extend_from_slice(&ch.record());
            self.counts.insert(channel, 0);
            self.counts_order.insert(channel, channel as u64);
            self.stats.channel_count += 1;
        }
        let offset = self.chunk.len() as u64;
        self.chunk.extend_from_slice(&records::message(channel, sequence, log_time, publish_time, data));
        if self.stats.message_count == 0 {
            self.stats.message_start_time = log_time;
            self.stats.message_end_time = log_time;
        } else {
            self.stats.message_start_time = self.stats.message_start_time.min(log_time);
            self.stats.message_end_time = self.stats.message_end_time.max(log_time);
        }
        self.stats.message_count += 1;
        *self.counts.get_mut(&channel).unwrap() += 1;
        self.index_order.insert(channel, channel as u64);
        self.index.entry(channel).or_default().push((log_time, offset));
        self.chunk_start = self.chunk_start.min(log_time);
        self.chunk_end = self.chunk_end.max(log_time);
        if self.chunk.len() as u64 >= self.opts.chunk_size {
            self.write_chunk()?;
        }
        Ok(())
    }

    /// A Metadata record (closing the open chunk first), indexed.
    pub fn write_metadata(&mut self, name: &str, metadata: &[(String, String)]) -> io::Result<()> {
        if !self.chunk.is_empty() {
            self.write_chunk()?;
        }
        let offset = self.written;
        let mut b = Body::default();
        b.str(name).str_map(metadata);
        let rec = b.record(op::METADATA);
        self.emit(&rec)?;
        self.stats.metadata_count += 1;
        self.metadata_indexes.push((offset, rec.len() as u64, name.to_string()));
        Ok(())
    }

    fn write_chunk(&mut self) -> io::Result<()> {
        let uncompressed = std::mem::take(&mut self.chunk);
        let usize_ = uncompressed.len() as u64;
        let mut compression = "";
        let mut compressed: Option<Vec<u8>> = None;
        if let ChunkCompression::Zstd(level) = self.opts.compression {
            if usize_ >= 1024 {
                let c = zstd_compress2(&uncompressed, level)?;
                if usize_ as f64 / c.len() as f64 >= 1.02 {
                    compression = "zstd";
                    compressed = Some(c);
                }
            }
        }
        let data = compressed.as_deref().unwrap_or(&uncompressed);
        let start = self.written;
        let mut b = Body::default();
        b.u64(self.chunk_start).u64(self.chunk_end).u64(usize_).u32(0).str(compression).bytes64(data);
        let rec = b.record(op::CHUNK);
        let compressed_size = data.len() as u64;
        self.emit(&rec)?;
        let chunk_length = self.written - start;
        let mi_start = self.written;
        let mut written_order = Vec::new();
        let mut offsets = HashMap::new();
        for ch in self.index_order.keys().copied().collect::<Vec<_>>() {
            let entries = self.index.get_mut(&ch).unwrap();
            if entries.is_empty() {
                continue;
            }
            offsets.insert(ch, self.written);
            written_order.push(ch);
            let mut b = Body::default();
            b.u16(ch).u32(entries.len() as u32 * 16);
            for (t, o) in entries.iter() {
                b.u64(*t).u64(*o);
            }
            entries.clear();
            let rec = b.record(op::MESSAGE_INDEX);
            self.emit(&rec)?;
        }
        self.chunk_indexes.push(ChunkIndex {
            message_start_time: self.chunk_start,
            message_end_time: self.chunk_end,
            chunk_start_offset: start,
            chunk_length,
            message_index_offsets: channel_order(written_order).into_iter().map(|c| (c, offsets[&c])).collect(),
            message_index_length: self.written - mi_start,
            compression: compression.to_string(),
            compressed_size,
            uncompressed_size: usize_,
        });
        self.chunk_start = u64::MAX;
        self.chunk_end = 0;
        self.stats.chunk_count += 1;
        Ok(())
    }

    /// The last chunk, DataEnd, the summary and the footer (`close`).
    pub fn finish(mut self) -> io::Result<W> {
        self.close_inner()?;
        Ok(self.out)
    }

    fn close_inner(&mut self) -> io::Result<()> {
        if self.closed {
            return Ok(());
        }
        self.closed = true;
        if !self.chunk.is_empty() {
            self.write_chunk()?;
        }
        let mut b = Body::default();
        b.u32(0);
        self.emit(&b.record(op::DATA_END))?;
        let mut crc = Crc32::new();
        let mut summary: Vec<u8> = Vec::new();
        let base = self.written;
        let pos = |s: &Vec<u8>| base + s.len() as u64;
        let schema_start = pos(&summary);
        for s in &self.schemas {
            summary.extend_from_slice(&s.record());
        }
        let channel_start = pos(&summary);
        for c in &self.channels {
            summary.extend_from_slice(&c.record());
        }
        let statistics_start = pos(&summary);
        let mut stats = self.stats.clone();
        stats.channel_message_counts = self.counts_order.keys().map(|c| (*c, self.counts[c])).collect();
        summary.extend_from_slice(&stats.record());
        let chunk_index_start = pos(&summary);
        for ci in &self.chunk_indexes {
            summary.extend_from_slice(&ci.record());
        }
        let attachment_index_start = pos(&summary);
        let metadata_index_start = attachment_index_start;
        for (offset, length, name) in &self.metadata_indexes {
            let mut b = Body::default();
            b.u64(*offset).u64(*length).str(name);
            summary.extend_from_slice(&b.record(op::METADATA_INDEX));
        }
        let summary_offset_start = pos(&summary);
        let offset = |group: u8, start: u64, len: u64, summary: &mut Vec<u8>| {
            let mut b = Body::default();
            b.u8(group).u64(start).u64(len);
            summary.extend_from_slice(&b.record(op::SUMMARY_OFFSET));
        };
        if !self.schemas.is_empty() {
            offset(op::SCHEMA, schema_start, channel_start - schema_start, &mut summary);
        }
        if !self.channels.is_empty() {
            offset(op::CHANNEL, channel_start, statistics_start - channel_start, &mut summary);
        }
        offset(op::STATISTICS, statistics_start, chunk_index_start - statistics_start, &mut summary);
        if !self.chunk_indexes.is_empty() {
            offset(op::CHUNK_INDEX, chunk_index_start, attachment_index_start - chunk_index_start, &mut summary);
        }
        if !self.metadata_indexes.is_empty() {
            offset(op::METADATA_INDEX, metadata_index_start, summary_offset_start - metadata_index_start, &mut summary);
        }
        let summary_start = base;
        let mut footer = vec![op::FOOTER];
        footer.extend_from_slice(&20u64.to_le_bytes());
        footer.extend_from_slice(&summary_start.to_le_bytes());
        footer.extend_from_slice(&summary_offset_start.to_le_bytes());
        crc.update(&summary);
        crc.update(&footer);
        footer.extend_from_slice(&crc.finish().to_le_bytes());
        self.emit(&summary)?;
        self.emit(&footer)?;
        self.emit(MAGIC)?;
        self.out.flush()
    }
}

/// libmcap's chunk compression: `ZSTD_compress2` at `level`.
pub(crate) fn zstd_compress2(src: &[u8], level: i32) -> io::Result<Vec<u8>> {
    use zstd::zstd_safe::{CCtx, CParameter};
    let mut cctx = CCtx::create();
    cctx.set_parameter(CParameter::CompressionLevel(level)).map_err(zstd_err)?;
    let mut dst = Vec::with_capacity(zstd::zstd_safe::compress_bound(src.len()));
    cctx.compress2(&mut dst, src).map_err(zstd_err)?;
    Ok(dst)
}

/// rosbag2_compression_zstd's per-message compression:
/// `ZSTD_compressCCtx` at level 1.
pub fn zstd_compress_message(src: &[u8]) -> io::Result<Vec<u8>> {
    let mut cctx = zstd::zstd_safe::CCtx::create();
    let mut dst = Vec::with_capacity(zstd::zstd_safe::compress_bound(src.len()));
    cctx.compress(&mut dst, src, 1).map_err(zstd_err)?;
    Ok(dst)
}

fn zstd_err(code: usize) -> io::Error {
    io::Error::other(format!("zstd: {}", zstd::zstd_safe::get_error_name(code)))
}
