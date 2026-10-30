//! MCAP records as libmcap 1.3.1 lays them out (writer.inl `write(...)`):
//! opcode, `u64` length, then the fields little endian; strings and maps
//! with a `u32` byte length. A string map is written sorted by key, as
//! libmcap sorts it; an integer map in the order it is given (the caller's
//! [`crate::stdmap`] order).

pub const MAGIC: &[u8; 8] = b"\x89MCAP0\r\n";

pub mod op {
    pub const HEADER: u8 = 0x01;
    pub const FOOTER: u8 = 0x02;
    pub const SCHEMA: u8 = 0x03;
    pub const CHANNEL: u8 = 0x04;
    pub const MESSAGE: u8 = 0x05;
    pub const CHUNK: u8 = 0x06;
    pub const MESSAGE_INDEX: u8 = 0x07;
    pub const CHUNK_INDEX: u8 = 0x08;
    pub const STATISTICS: u8 = 0x0b;
    pub const METADATA: u8 = 0x0c;
    pub const METADATA_INDEX: u8 = 0x0d;
    pub const SUMMARY_OFFSET: u8 = 0x0e;
    pub const DATA_END: u8 = 0x0f;
}

/// A record body being built.
#[derive(Default)]
pub struct Body(pub Vec<u8>);

impl Body {
    pub fn u8(&mut self, v: u8) -> &mut Self {
        self.0.push(v);
        self
    }
    pub fn u16(&mut self, v: u16) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn u32(&mut self, v: u32) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn u64(&mut self, v: u64) -> &mut Self {
        self.0.extend_from_slice(&v.to_le_bytes());
        self
    }
    pub fn str(&mut self, s: &str) -> &mut Self {
        self.u32(s.len() as u32);
        self.0.extend_from_slice(s.as_bytes());
        self
    }
    pub fn bytes32(&mut self, b: &[u8]) -> &mut Self {
        self.u32(b.len() as u32);
        self.0.extend_from_slice(b);
        self
    }
    pub fn bytes64(&mut self, b: &[u8]) -> &mut Self {
        self.u64(b.len() as u64);
        self.0.extend_from_slice(b);
        self
    }
    /// A string map, sorted by key (std::sort of the pairs).
    pub fn str_map(&mut self, pairs: &[(String, String)]) -> &mut Self {
        let mut sorted: Vec<&(String, String)> = pairs.iter().collect();
        sorted.sort();
        let size: usize = sorted.iter().map(|(k, v)| 8 + k.len() + v.len()).sum();
        self.u32(size as u32);
        for (k, v) in sorted {
            self.str(k);
            self.str(v);
        }
        self
    }
    /// The record: opcode, length, body.
    pub fn record(&self, opcode: u8) -> Vec<u8> {
        let mut r = Vec::with_capacity(9 + self.0.len());
        r.push(opcode);
        r.extend_from_slice(&(self.0.len() as u64).to_le_bytes());
        r.extend_from_slice(&self.0);
        r
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Schema {
    pub id: u16,
    pub name: String,
    pub encoding: String,
    pub data: Vec<u8>,
}

impl Schema {
    pub fn record(&self) -> Vec<u8> {
        let mut b = Body::default();
        b.u16(self.id).str(&self.name).str(&self.encoding).bytes32(&self.data);
        b.record(op::SCHEMA)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Channel {
    pub id: u16,
    pub schema_id: u16,
    pub topic: String,
    pub message_encoding: String,
    pub metadata: Vec<(String, String)>,
}

impl Channel {
    pub fn record(&self) -> Vec<u8> {
        let mut b = Body::default();
        b.u16(self.id).u16(self.schema_id).str(&self.topic).str(&self.message_encoding).str_map(&self.metadata);
        b.record(op::CHANNEL)
    }
}

pub fn message(channel: u16, sequence: u32, log_time: u64, publish_time: u64, data: &[u8]) -> Vec<u8> {
    let mut r = Vec::with_capacity(9 + 22 + data.len());
    r.push(op::MESSAGE);
    r.extend_from_slice(&(22 + data.len() as u64).to_le_bytes());
    r.extend_from_slice(&channel.to_le_bytes());
    r.extend_from_slice(&sequence.to_le_bytes());
    r.extend_from_slice(&log_time.to_le_bytes());
    r.extend_from_slice(&publish_time.to_le_bytes());
    r.extend_from_slice(data);
    r
}

#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct Statistics {
    pub message_count: u64,
    pub schema_count: u16,
    pub channel_count: u32,
    pub attachment_count: u32,
    pub metadata_count: u32,
    pub chunk_count: u32,
    pub message_start_time: u64,
    pub message_end_time: u64,
    /// In the order to write (libmcap's unordered_map order).
    pub channel_message_counts: Vec<(u16, u64)>,
}

impl Statistics {
    pub fn record(&self) -> Vec<u8> {
        let mut b = Body::default();
        b.u64(self.message_count)
            .u16(self.schema_count)
            .u32(self.channel_count)
            .u32(self.attachment_count)
            .u32(self.metadata_count)
            .u32(self.chunk_count)
            .u64(self.message_start_time)
            .u64(self.message_end_time)
            .u32(self.channel_message_counts.len() as u32 * 10);
        for (c, n) in &self.channel_message_counts {
            b.u16(*c).u64(*n);
        }
        b.record(op::STATISTICS)
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ChunkIndex {
    pub message_start_time: u64,
    pub message_end_time: u64,
    pub chunk_start_offset: u64,
    pub chunk_length: u64,
    /// In the order to write.
    pub message_index_offsets: Vec<(u16, u64)>,
    pub message_index_length: u64,
    pub compression: String,
    pub compressed_size: u64,
    pub uncompressed_size: u64,
}

impl ChunkIndex {
    pub fn record(&self) -> Vec<u8> {
        let mut b = Body::default();
        b.u64(self.message_start_time)
            .u64(self.message_end_time)
            .u64(self.chunk_start_offset)
            .u64(self.chunk_length)
            .u32(self.message_index_offsets.len() as u32 * 10);
        for (c, o) in &self.message_index_offsets {
            b.u16(*c).u64(*o);
        }
        b.u64(self.message_index_length).str(&self.compression).u64(self.compressed_size).u64(self.uncompressed_size);
        b.record(op::CHUNK_INDEX)
    }
}
