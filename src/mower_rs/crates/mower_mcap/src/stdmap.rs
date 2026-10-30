//! The iteration order of a libstdc++ (GCC 13, the robot image's)
//! `std::unordered_map`, which decides four orders in a rosbag2 bag: the
//! message indexes after a chunk, Statistics' per-channel counts and a chunk
//! index's offsets (libmcap 1.3.1's `unordered_map<ChannelId, ...>`), and
//! the topics of the final metadata record (rosbag2's
//! `unordered_map<std::string, TopicInformation>`).
//!
//! The order depends only on the keys, their hashes and the insertion
//! order, so it can be computed: libstdc++ keeps every node in one singly
//! linked list; a node goes to the front of its bucket, or to the front of
//! the list when its bucket is empty (`_M_insert_bucket_begin`); a rehash
//! walks the list and does the same into the new buckets
//! (`_M_rehash_aux`); the bucket count follows `_Prime_rehash_policy` (13
//! buckets for the first element, then the next prime at least twice as
//! many whenever the count would pass the bucket count). Integer keys hash
//! to themselves; strings with `std::_Hash_bytes` (seed 0xc70f6907). The
//! E5-0 fixtures check all four orders.

/// libstdc++'s `std::hash<std::string>`: `_Hash_bytes(data, len, 0xc70f6907)`
/// for a 64-bit `size_t` (libsupc++ hash_bytes.cc, the MurmurHash2 variant).
pub fn hash_bytes(data: &[u8]) -> u64 {
    const MUL: u64 = (0xc6a4_a793u64 << 32) + 0x5bd1_e995;
    let shift_mix = |v: u64| v ^ (v >> 47);
    let len = data.len();
    let aligned = len & !7;
    let mut hash = 0xc70f_6907u64 ^ (len as u64).wrapping_mul(MUL);
    for chunk in data[..aligned].chunks_exact(8) {
        let d = shift_mix(u64::from_le_bytes(chunk.try_into().unwrap()).wrapping_mul(MUL)).wrapping_mul(MUL);
        hash ^= d;
        hash = hash.wrapping_mul(MUL);
    }
    if len & 7 != 0 {
        let mut d = 0u64;
        for &b in data[aligned..].iter().rev() {
            d = (d << 8) | b as u64;
        }
        hash ^= d;
        hash = hash.wrapping_mul(MUL);
    }
    shift_mix(shift_mix(hash).wrapping_mul(MUL))
}

/// `__prime_list` from libstdc++'s hashtable_c++0x.cc, as far as a bag can
/// need it.
const PRIMES: [u64; 60] = [
    2, 3, 5, 7, 11, 13, 17, 19, 23, 29, 31, 37, 41, 43, 47, 53, 59, 61, 67, 71, 73, 79, 83, 89, 97, 103, 109, 113,
    127, 137, 139, 149, 157, 167, 179, 193, 199, 211, 227, 241, 257, 277, 293, 313, 337, 359, 383, 409, 439, 467,
    503, 541, 577, 619, 661, 709, 761, 823, 887, 953,
];
const FAST_BKT: [u64; 14] = [2, 2, 2, 3, 5, 5, 7, 7, 11, 11, 11, 11, 13, 13];

/// The keys of one `unordered_map`, in the order iteration visits them.
#[derive(Debug, Clone)]
pub struct StdOrder<K: Clone + PartialEq> {
    /// (key, hash), in list order
    list: Vec<(K, u64)>,
    bucket_count: u64,
    next_resize: u64,
}

impl<K: Clone + PartialEq> Default for StdOrder<K> {
    fn default() -> Self {
        StdOrder { list: Vec::new(), bucket_count: 1, next_resize: 0 }
    }
}

impl<K: Clone + PartialEq> StdOrder<K> {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn len(&self) -> usize {
        self.list.len()
    }

    pub fn is_empty(&self) -> bool {
        self.list.is_empty()
    }

    pub fn contains(&self, key: &K) -> bool {
        self.list.iter().any(|(k, _)| k == key)
    }

    /// Iteration order.
    pub fn keys(&self) -> impl Iterator<Item = &K> {
        self.list.iter().map(|(k, _)| k)
    }

    fn next_bkt(&mut self, n: u64) -> u64 {
        if (n as usize) < FAST_BKT.len() {
            self.next_resize = FAST_BKT[n as usize];
            return FAST_BKT[n as usize];
        }
        let p = *PRIMES[6..].iter().find(|&&p| p >= n).expect("more buckets than any bag needs");
        self.next_resize = p;
        p
    }

    /// `_Prime_rehash_policy::_M_need_rehash(bucket_count, n_elt, 1)`, with
    /// the max load factor 1.0.
    fn need_rehash(&mut self) -> Option<u64> {
        let n = self.list.len() as u64 + 1;
        if n > self.next_resize {
            let min_bkts = n.max(if self.next_resize == 0 { 11 } else { 0 });
            if min_bkts >= self.bucket_count {
                return Some(self.next_bkt((min_bkts + 1).max(self.bucket_count * 2)));
            }
            self.next_resize = self.bucket_count;
        }
        None
    }

    /// The position after which a node of bucket `b` goes, or `None` for the
    /// front of the list: the node before the bucket's first node, as
    /// libstdc++'s bucket array stores it.
    fn bucket_begin(&self, b: u64) -> Option<Option<usize>> {
        let first = self.list.iter().position(|(_, h)| h % self.bucket_count == b)?;
        Some(first.checked_sub(1))
    }

    fn rehash(&mut self, n: u64) {
        let old = std::mem::take(&mut self.list);
        self.bucket_count = n;
        for node in old {
            let b = node.1 % n;
            match self.bucket_begin(b) {
                None => self.list.insert(0, node),
                Some(before) => {
                    let at = before.map_or(0, |i| i + 1);
                    self.list.insert(at, node);
                }
            }
        }
    }

    /// `insert` / `emplace` / `operator[]` of a new key (a key already there
    /// changes nothing).
    pub fn insert(&mut self, key: K, hash: u64) {
        if self.contains(&key) {
            return;
        }
        if let Some(n) = self.need_rehash() {
            self.rehash(n);
        }
        match self.bucket_begin(hash % self.bucket_count) {
            None => self.list.insert(0, (key, hash)),
            Some(before) => {
                let at = before.map_or(0, |i| i + 1);
                self.list.insert(at, (key, hash));
            }
        }
    }
}

/// A channel-id map's order (`std::hash<uint16_t>` is the identity).
pub fn channel_order(inserted: impl IntoIterator<Item = u16>) -> Vec<u16> {
    let mut m = StdOrder::new();
    for id in inserted {
        m.insert(id, id as u64);
    }
    m.keys().copied().collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn yard2_s_orders() {
        // the live yard2 bag (E5 design 2.1): channels in first-message order
        // 1, 2, 6, 13, 14, 4, 7, 9, 5, 12
        assert_eq!(channel_order([1, 2, 6, 13, 14, 4, 7, 9, 5, 12]), [12, 5, 9, 7, 4, 13, 6, 2, 14, 1]);
        // its first chunk's message indexes, and that chunk index's offsets
        assert_eq!(channel_order([1, 2, 6, 13, 14, 4]), [4, 13, 6, 2, 14, 1]);
        assert_eq!(channel_order([4, 13, 6, 2, 14, 1]), [1, 14, 2, 6, 13, 4]);
    }

    #[test]
    fn string_hashes_and_a_rehash() {
        // three topics of F1's snapshot group, created in this order
        let mut m = StdOrder::new();
        for t in ["/local_costmap/costmap", "/global_costmap/costmap", "/plan"] {
            m.insert(t.to_string(), hash_bytes(t.as_bytes()));
        }
        let order: Vec<&str> = m.keys().map(|s| s.as_str()).collect();
        assert_eq!(order, ["/plan", "/global_costmap/costmap", "/local_costmap/costmap"]);
        // 13 buckets, then 29 at the 14th key, 59 at the 30th
        let mut m = StdOrder::new();
        for i in 0..40u16 {
            m.insert(i, i as u64);
            assert_eq!(m.bucket_count, if i < 13 { 13 } else if i < 29 { 29 } else { 59 }, "{i}");
        }
    }
}
