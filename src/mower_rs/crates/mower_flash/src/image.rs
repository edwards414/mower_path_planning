//! The image and its manifest, read the way `mower_flash.py` reads them.

use mower_base_core::boot::{APP_END_ADDRESS, APP_MAX_SIZE, APP_START_ADDRESS};

use crate::link::os_error;
use crate::pyjson::{self, Py};
use crate::Failure;

/// `FileNotFoundError: [Errno 2] No such file or directory: 'x'` and the rest
/// of the OSError family, for a failed `open()`.
pub fn open_error(err: &std::io::Error, path: &str) -> Failure {
    let errno = err.raw_os_error().unwrap_or(0);
    let class = match errno {
        libc::ENOENT => "FileNotFoundError",
        libc::EACCES | libc::EPERM => "PermissionError",
        libc::EISDIR => "IsADirectoryError",
        libc::ENOTDIR => "NotADirectoryError",
        libc::EEXIST => "FileExistsError",
        _ => "OSError",
    };
    Failure::Uncaught(format!("{class}: {}: {}", os_error(errno), pyjson::str_repr(path)))
}

/// `load_image(path)`: the same checks in the same order, the same
/// `sys.exit` texts, padded with 0xFF to a multiple of 4.
pub fn load_image(path: &str) -> Result<Vec<u8>, Failure> {
    let mut image = std::fs::read(path).map_err(|e| open_error(&e, path))?;
    let lower = path.to_lowercase();
    if lower.ends_with(".hex") || lower.ends_with(".elf") {
        return Err(Failure::Exit("give the .bin (Debug/mower_robot_firmware.bin), not .hex/.elf".into()));
    }
    if image.is_empty() {
        return Err(Failure::Exit("image is empty".into()));
    }
    if image.len() > APP_MAX_SIZE as usize {
        return Err(Failure::Exit(format!(
            "image is {} bytes, larger than the {}-byte app region",
            image.len(),
            APP_MAX_SIZE
        )));
    }
    if image.len() < 8 {
        // struct.unpack("<II", image[:8])
        return Err(Failure::Uncaught("struct.error: unpack requires a buffer of 8 bytes".into()));
    }
    let sp = u32::from_le_bytes([image[0], image[1], image[2], image[3]]);
    let pc = u32::from_le_bytes([image[4], image[5], image[6], image[7]]);
    if !(0x2000_0000 < sp && sp <= 0x2002_0000) || !((APP_START_ADDRESS..APP_END_ADDRESS).contains(&pc) && pc & 1 != 0) {
        return Err(Failure::Exit(format!(
            "this does not look like an app linked at 0x{APP_START_ADDRESS:08X} (SP=0x{sp:08X} PC=0x{pc:08X}); \
             rebuild with the updated linker script"
        )));
    }
    let pad = (4 - image.len() % 4) % 4;
    image.extend(std::iter::repeat_n(0xFF, pad));
    Ok(image)
}

/// `os.path.splitext(p)[0]` (posixpath: the last dot after the last slash,
/// not counting the dots a file name starts with).
pub fn splitext_root(p: &str) -> &str {
    let chars: Vec<(usize, char)> = p.char_indices().collect();
    let sep = chars.iter().rposition(|&(_, c)| c == '/');
    let dot = chars.iter().rposition(|&(_, c)| c == '.');
    if let Some(dot) = dot {
        if sep.is_none_or(|s| dot > s) {
            let first = sep.map_or(0, |s| s + 1);
            if chars[first..dot].iter().any(|&(_, c)| c != '.') {
                return &p[..chars[dot].0];
            }
        }
    }
    p
}

/// `open(path)` + `json.load(f)`: UTF-8, universal newlines, CPython's JSON.
pub fn load_manifest(image_path: &str, manifest_path: Option<&str>) -> Result<Py, Failure> {
    let path = match manifest_path {
        Some(p) => p.to_string(),
        None => format!("{}.json", splitext_root(image_path)),
    };
    let bytes = std::fs::read(&path).map_err(|e| open_error(&e, &path))?;
    let text = match std::str::from_utf8(&bytes) {
        Ok(t) => t,
        Err(e) => {
            // CPython's utf-8 decoder: the bad sequence is the lead byte and
            // the continuation bytes before the bad one, or everything to the
            // end of the data
            let at = e.valid_up_to();
            let (len, why) = match e.error_len() {
                None => (bytes.len() - at, "unexpected end of data"),
                Some(n) if matches!(bytes[at], 0x80..=0xC1 | 0xF5..=0xFF) => (n, "invalid start byte"),
                Some(n) => (n, "invalid continuation byte"),
            };
            let what = if len == 1 {
                format!("byte 0x{:02x} in position {at}", bytes[at])
            } else {
                format!("bytes in position {at}-{}", at + len - 1)
            };
            return Err(Failure::Uncaught(format!("UnicodeDecodeError: 'utf-8' codec can't decode {what}: {why}")));
        }
    };
    let text = text.replace("\r\n", "\n").replace('\r', "\n");
    pyjson::loads(&text).map_err(Failure::Uncaught)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splitext_like_posixpath() {
        for (p, root) in [
            ("a/b.bin", "a/b"),
            ("b.tar.gz", "b.tar"),
            (".bashrc", ".bashrc"),
            ("a/.b", "a/.b"),
            ("a.b/c", "a.b/c"),
            ("..x", "..x"),
            ("..x.bin", "..x"),
            ("x.", "x"),
            ("/opt/mower/firmware/mower_robot_firmware.bin", "/opt/mower/firmware/mower_robot_firmware"),
        ] {
            assert_eq!(splitext_root(p), root, "{p}");
        }
    }
}
