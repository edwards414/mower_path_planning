//! The S3 client against botocore itself (G5's "SigV4 against botocore's
//! signer for fixed time and keys"): `tests/vectors/sigv4.json` holds every
//! request the image's boto3 1.34.46 sends for what the uploader does, over
//! HTTP (as to moto) and HTTPS (R2), with the clock frozen
//! (`tests/oracle/sigv4_gen.py`). For each:
//! - [`sign`] on botocore's own inputs gives botocore's canonical request,
//!   string to sign and `Authorization`;
//! - the client's own request for the same operation, signed by
//!   [`Client::prepare`], has the same URL, the same signed headers and so
//!   the same `Authorization`.

use mower_recorder_core::s3::{sign, Client, Credentials, Request, SignInput};
use serde_json::Value;

fn vectors() -> Value {
    let p = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/vectors/sigv4.json");
    serde_json::from_str(&std::fs::read_to_string(p).unwrap()).unwrap()
}

fn header<'a>(v: &'a Value, name: &str) -> Option<&'a str> {
    v["headers"].as_array().unwrap().iter().find(|h| h[0].as_str().unwrap().eq_ignore_ascii_case(name)).map(|h| h[1].as_str().unwrap())
}

fn frozen() -> std::time::SystemTime {
    // 2026-10-30T08:18:51Z
    std::time::UNIX_EPOCH + std::time::Duration::from_secs(1_793_348_331)
}

/// The 9 MiB file of the generator.
fn big() -> Vec<u8> {
    use sha2::Digest;
    let mut out = Vec::with_capacity(9 * 1024 * 1024);
    for i in 0u32..(9 * 1024 * 1024 / 32) {
        out.extend_from_slice(&sha2::Sha256::digest(i.to_le_bytes()));
    }
    out
}

#[test]
fn sign_reproduces_botocore_on_its_own_inputs() {
    let doc = vectors();
    let cred = Credentials { access_key: doc["access_key"].as_str().unwrap().into(), secret_key: doc["secret_key"].as_str().unwrap().into() };
    let vs = doc["vectors"].as_array().unwrap();
    assert_eq!(vs.len(), 24);
    for v in vs {
        let url = v["url"].as_str().unwrap();
        let after_scheme = url.split("://").nth(1).unwrap();
        let path_q = &after_scheme[after_scheme.find('/').unwrap()..];
        let (path, query) = path_q.split_once('?').unwrap_or((path_q, ""));
        let auth = header(v, "authorization").unwrap();
        let signed: Vec<&str> = auth.split("SignedHeaders=").nth(1).unwrap().split(',').next().unwrap().split(';').collect();
        // what botocore had when it signed: the signed headers (and the
        // User-Agent, which must stay out)
        let mut headers: Vec<(String, String)> = v["headers"]
            .as_array()
            .unwrap()
            .iter()
            .map(|h| (h[0].as_str().unwrap().to_string(), h[1].as_str().unwrap().to_string()))
            .filter(|(k, _)| signed.contains(&k.to_ascii_lowercase().as_str()) || k.eq_ignore_ascii_case("user-agent"))
            .collect();
        if !headers.iter().any(|(k, _)| k.eq_ignore_ascii_case("host")) {
            headers.push(("host".into(), after_scheme.split('/').next().unwrap().into()));
        }
        let s = sign(
            &cred,
            &SignInput {
                method: v["method"].as_str().unwrap(),
                path,
                query,
                headers: &headers,
                payload_hash: header(v, "x-amz-content-sha256").unwrap(),
                timestamp: doc["timestamp"].as_str().unwrap(),
                region: "auto",
                service: "s3",
            },
        );
        let ctx = format!("{} {} #{}", v["endpoint"], v["step"], v["index"]);
        assert_eq!(s.canonical_request, v["canonical_request"].as_str().unwrap(), "{ctx}");
        assert_eq!(s.string_to_sign, v["string_to_sign"].as_str().unwrap(), "{ctx}");
        assert_eq!(s.authorization, auth, "{ctx}");
    }
}

#[test]
fn the_client_signs_what_boto3_sends() {
    let doc = vectors();
    let cred = Credentials { access_key: doc["access_key"].as_str().unwrap().into(), secret_key: doc["secret_key"].as_str().unwrap().into() };
    let big = big();
    let small = b"run_id: MW-E5FIX_20261030T081851\n".to_vec();
    let manifest = b"{\n  \"schema\": \"x\"\n}\n".to_vec();
    let md5_hex = |d: &[u8]| {
        use md5::Digest;
        format!("\"{}\"", mower_recorder_core::util::hex(&md5::Md5::digest(d)))
    };
    let parts = [&big[..8 * 1024 * 1024], &big[8 * 1024 * 1024..]];
    let xml = Client::complete_xml(&parts.iter().map(|p| md5_hex(p)).collect::<Vec<_>>());
    let meta = |c: &str| vec![("sha256".to_string(), c.repeat(32))];
    let (b, small_key, big_key, r) =
        ("mower-bags", "bags/MW-E5FIX/MW-E5FIX_20261030T081851/run_metadata.yaml", "bags/MW-E5FIX/run 1/video/front ü+~.mp4", "bags/MW-E5FIX/r/");
    let (mkey, bag_key, vkey) = ("bags/MW-E5FIX/r/_manifest.json", "bags/MW-E5FIX/r/bag/bag_0.mcap", "bags/MW-E5FIX/r/video/x.mp4");
    let mut checked = 0;
    for v in doc["vectors"].as_array().unwrap() {
        let endpoint = v["endpoint"].as_str().unwrap();
        let mut client = Client::new(endpoint, cred.clone());
        client.clock = frozen;
        let req: Request = match (v["step"].as_str().unwrap(), v["index"].as_u64().unwrap()) {
            ("upload_file small", 0) => Client::put_object_request(b, small_key, &small, None, &meta("ab")),
            ("upload_file big", 0) => Client::create_multipart_upload_request(b, big_key, &meta("cd")),
            ("upload_file big", i @ (1 | 2)) => Client::upload_part_request(b, big_key, "e5-upload-id/1", i as u32, parts[i as usize - 1]),
            ("upload_file big", 3) => Client::complete_multipart_upload_request(b, big_key, "e5-upload-id/1", &xml),
            ("put_object manifest", 0) => Client::put_object_request(b, mkey, &manifest, Some("application/json"), &[]),
            ("head_object", 0) => Client::head_object_request(b, bag_key),
            ("get_object", 0) => Client::get_object_request(b, mkey),
            ("list_objects_v2", 0) => Client::list_objects_v2_request(b, r, None),
            ("list_objects_v2 token", 0) => Client::list_objects_v2_request(b, r, Some("1/a+b=c")),
            ("delete_object", 0) => Client::delete_object_request(b, mkey),
            ("abort", 0) => Client::abort_multipart_upload_request(b, vkey, "e5-upload-id/1"),
            other => panic!("no request for {other:?}"),
        };
        let p = client.prepare(&req);
        let ctx = format!("{endpoint} {} #{}", v["step"], v["index"]);
        assert_eq!(p.url, v["url"].as_str().unwrap(), "{ctx}");
        assert_eq!(p.signed.authorization, header(v, "authorization").unwrap(), "{ctx}");
        if let Some(body) = v["body"].as_str() {
            assert_eq!(std::str::from_utf8(req.body).unwrap(), body, "{ctx}");
        }
        assert_eq!(mower_recorder_core::util::sha256_hex(req.body), v["body_sha256"].as_str().unwrap(), "{ctx}");
        checked += 1;
    }
    assert_eq!(checked, 24);
}
