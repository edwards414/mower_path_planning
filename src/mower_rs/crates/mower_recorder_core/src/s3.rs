//! The R2 (S3) client the uploader needs, written by hand over `ureq` (D8):
//! what the image's boto3 1.34.46 / botocore / s3transfer 0.10.1 send, as the
//! E5-0/E5-2 capture F4 recorded them (e2-verify `run.sh upload A`).
//!
//! - **Signing** ([`sign`]): botocore's `S3SigV4Auth`. Every header but
//!   `expect`, `user-agent` and `x-amzn-trace-id` is signed; the canonical
//!   URI is the path as sent (S3 is not normalized); the query is split on
//!   `&`, sorted, joined. The payload is signed (its sha256), except a
//!   PutObject or UploadPart over HTTPS, which carries `Content-MD5` and so
//!   gets `UNSIGNED-PAYLOAD` (`_should_sha256_sign_payload`).
//! - **Requests:** path-style URLs; the key percent-encoded but for `/~`;
//!   PutObject and UploadPart with `Content-MD5` (s3transfer computes it),
//!   metadata as `x-amz-meta-*`; the manifest with `Content-Type`;
//!   ListObjectsV2 with `encoding-type=url` (keys come back encoded).
//! - **upload_file** (s3transfer): one PutObject under 8 MiB; at 8 MiB and
//!   more CreateMultipartUpload (with the metadata), 8 MiB parts, at most 4
//!   in flight (s3transfer runs 10 threads; F4 saw 3 parts at once),
//!   CompleteMultipartUpload with boto3's XML, AbortMultipartUpload on a
//!   failure.
//! - **Retries:** botocore's legacy mode, 5 attempts per request, sleeping
//!   `rand × 2^(n−1)` s after the n-th: connection errors, 5xx, 429, 509 and
//!   the throttling, `RequestTimeout` and `BadDigest` codes.
//!
//! Not the same as boto3, and not observable to the app: the `User-Agent`,
//! botocore's unsigned `amz-sdk-*` headers and `Expect: 100-continue`.

use std::io::Read;
use std::path::Path;
use std::time::Duration;

use hmac::{Hmac, Mac};
use sha2::{Digest, Sha256};

use crate::util::{hex, sha256_hex};

pub const EMPTY_SHA256: &str = "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855";
pub const UNSIGNED_PAYLOAD: &str = "UNSIGNED-PAYLOAD";
/// s3transfer's `multipart_threshold` and `multipart_chunksize`.
pub const MULTIPART_THRESHOLD: u64 = 8 * 1024 * 1024;
pub const PART_SIZE: u64 = 8 * 1024 * 1024;
pub const PARTS_IN_FLIGHT: usize = 4;
pub const MAX_ATTEMPTS: u32 = 5;

/// An S3 error: botocore's `ClientError` text, `An error occurred (<Code>)
/// when calling the <Op> operation: <Message>`, or a transport error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct S3Error {
    pub operation: &'static str,
    /// `None` for a transport error (no answer).
    pub status: Option<u16>,
    pub code: String,
    pub message: String,
    /// Retries made before giving up (botocore's `reached max retries`).
    pub retries: u32,
}

impl S3Error {
    pub fn is_not_found(&self) -> bool {
        matches!(self.code.as_str(), "404" | "NoSuchKey" | "NotFound") || self.status == Some(404)
    }
}

impl std::fmt::Display for S3Error {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.status.is_none() {
            return write!(f, "{} ({})", self.message, self.operation);
        }
        let retries = if self.retries > 0 { format!(" (reached max retries: {})", self.retries) } else { String::new() };
        write!(f, "An error occurred ({}) when calling the {} operation{retries}: {}", self.code, self.operation, self.message)
    }
}

impl std::error::Error for S3Error {}

/// One page of ListObjectsV2.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct ListPage {
    pub keys: Vec<String>,
    pub is_truncated: bool,
    pub next_token: Option<String>,
}

/// What run_manifest calls on its client (boto3's method names).
pub trait S3 {
    /// s3transfer's `upload_file(path, bucket, key, ExtraArgs={'Metadata': ...})`.
    fn upload_file(&self, path: &Path, bucket: &str, key: &str, metadata: &[(String, String)]) -> Result<(), S3Error>;
    fn put_object(&self, bucket: &str, key: &str, body: &[u8], content_type: Option<&str>) -> Result<(), S3Error>;
    /// The ContentLength, or `None` when the object does not exist.
    fn head_object(&self, bucket: &str, key: &str) -> Result<Option<u64>, S3Error>;
    /// The body, or `None` when the object does not exist.
    fn get_object(&self, bucket: &str, key: &str) -> Result<Option<Vec<u8>>, S3Error>;
    fn list_objects_v2(&self, bucket: &str, prefix: &str, token: Option<&str>) -> Result<ListPage, S3Error>;
    fn delete_object(&self, bucket: &str, key: &str) -> Result<(), S3Error>;
}

// ---- SigV4 -----------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct Credentials {
    pub access_key: String,
    pub secret_key: String,
}

/// What [`sign`] needs: the request as it goes on the wire.
#[derive(Debug, Clone)]
pub struct SignInput<'a> {
    pub method: &'a str,
    /// The path as sent (already percent-encoded).
    pub path: &'a str,
    /// The query as sent (already encoded), without `?`.
    pub query: &'a str,
    /// Every header to send but `Authorization` (names in any case).
    pub headers: &'a [(String, String)],
    /// `x-amz-content-sha256`
    pub payload_hash: &'a str,
    /// `YYYYMMDDTHHMMSSZ`
    pub timestamp: &'a str,
    pub region: &'a str,
    pub service: &'a str,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Signed {
    pub canonical_request: String,
    pub string_to_sign: String,
    pub signature: String,
    pub authorization: String,
}

const UNSIGNED_HEADERS: [&str; 3] = ["expect", "user-agent", "x-amzn-trace-id"];

fn hmac(key: &[u8], msg: &str) -> Vec<u8> {
    let mut m = Hmac::<Sha256>::new_from_slice(key).expect("any key length");
    m.update(msg.as_bytes());
    m.finalize().into_bytes().to_vec()
}

/// botocore `S3SigV4Auth.add_auth`, minus the clock (`timestamp` is given).
pub fn sign(cred: &Credentials, r: &SignInput) -> Signed {
    // headers_to_sign: lower-cased, the unsigned ones dropped
    let mut hs: Vec<(String, String)> = r
        .headers
        .iter()
        .map(|(k, v)| (k.to_ascii_lowercase(), v.clone()))
        .filter(|(k, _)| !UNSIGNED_HEADERS.contains(&k.as_str()))
        .collect();
    hs.sort();
    let mut names: Vec<String> = hs.iter().map(|(k, _)| k.clone()).collect();
    names.dedup();
    // canonical_headers: values of a name sorted and joined with ',', each
    // with its runs of whitespace collapsed
    let mut canon_headers = Vec::new();
    for n in &names {
        let mut vals: Vec<String> =
            hs.iter().filter(|(k, _)| k == n).map(|(_, v)| v.split_whitespace().collect::<Vec<_>>().join(" ")).collect();
        vals.sort();
        canon_headers.push(format!("{n}:{}", vals.join(",")));
    }
    let signed_headers = names.join(";");
    // _canonical_query_string_url
    let mut pairs: Vec<(String, String)> = if r.query.is_empty() {
        Vec::new()
    } else {
        r.query
            .split('&')
            .map(|p| match p.split_once('=') {
                Some((k, v)) => (k.to_string(), v.to_string()),
                None => (p.to_string(), String::new()),
            })
            .collect()
    };
    pairs.sort();
    let query = pairs.iter().map(|(k, v)| format!("{k}={v}")).collect::<Vec<_>>().join("&");
    let canonical_request = [
        r.method.to_ascii_uppercase(),
        r.path.to_string(),
        query,
        canon_headers.join("\n") + "\n",
        signed_headers.clone(),
        r.payload_hash.to_string(),
    ]
    .join("\n");
    let date = &r.timestamp[..8];
    let scope = format!("{date}/{}/{}/aws4_request", r.region, r.service);
    let string_to_sign =
        format!("AWS4-HMAC-SHA256\n{}\n{scope}\n{}", r.timestamp, sha256_hex(canonical_request.as_bytes()));
    let k_date = hmac(format!("AWS4{}", cred.secret_key).as_bytes(), date);
    let k_region = hmac(&k_date, r.region);
    let k_service = hmac(&k_region, r.service);
    let k_signing = hmac(&k_service, "aws4_request");
    let signature = hex(&hmac(&k_signing, &string_to_sign));
    let authorization = format!(
        "AWS4-HMAC-SHA256 Credential={}/{scope}, SignedHeaders={signed_headers}, Signature={signature}",
        cred.access_key
    );
    Signed { canonical_request, string_to_sign, signature, authorization }
}

/// `urllib.parse.quote(s, safe=...)`: unreserved ASCII and `safe` kept.
pub fn quote(s: &str, safe: &str) -> String {
    let mut out = String::new();
    for b in s.bytes() {
        if b.is_ascii_alphanumeric() || b"_.-~".contains(&b) || safe.as_bytes().contains(&b) {
            out.push(b as char);
        } else {
            out.push_str(&format!("%{b:02X}"));
        }
    }
    out
}

/// `urllib.parse.unquote` of ASCII escapes (what `encoding-type=url` gives).
pub fn unquote(s: &str) -> String {
    let b = s.as_bytes();
    let mut out = Vec::with_capacity(b.len());
    let mut i = 0;
    while i < b.len() {
        if b[i] == b'%' && i + 2 < b.len() {
            if let Ok(v) = u8::from_str_radix(std::str::from_utf8(&b[i + 1..i + 3]).unwrap_or("zz"), 16) {
                out.push(v);
                i += 3;
                continue;
            }
        }
        out.push(b[i]);
        i += 1;
    }
    String::from_utf8_lossy(&out).into_owned()
}

/// `YYYYMMDDTHHMMSSZ` of now (UTC).
pub fn amz_timestamp(now: std::time::SystemTime) -> String {
    let secs = now.duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_secs() as i64;
    let days = secs.div_euclid(86_400);
    let rem = secs.rem_euclid(86_400);
    let (y, m, d) = civil_from_days(days);
    format!("{y:04}{m:02}{d:02}T{:02}{:02}{:02}Z", rem / 3600, (rem % 3600) / 60, rem % 60)
}

/// Howard Hinnant's days-to-civil.
fn civil_from_days(z: i64) -> (i64, u32, u32) {
    let z = z + 719_468;
    let era = z.div_euclid(146_097);
    let doe = z.rem_euclid(146_097);
    let yoe = (doe - doe / 1460 + doe / 36_524 - doe / 146_096) / 365;
    let y = yoe + era * 400;
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    let mp = (5 * doy + 2) / 153;
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32;
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32;
    (if m <= 2 { y + 1 } else { y }, m, d)
}

// ---- the client --------------------------------------------------------------

/// R2 (or any S3) over HTTPS, path style, region `auto`.
pub struct Client {
    agent: ureq::Agent,
    /// `https://<account>.r2.cloudflarestorage.com`, no trailing `/`
    endpoint: String,
    host: String,
    https: bool,
    cred: Credentials,
    region: String,
    /// A test may freeze the clock.
    pub clock: fn() -> std::time::SystemTime,
    /// A test may skip the back-off sleeps.
    pub sleep: fn(Duration),
}

/// One S3 request before it is signed (what [`Client::prepare`] signs).
pub struct Request<'a> {
    pub op: &'static str,
    pub method: &'a str,
    pub bucket: &'a str,
    pub key: Option<&'a str>,
    pub query: String,
    pub headers: Vec<(String, String)>,
    pub body: &'a [u8],
    /// PutObject / UploadPart: a streaming body with its Content-MD5
    pub unsigned_ok: bool,
}

/// A request as it goes on the wire.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Prepared {
    pub method: String,
    pub url: String,
    /// Every header but `User-Agent`, `Authorization` last.
    pub headers: Vec<(String, String)>,
    pub signed: Signed,
}

struct Resp {
    status: u16,
    headers: Vec<(String, String)>,
    body: Vec<u8>,
}

fn md5_b64(data: &[u8]) -> String {
    use base64::Engine;
    use md5::Md5;
    base64::engine::general_purpose::STANDARD.encode(Md5::digest(data))
}

impl Client {
    pub fn new(endpoint: &str, cred: Credentials) -> Client {
        let endpoint = endpoint.trim_end_matches('/').to_string();
        let https = endpoint.starts_with("https://");
        let host = endpoint.split("://").nth(1).unwrap_or("").split('/').next().unwrap_or("").to_string();
        // botocore's defaults: 60 s connect and read
        let agent = ureq::AgentBuilder::new().timeout_connect(Duration::from_secs(60)).timeout_read(Duration::from_secs(60)).build();
        Client {
            agent,
            endpoint,
            host,
            https,
            cred,
            region: "auto".into(),
            clock: std::time::SystemTime::now,
            sleep: std::thread::sleep,
        }
    }

    /// bag_store's endpoint for an R2 account.
    pub fn r2(account_id: &str, cred: Credentials) -> Client {
        Client::new(&format!("https://{account_id}.r2.cloudflarestorage.com"), cred)
    }

    fn path(bucket: &str, key: Option<&str>) -> String {
        match key {
            Some(k) => format!("/{bucket}/{}", quote(k, "/~")),
            None => format!("/{bucket}"),
        }
    }

    /// Signs a request: the URL, the headers botocore would send, the
    /// signature.
    pub fn prepare(&self, r: &Request) -> Prepared {
        let path = Self::path(r.bucket, r.key);
        let payload = if r.unsigned_ok && self.https { UNSIGNED_PAYLOAD.to_string() } else { sha256_hex(r.body) };
        let ts = amz_timestamp((self.clock)());
        let mut headers = vec![("Host".to_string(), self.host.clone())];
        headers.extend(r.headers.iter().cloned());
        headers.push(("X-Amz-Date".into(), ts.clone()));
        headers.push(("X-Amz-Content-SHA256".into(), payload.clone()));
        let signed = sign(
            &self.cred,
            &SignInput {
                method: r.method,
                path: &path,
                query: &r.query,
                headers: &headers,
                payload_hash: &payload,
                timestamp: &ts,
                region: &self.region,
                service: "s3",
            },
        );
        headers.push(("Authorization".into(), signed.authorization.clone()));
        let url = if r.query.is_empty() { format!("{}{path}", self.endpoint) } else { format!("{}{path}?{}", self.endpoint, r.query) };
        Prepared { method: r.method.to_string(), url, headers, signed }
    }

    fn once(&self, r: &Request) -> Result<Resp, S3Error> {
        let p = self.prepare(r);
        let mut req = self.agent.request(&p.method, &p.url);
        for (k, v) in p.headers.iter().filter(|(k, _)| k != "Host") {
            req = req.set(k, v);
        }
        req = req.set("User-Agent", concat!("mower_recorder_core/", env!("CARGO_PKG_VERSION")));
        let res = if r.body.is_empty() && matches!(r.method, "GET" | "HEAD" | "DELETE") { req.call() } else { req.send_bytes(r.body) };
        let resp = match res {
            Ok(resp) | Err(ureq::Error::Status(_, resp)) => resp,
            Err(ureq::Error::Transport(t)) => {
                return Err(S3Error { operation: r.op, status: None, code: String::new(), message: t.to_string(), retries: 0 });
            }
        };
        let status = resp.status();
        let headers: Vec<(String, String)> =
            resp.headers_names().into_iter().map(|n| (n.to_ascii_lowercase(), resp.header(&n).unwrap_or("").to_string())).collect();
        let mut body = Vec::new();
        if r.method != "HEAD" {
            resp.into_reader().take(1 << 30).read_to_end(&mut body).map_err(|e| S3Error {
                operation: r.op,
                status: None,
                code: String::new(),
                message: e.to_string(),
                retries: 0,
            })?;
        }
        if status >= 300 {
            let text = String::from_utf8_lossy(&body);
            let code = xml_text(&text, "Code").unwrap_or_else(|| status.to_string());
            // botocore's message for an error without a body: the reason phrase
            let message = xml_text(&text, "Message").unwrap_or_else(|| reason(status).to_string());
            return Err(S3Error { operation: r.op, status: Some(status), code, message, retries: 0 });
        }
        Ok(Resp { status, headers, body })
    }

    /// One request with botocore's legacy retries.
    fn call(&self, r: Request) -> Result<Resp, S3Error> {
        let mut attempt = 1;
        loop {
            match self.once(&r) {
                Ok(resp) => return Ok(resp),
                Err(e) if attempt < MAX_ATTEMPTS && retryable(&e) => {
                    (self.sleep)(Duration::from_secs_f64(jitter() * 2f64.powi(attempt as i32 - 1)));
                    attempt += 1;
                }
                Err(mut e) => {
                    e.retries = attempt - 1;
                    return Err(e);
                }
            }
        }
    }

    fn meta_headers(metadata: &[(String, String)]) -> Vec<(String, String)> {
        metadata.iter().map(|(k, v)| (format!("x-amz-meta-{k}"), v.clone())).collect()
    }

    pub fn create_multipart_upload_request<'a>(bucket: &'a str, key: &'a str, metadata: &[(String, String)]) -> Request<'a> {
        Request {
            op: "CreateMultipartUpload",
            method: "POST",
            bucket,
            key: Some(key),
            query: "uploads".into(),
            headers: Self::meta_headers(metadata),
            body: b"",
            unsigned_ok: false,
        }
    }

    pub fn create_multipart_upload(&self, bucket: &str, key: &str, metadata: &[(String, String)]) -> Result<String, S3Error> {
        let r = self.call(Self::create_multipart_upload_request(bucket, key, metadata))?;
        xml_text(&String::from_utf8_lossy(&r.body), "UploadId").ok_or_else(|| S3Error {
            operation: "CreateMultipartUpload",
            status: Some(r.status),
            code: "MalformedXML".into(),
            message: "no UploadId".into(),
            retries: 0,
        })
    }

    pub fn upload_part_request<'a>(bucket: &'a str, key: &'a str, upload_id: &str, part: u32, data: &'a [u8]) -> Request<'a> {
        Request {
            op: "UploadPart",
            method: "PUT",
            bucket,
            key: Some(key),
            query: format!("uploadId={}&partNumber={part}", quote(upload_id, "")),
            headers: vec![("Content-MD5".into(), md5_b64(data))],
            body: data,
            unsigned_ok: true,
        }
    }

    pub fn upload_part(&self, bucket: &str, key: &str, upload_id: &str, part: u32, data: &[u8]) -> Result<String, S3Error> {
        let r = self.call(Self::upload_part_request(bucket, key, upload_id, part, data))?;
        Ok(r.headers.iter().find(|(k, _)| k == "etag").map(|(_, v)| v.clone()).unwrap_or_default())
    }

    /// boto3's CompleteMultipartUpload body.
    pub fn complete_xml(etags: &[String]) -> String {
        let mut xml = String::from("<CompleteMultipartUpload xmlns=\"http://s3.amazonaws.com/doc/2006-03-01/\">");
        for (i, e) in etags.iter().enumerate() {
            xml.push_str(&format!("<Part><ETag>{}</ETag><PartNumber>{}</PartNumber></Part>", xml_escape(e), i + 1));
        }
        xml.push_str("</CompleteMultipartUpload>");
        xml
    }

    pub fn complete_multipart_upload_request<'a>(bucket: &'a str, key: &'a str, upload_id: &str, xml: &'a str) -> Request<'a> {
        Request {
            op: "CompleteMultipartUpload",
            method: "POST",
            bucket,
            key: Some(key),
            query: format!("uploadId={}", quote(upload_id, "")),
            headers: Vec::new(),
            body: xml.as_bytes(),
            unsigned_ok: false,
        }
    }

    pub fn complete_multipart_upload(&self, bucket: &str, key: &str, upload_id: &str, etags: &[String]) -> Result<(), S3Error> {
        let xml = Self::complete_xml(etags);
        let r = self.call(Self::complete_multipart_upload_request(bucket, key, upload_id, &xml))?;
        // S3 can answer 200 with an error document
        let text = String::from_utf8_lossy(&r.body);
        if text.contains("<Error>") {
            return Err(S3Error {
                operation: "CompleteMultipartUpload",
                status: Some(r.status),
                code: xml_text(&text, "Code").unwrap_or_default(),
                message: xml_text(&text, "Message").unwrap_or_default(),
                retries: 0,
            });
        }
        Ok(())
    }

    pub fn abort_multipart_upload_request<'a>(bucket: &'a str, key: &'a str, upload_id: &str) -> Request<'a> {
        Request {
            op: "AbortMultipartUpload",
            method: "DELETE",
            bucket,
            key: Some(key),
            query: format!("uploadId={}", quote(upload_id, "")),
            headers: Vec::new(),
            body: b"",
            unsigned_ok: false,
        }
    }

    pub fn abort_multipart_upload(&self, bucket: &str, key: &str, upload_id: &str) -> Result<(), S3Error> {
        self.call(Self::abort_multipart_upload_request(bucket, key, upload_id)).map(|_| ())
    }

    pub fn put_object_request<'a>(
        bucket: &'a str,
        key: &'a str,
        body: &'a [u8],
        content_type: Option<&str>,
        metadata: &[(String, String)],
    ) -> Request<'a> {
        let mut headers = Self::meta_headers(metadata);
        if let Some(ct) = content_type {
            headers.push(("Content-Type".into(), ct.into()));
        }
        headers.push(("Content-MD5".into(), md5_b64(body)));
        Request { op: "PutObject", method: "PUT", bucket, key: Some(key), query: String::new(), headers, body, unsigned_ok: true }
    }

    pub fn put_object_with(
        &self,
        bucket: &str,
        key: &str,
        body: &[u8],
        content_type: Option<&str>,
        metadata: &[(String, String)],
    ) -> Result<(), S3Error> {
        self.call(Self::put_object_request(bucket, key, body, content_type, metadata)).map(|_| ())
    }

    pub fn head_object_request<'a>(bucket: &'a str, key: &'a str) -> Request<'a> {
        Request { op: "HeadObject", method: "HEAD", bucket, key: Some(key), query: String::new(), headers: Vec::new(), body: b"", unsigned_ok: false }
    }

    pub fn get_object_request<'a>(bucket: &'a str, key: &'a str) -> Request<'a> {
        Request { op: "GetObject", method: "GET", bucket, key: Some(key), query: String::new(), headers: Vec::new(), body: b"", unsigned_ok: false }
    }

    pub fn delete_object_request<'a>(bucket: &'a str, key: &'a str) -> Request<'a> {
        Request { op: "DeleteObject", method: "DELETE", bucket, key: Some(key), query: String::new(), headers: Vec::new(), body: b"", unsigned_ok: false }
    }

    /// ListObjectsV2 as boto3 orders its query.
    pub fn list_objects_v2_request<'a>(bucket: &'a str, prefix: &str, token: Option<&str>) -> Request<'a> {
        let mut query = format!("list-type=2&prefix={}", quote(prefix, ""));
        if let Some(t) = token {
            query.push_str(&format!("&continuation-token={}", quote(t, "")));
        }
        query.push_str("&encoding-type=url");
        Request { op: "ListObjectsV2", method: "GET", bucket, key: None, query, headers: Vec::new(), body: b"", unsigned_ok: false }
    }
}

fn reason(status: u16) -> &'static str {
    match status {
        400 => "Bad Request",
        403 => "Forbidden",
        404 => "Not Found",
        500 => "Internal Server Error",
        502 => "Bad Gateway",
        503 => "Service Unavailable",
        504 => "Gateway Timeout",
        _ => "",
    }
}

/// botocore's legacy retry rules (`_retry.json`, general + s3).
fn retryable(e: &S3Error) -> bool {
    match e.status {
        None => true,
        Some(500 | 502 | 503 | 504 | 509 | 429) => true,
        Some(_) => matches!(
            e.code.as_str(),
            "Throttling" | "ThrottlingException" | "ThrottledException" | "RequestThrottledException"
                | "ProvisionedThroughputExceededException" | "TransactionInProgressException" | "RequestLimitExceeded"
                | "BandwidthLimitExceeded" | "LimitExceededException" | "RequestThrottled" | "SlowDown"
                | "PriorRequestNotComplete" | "EC2ThrottledException" | "RequestTimeout" | "BadDigest"
        ),
    }
}

/// A number in [0, 1) for the back-off (no need for a good generator).
fn jitter() -> f64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static STATE: AtomicU64 = AtomicU64::new(0);
    let mut x = STATE.load(Ordering::Relaxed);
    if x == 0 {
        x = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap_or_default().as_nanos() as u64 | 1;
    }
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    STATE.store(x, Ordering::Relaxed);
    (x >> 11) as f64 / (1u64 << 53) as f64
}

/// Text escaping as botocore's serializer (ElementTree) does it: quotes stay.
fn xml_escape(s: &str) -> String {
    s.replace('&', "&amp;").replace('<', "&lt;").replace('>', "&gt;")
}

fn xml_unescape(s: &str) -> String {
    s.replace("&quot;", "\"").replace("&lt;", "<").replace("&gt;", ">").replace("&apos;", "'").replace("&amp;", "&")
}

/// The text of the first `<tag>` (S3's flat answers need no more).
fn xml_text(xml: &str, tag: &str) -> Option<String> {
    xml_all(xml, tag).into_iter().next()
}

fn xml_all(xml: &str, tag: &str) -> Vec<String> {
    xml_raw(xml, tag).into_iter().map(xml_unescape).collect()
}

/// The raw contents of every `<tag>...</tag>`.
fn xml_raw<'a>(xml: &'a str, tag: &str) -> Vec<&'a str> {
    let (open, close) = (format!("<{tag}>"), format!("</{tag}>"));
    let mut out = Vec::new();
    let mut rest = xml;
    while let Some(i) = rest.find(&open) {
        let after = &rest[i + open.len()..];
        let Some(j) = after.find(&close) else { break };
        out.push(&after[..j]);
        rest = &after[j + close.len()..];
    }
    out
}

impl S3 for Client {
    fn upload_file(&self, path: &Path, bucket: &str, key: &str, metadata: &[(String, String)]) -> Result<(), S3Error> {
        let io = |e: std::io::Error| S3Error { operation: "PutObject", status: None, code: String::new(), message: e.to_string(), retries: 0 };
        let size = std::fs::metadata(path).map_err(io)?.len();
        if size < MULTIPART_THRESHOLD {
            let body = std::fs::read(path).map_err(io)?;
            return self.put_object_with(bucket, key, &body, None, metadata);
        }
        let upload_id = self.create_multipart_upload(bucket, key, metadata)?;
        let parts = size.div_ceil(PART_SIZE) as u32;
        let result: Result<Vec<String>, S3Error> = (|| {
            let mut etags = vec![String::new(); parts as usize];
            let mut next = 1u32;
            while next <= parts {
                let batch: Vec<u32> = (next..=parts).take(PARTS_IN_FLIGHT).collect();
                next += batch.len() as u32;
                let results: Vec<Result<(u32, String), S3Error>> = std::thread::scope(|s| {
                    let handles: Vec<_> = batch
                        .iter()
                        .map(|&n| {
                            let upload_id = &upload_id;
                            s.spawn(move || {
                                let mut f = std::fs::File::open(path).map_err(io)?;
                                use std::io::Seek;
                                f.seek(std::io::SeekFrom::Start((n as u64 - 1) * PART_SIZE)).map_err(io)?;
                                let mut buf = Vec::with_capacity(PART_SIZE as usize);
                                f.take(PART_SIZE).read_to_end(&mut buf).map_err(io)?;
                                self.upload_part(bucket, key, upload_id, n, &buf).map(|e| (n, e))
                            })
                        })
                        .collect();
                    handles.into_iter().map(|h| h.join().expect("an upload thread panicked")).collect()
                });
                for r in results {
                    let (n, etag) = r?;
                    etags[n as usize - 1] = etag;
                }
            }
            Ok(etags)
        })();
        match result.and_then(|etags| self.complete_multipart_upload(bucket, key, &upload_id, &etags)) {
            Ok(()) => Ok(()),
            Err(e) => {
                let _ = self.abort_multipart_upload(bucket, key, &upload_id);
                Err(e)
            }
        }
    }

    fn put_object(&self, bucket: &str, key: &str, body: &[u8], content_type: Option<&str>) -> Result<(), S3Error> {
        self.put_object_with(bucket, key, body, content_type, &[])
    }

    fn head_object(&self, bucket: &str, key: &str) -> Result<Option<u64>, S3Error> {
        let r = self.call(Self::head_object_request(bucket, key));
        match r {
            Ok(r) => Ok(r.headers.iter().find(|(k, _)| k == "content-length").and_then(|(_, v)| v.parse().ok())),
            Err(e) if e.is_not_found() => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn get_object(&self, bucket: &str, key: &str) -> Result<Option<Vec<u8>>, S3Error> {
        let r = self.call(Self::get_object_request(bucket, key));
        match r {
            Ok(r) => Ok(Some(r.body)),
            Err(e) if e.is_not_found() => Ok(None),
            Err(e) => Err(e),
        }
    }

    fn list_objects_v2(&self, bucket: &str, prefix: &str, token: Option<&str>) -> Result<ListPage, S3Error> {
        let r = self.call(Self::list_objects_v2_request(bucket, prefix, token))?;
        let text = String::from_utf8_lossy(&r.body).into_owned();
        let encoded = xml_text(&text, "EncodingType").as_deref() == Some("url");
        let mut keys = Vec::new();
        for contents in xml_raw(&text, "Contents") {
            if let Some(k) = xml_raw(contents, "Key").first() {
                let k = xml_unescape(k);
                keys.push(if encoded { unquote(&k) } else { k });
            }
        }
        Ok(ListPage {
            keys,
            is_truncated: xml_text(&text, "IsTruncated").as_deref() == Some("true"),
            next_token: xml_text(&text, "NextContinuationToken"),
        })
    }

    fn delete_object(&self, bucket: &str, key: &str) -> Result<(), S3Error> {
        self.call(Self::delete_object_request(bucket, key)).map(|_| ())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn quoting_as_botocore_does() {
        assert_eq!(quote("bags/MW 1/a+b~c.mcap", "/~"), "bags/MW%201/a%2Bb~c.mcap");
        assert_eq!(quote("bags/x/", ""), "bags%2Fx%2F");
        assert_eq!(unquote("bags/a%20b%2Bc"), "bags/a b+c");
    }

    #[test]
    fn timestamps() {
        let t = std::time::UNIX_EPOCH + Duration::from_secs(1_793_348_331);
        assert_eq!(amz_timestamp(t), "20261030T081851Z");
    }

    #[test]
    fn xml_fields() {
        let x = "<R><IsTruncated>true</IsTruncated><Contents><Key>a%2Fb</Key></Contents><NextContinuationToken>t&amp;1</NextContinuationToken></R>";
        assert_eq!(xml_text(x, "NextContinuationToken").as_deref(), Some("t&1"));
        assert_eq!(xml_all(x, "Contents").len(), 1);
    }
}
