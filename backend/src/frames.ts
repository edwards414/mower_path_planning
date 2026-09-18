// mrelay1 framing (docs/BACKEND_ARCHITECTURE.md §5).
//
// app <-> hub :  [type][payload]
// hub <-> robot: [type][sid: 8 bytes][payload]
//
// type: 0x01 text final, 0x11 text more, 0x02 binary final, 0x12 binary more.
// The hub never reassembles: it only adds or strips the session id.

export const SUBPROTOCOL = "mrelay1";
export const SID_BYTES = 8;
export const T_TEXT = 0x01;
export const T_TEXT_MORE = 0x11;
export const T_BIN = 0x02;
export const T_BIN_MORE = 0x12;

export function isFrameType(t: number): boolean {
  return t === T_TEXT || t === T_TEXT_MORE || t === T_BIN || t === T_BIN_MORE;
}

export function sidToBytes(sidHex: string): Uint8Array {
  const out = new Uint8Array(SID_BYTES);
  for (let i = 0; i < SID_BYTES; i++) out[i] = parseInt(sidHex.slice(i * 2, i * 2 + 2), 16);
  return out;
}

export function bytesToSid(b: Uint8Array): string {
  let s = "";
  for (let i = 0; i < SID_BYTES; i++) s += b[i].toString(16).padStart(2, "0");
  return s;
}

/** app frame (or a plain text message) -> robot frame with sid prepended. */
export function appToRobot(msg: string | ArrayBuffer, sidHex: string): Uint8Array | null {
  let type: number;
  let payload: Uint8Array;
  if (typeof msg === "string") {
    type = T_TEXT;
    payload = new TextEncoder().encode(msg);
  } else {
    const v = new Uint8Array(msg);
    if (v.byteLength < 1 || !isFrameType(v[0])) return null;
    type = v[0];
    payload = v.subarray(1);
  }
  const out = new Uint8Array(1 + SID_BYTES + payload.byteLength);
  out[0] = type;
  out.set(sidToBytes(sidHex), 1);
  out.set(payload, 1 + SID_BYTES);
  return out;
}

/** robot frame -> { sid, frame for the app } or null when malformed. */
export function robotToApp(msg: ArrayBuffer): { sid: string; frame: Uint8Array } | null {
  const v = new Uint8Array(msg);
  if (v.byteLength < 1 + SID_BYTES || !isFrameType(v[0])) return null;
  const sid = bytesToSid(v.subarray(1, 1 + SID_BYTES));
  const frame = new Uint8Array(1 + v.byteLength - 1 - SID_BYTES);
  frame[0] = v[0];
  frame.set(v.subarray(1 + SID_BYTES), 1);
  return { sid, frame };
}
