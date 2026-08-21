//! Wire codecs for Serika Social.
//!
//! The normative specification is `server/proto/pose_codec.md`. This module and the C#
//! implementation in the game client must agree byte-for-byte; `tests/golden.rs` is what
//! proves it. Read the spec before changing anything here.

pub mod pose;
pub mod voice;

pub use pose::{Lod, PoseFrame, HUMANOID_BONE_COUNT, LOD1_BONE_COUNT};
pub use voice::VoiceFrame;

/// Every way a frame can fail to decode.
///
/// Decoding is a trust boundary: these bytes come off the wire from a client that may be
/// hostile. Every failure is a rejection, never a fixup or a guess.
#[derive(Debug, thiserror::Error, PartialEq, Eq)]
pub enum DecodeError {
    #[error("frame too short: need {need} bytes, got {got}")]
    TooShort { need: usize, got: usize },
    #[error("unknown LOD {0}")]
    BadLod(u8),
    #[error("reserved flag bits set (0x{0:02x}); frame is from a newer protocol version")]
    ReservedBitsSet(u8),
    #[error("trailing garbage: {0} bytes past the end of the frame")]
    TrailingBytes(usize),
}
