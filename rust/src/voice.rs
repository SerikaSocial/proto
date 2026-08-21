//! Opus frame framing. See `server/proto/pose_codec.md` §Voice frames.
//!
//! The relay never decodes audio — it reads the header to route and rank, and forwards the
//! payload untouched. That keeps per-instance CPU near zero and leaves the door open for
//! end-to-end encryption of the payload later.

use crate::DecodeError;

const HEADER_LEN: usize = 4;

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VoiceFrame<'a> {
    pub sequence: u8,
    /// Client-reported loudness, 0-255, used for speaker ranking.
    ///
    /// **Untrusted.** A client can claim to be loud to win a top-N slot; the relay rate
    /// limits how often any one peer may hold one. Ranking is fairness, not security.
    pub rms: u8,
    pub payload: &'a [u8],
}

impl<'a> VoiceFrame<'a> {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(HEADER_LEN + self.payload.len());
        out.push(self.sequence);
        out.push(self.rms);
        out.extend_from_slice(&(self.payload.len() as u16).to_le_bytes());
        out.extend_from_slice(self.payload);
        out
    }

    pub fn decode(buf: &'a [u8]) -> Result<Self, DecodeError> {
        if buf.len() < HEADER_LEN {
            return Err(DecodeError::TooShort { need: HEADER_LEN, got: buf.len() });
        }
        let len = u16::from_le_bytes([buf[2], buf[3]]) as usize;
        let want = HEADER_LEN + len;
        if buf.len() < want {
            return Err(DecodeError::TooShort { need: want, got: buf.len() });
        }
        if buf.len() > want {
            return Err(DecodeError::TrailingBytes(buf.len() - want));
        }
        Ok(VoiceFrame { sequence: buf[0], rms: buf[1], payload: &buf[HEADER_LEN..want] })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn roundtrips() {
        let payload = [0xDEu8, 0xAD, 0xBE, 0xEF];
        let frame = VoiceFrame { sequence: 42, rms: 200, payload: &payload };
        let bytes = frame.encode();
        assert_eq!(VoiceFrame::decode(&bytes).unwrap(), frame);
    }

    #[test]
    fn empty_payload_is_legal() {
        // Comfort noise and end-of-talkspurt markers carry no audio.
        let frame = VoiceFrame { sequence: 0, rms: 0, payload: &[] };
        assert_eq!(VoiceFrame::decode(&frame.encode()).unwrap(), frame);
    }

    #[test]
    fn a_lying_length_field_cannot_over_read() {
        // The classic buffer over-read: header claims 4096 bytes, only 4 are present.
        let mut buf = vec![1u8, 2, 0, 0];
        buf[2..4].copy_from_slice(&4096u16.to_le_bytes());
        assert_eq!(
            VoiceFrame::decode(&buf),
            Err(DecodeError::TooShort { need: 4100, got: 4 })
        );
    }
}
