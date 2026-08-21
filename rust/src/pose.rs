//! Avatar pose encoding. See `server/proto/pose_codec.md` §Frame layout.

use crate::DecodeError;

/// Bones in canonical rig order. Indices 0..22 ship at LOD1; the rest are fingers.
pub const HUMANOID_BONE_COUNT: usize = 55;
/// How many of those go out at LOD1 — body only, no fingers.
pub const LOD1_BONE_COUNT: usize = 22;

/// Positions quantize over ±`POS_RANGE` metres from the instance origin.
const POS_RANGE: f64 = 256.0;
const POS_SPAN: f64 = POS_RANGE * 2.0;
const U16_MAX: f64 = u16::MAX as f64;

/// Smallest-three components live in ±1/√2, mapped onto 10 bits.
const QUAT_SCALE: f64 = std::f64::consts::SQRT_2;
/// 1022, not 1023, so that the range has an odd number of codes and **zero lands exactly
/// on code 511**. With an even count, 0.0 falls between two codes and the identity
/// quaternion — by far the most common value on the wire, since it's every bone that
/// isn't being animated — cannot round-trip. It came back as 0.00069 per component, a
/// 0.14° error on every idle bone of every avatar.
///
/// Giving up code 1023 costs 0.1% of precision. Worth it.
const QUAT_MAX: f64 = 1022.0;

const HEADER_LEN: usize = 12;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Lod {
    /// Full body plus fingers — the nearest handful of avatars.
    Full = 0,
    /// Body only, fingers dropped.
    Body = 1,
    /// Head rotation and hand positions; the receiver IKs the rest.
    Distant = 2,
}

impl Lod {
    fn from_bits(bits: u8) -> Result<Self, DecodeError> {
        match bits {
            0 => Ok(Lod::Full),
            1 => Ok(Lod::Body),
            2 => Ok(Lod::Distant),
            other => Err(DecodeError::BadLod(other)),
        }
    }

    /// How many bone rotations ride in the payload at this LOD.
    pub fn bone_count(self) -> usize {
        match self {
            Lod::Full => HUMANOID_BONE_COUNT,
            Lod::Body => LOD1_BONE_COUNT,
            // The single head rotation, handled separately from the bone array.
            Lod::Distant => 1,
        }
    }

    /// Exact encoded frame size in bytes.
    pub fn frame_len(self) -> usize {
        match self {
            Lod::Full => HEADER_LEN + HUMANOID_BONE_COUNT * 4,
            Lod::Body => HEADER_LEN + LOD1_BONE_COUNT * 4,
            // head rotation (4) + two hand positions (6 each)
            Lod::Distant => HEADER_LEN + 4 + 12,
        }
    }
}

#[derive(Debug, Clone, PartialEq)]
pub struct PoseFrame {
    pub lod: Lod,
    pub sequence: u8,
    pub root_pos: [f32; 3],
    pub root_rot: [f32; 4],
    /// `lod.bone_count()` rotations in canonical order. At `Distant` this is the single
    /// head rotation.
    pub bones: Vec<[f32; 4]>,
    /// Left and right hand positions. Only meaningful at `Distant`.
    pub hands: [[f32; 3]; 2],
}

impl PoseFrame {
    pub fn encode(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(self.lod.frame_len());

        out.push(self.lod as u8);
        out.push(self.sequence);
        for v in self.root_pos {
            out.extend_from_slice(&quantize_pos(v).to_le_bytes());
        }
        out.extend_from_slice(&pack_quat(self.root_rot).to_le_bytes());

        match self.lod {
            Lod::Full | Lod::Body => {
                for bone in self.bones.iter().take(self.lod.bone_count()) {
                    out.extend_from_slice(&pack_quat(*bone).to_le_bytes());
                }
                // A short `bones` vec would otherwise silently produce a truncated frame
                // that still decodes — pad so the length invariant always holds.
                let missing = self.lod.bone_count().saturating_sub(self.bones.len());
                for _ in 0..missing {
                    out.extend_from_slice(&pack_quat(IDENTITY).to_le_bytes());
                }
            }
            Lod::Distant => {
                let head = self.bones.first().copied().unwrap_or(IDENTITY);
                out.extend_from_slice(&pack_quat(head).to_le_bytes());
                for hand in self.hands {
                    for v in hand {
                        out.extend_from_slice(&quantize_pos(v).to_le_bytes());
                    }
                }
            }
        }

        debug_assert_eq!(out.len(), self.lod.frame_len());
        out
    }

    pub fn decode(buf: &[u8]) -> Result<Self, DecodeError> {
        if buf.len() < HEADER_LEN {
            return Err(DecodeError::TooShort { need: HEADER_LEN, got: buf.len() });
        }

        let flags = buf[0];
        // Reject rather than mask: an unknown bit means a newer sender, and silently
        // reinterpreting its frame is how you get a desync you can't reproduce.
        if flags & 0xFC != 0 {
            return Err(DecodeError::ReservedBitsSet(flags & 0xFC));
        }
        let lod = Lod::from_bits(flags & 0x03)?;

        let want = lod.frame_len();
        if buf.len() < want {
            return Err(DecodeError::TooShort { need: want, got: buf.len() });
        }
        if buf.len() > want {
            return Err(DecodeError::TrailingBytes(buf.len() - want));
        }

        let sequence = buf[1];
        let root_pos = [
            dequantize_pos(read_u16(buf, 2)),
            dequantize_pos(read_u16(buf, 4)),
            dequantize_pos(read_u16(buf, 6)),
        ];
        let root_rot = unpack_quat(read_u32(buf, 8));

        let mut bones = Vec::with_capacity(lod.bone_count());
        let mut hands = [[0.0f32; 3]; 2];

        match lod {
            Lod::Full | Lod::Body => {
                for i in 0..lod.bone_count() {
                    bones.push(unpack_quat(read_u32(buf, HEADER_LEN + i * 4)));
                }
            }
            Lod::Distant => {
                bones.push(unpack_quat(read_u32(buf, HEADER_LEN)));
                let mut off = HEADER_LEN + 4;
                for hand in hands.iter_mut() {
                    for axis in hand.iter_mut() {
                        *axis = dequantize_pos(read_u16(buf, off));
                        off += 2;
                    }
                }
            }
        }

        Ok(PoseFrame { lod, sequence, root_pos, root_rot, bones, hands })
    }
}

const IDENTITY: [f32; 4] = [0.0, 0.0, 0.0, 1.0];

fn read_u16(buf: &[u8], at: usize) -> u16 {
    u16::from_le_bytes([buf[at], buf[at + 1]])
}

fn read_u32(buf: &[u8], at: usize) -> u32 {
    u32::from_le_bytes([buf[at], buf[at + 1], buf[at + 2], buf[at + 3]])
}

/// Metres to 16-bit. ~7.8 mm resolution over ±256 m.
pub fn quantize_pos(v: f32) -> u16 {
    let clamped = (v as f64).clamp(-POS_RANGE, POS_RANGE);
    let norm = (clamped + POS_RANGE) / POS_SPAN;
    // f64::round is half-away-from-zero, which is what the spec mandates for
    // cross-platform determinism.
    (norm * U16_MAX).round() as u16
}

pub fn dequantize_pos(q: u16) -> f32 {
    ((q as f64) / U16_MAX * POS_SPAN - POS_RANGE) as f32
}

/// Smallest-three quaternion packing. See `pose_codec.md` §Smallest-three quaternion.
pub fn pack_quat(q: [f32; 4]) -> u32 {
    let mut q = [q[0] as f64, q[1] as f64, q[2] as f64, q[3] as f64];

    // Normalize defensively — a drifted quaternion would push components outside ±1/√2
    // and corrupt the packing.
    let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if len > 0.0 {
        for c in q.iter_mut() {
            *c /= len;
        }
    } else {
        q = [0.0, 0.0, 0.0, 1.0];
    }

    let mut largest = 0usize;
    for i in 1..4 {
        if q[i].abs() > q[largest].abs() {
            largest = i;
        }
    }

    // q and -q are the same rotation, so choosing the sign is free — and it lets the
    // decoder assume the reconstructed component is positive.
    if q[largest] < 0.0 {
        for c in q.iter_mut() {
            *c = -*c;
        }
    }

    let mut word = (largest as u32) << 30;
    let mut shift = 20;
    for (i, c) in q.iter().enumerate() {
        if i == largest {
            continue;
        }
        let norm = (c * QUAT_SCALE + 1.0) / 2.0;
        let packed = (norm * QUAT_MAX).round().clamp(0.0, QUAT_MAX) as u32;
        word |= packed << shift;
        shift -= 10;
    }
    word
}

pub fn unpack_quat(word: u32) -> [f32; 4] {
    let largest = (word >> 30) as usize;
    let mut q = [0.0f64; 4];

    let mut shift = 20;
    let mut sum_sq = 0.0f64;
    for i in 0..4 {
        if i == largest {
            continue;
        }
        let packed = ((word >> shift) & 0x3FF) as f64;
        let v = (packed / QUAT_MAX * 2.0 - 1.0) / QUAT_SCALE;
        q[i] = v;
        sum_sq += v * v;
        shift -= 10;
    }

    // Rounding can push sum_sq marginally past 1.0. Without the clamp this is sqrt of a
    // negative, and the resulting NaN propagates silently into the skeleton.
    q[largest] = (1.0 - sum_sq).max(0.0).sqrt();

    let len = (q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]).sqrt();
    if len > 0.0 {
        for c in q.iter_mut() {
            *c /= len;
        }
    }

    [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32]
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Angular distance between two rotations, in degrees.
    ///
    /// Deliberately *not* `2 * acos(dot)`: near-identical quaternions give a dot product
    /// within an ulp of 1.0, where `acos` has almost no precision left and the derivative
    /// is infinite. In f32 that reported a 0.05° error as 1.04°. Measuring the length of
    /// the error vector in f64 and converting via `asin` is stable in exactly the regime
    /// we care about.
    fn angle_between_deg(a: [f32; 4], b: [f32; 4]) -> f64 {
        let a = a.map(|v| v as f64);
        let mut b = b.map(|v| v as f64);
        // q and -q are the same rotation; pick the closer representative.
        let dot: f64 = (0..4).map(|i| a[i] * b[i]).sum();
        if dot < 0.0 {
            b = b.map(|v| -v);
        }
        let err: f64 = (0..4).map(|i| (a[i] - b[i]).powi(2)).sum::<f64>().sqrt();
        2.0 * (err / 2.0).clamp(-1.0, 1.0).asin().to_degrees()
    }

    #[test]
    fn position_roundtrip_stays_within_resolution() {
        // One quantization step over the ±256 m range.
        let step = (POS_SPAN / U16_MAX) as f32;
        for v in [-256.0f32, -17.5, -0.001, 0.0, 0.001, 1.0, 42.7, 255.9] {
            let back = dequantize_pos(quantize_pos(v));
            assert!((back - v).abs() <= step, "{v} -> {back}");
        }
    }

    #[test]
    fn positions_clamp_instead_of_wrapping() {
        // Out-of-range must saturate. Wrapping would teleport a distant avatar across
        // the world, which is far worse than pinning it at the boundary.
        assert_eq!(quantize_pos(9999.0), u16::MAX);
        assert_eq!(quantize_pos(-9999.0), 0);
    }

    #[test]
    fn identity_quaternion_roundtrips_exactly() {
        // The most common value on the wire. Any error here shows up as a permanent
        // sub-degree twist on every idle bone of every avatar in the instance.
        assert_eq!(unpack_quat(pack_quat(IDENTITY)), IDENTITY);
    }

    #[test]
    fn quaternion_roundtrip_stays_under_a_fifth_of_a_degree() {
        let cases = [
            [0.0, 0.0, 0.0, 1.0],
            [0.5, 0.5, 0.5, 0.5],
            [-0.5, 0.5, -0.5, 0.5],
            [0.7071068, 0.0, 0.0, 0.7071068],
            [0.0, -0.7071068, 0.0, 0.7071068],
            [0.183, 0.365, 0.548, 0.730],
        ];
        for q in cases {
            let back = unpack_quat(pack_quat(q));
            let angle = angle_between_deg(q, back);
            // Three components at half a step each works out to ~0.14° worst case.
            assert!(angle < 0.2, "{q:?} -> {back:?}, off by {angle}°");
        }
    }

    #[test]
    fn negated_quaternion_packs_identically() {
        // q and -q are the same rotation and must not produce two different encodings,
        // or identical poses would burn bandwidth looking like changes.
        let q = [0.183, 0.365, 0.548, 0.730];
        let neg = [-q[0], -q[1], -q[2], -q[3]];
        assert_eq!(pack_quat(q), pack_quat(neg));
    }

    #[test]
    fn degenerate_quaternion_becomes_identity() {
        assert_eq!(unpack_quat(pack_quat([0.0, 0.0, 0.0, 0.0])), IDENTITY);
    }

    #[test]
    fn frames_roundtrip_at_every_lod() {
        for lod in [Lod::Full, Lod::Body, Lod::Distant] {
            let frame = PoseFrame {
                lod,
                sequence: 7,
                root_pos: [1.5, 0.0, -3.25],
                root_rot: [0.0, 0.7071068, 0.0, 0.7071068],
                bones: vec![[0.0, 0.0, 0.0, 1.0]; lod.bone_count()],
                hands: [[0.3, 1.1, 0.2], [-0.3, 1.1, 0.2]],
            };
            let bytes = frame.encode();
            assert_eq!(bytes.len(), lod.frame_len());
            let back = PoseFrame::decode(&bytes).expect("decode");
            assert_eq!(back.lod, lod);
            assert_eq!(back.sequence, 7);
            assert_eq!(back.bones.len(), lod.bone_count());
        }
    }

    #[test]
    fn short_bone_vec_is_padded_not_truncated() {
        // A caller passing too few bones must still produce a spec-length frame.
        let frame = PoseFrame {
            lod: Lod::Full,
            sequence: 0,
            root_pos: [0.0; 3],
            root_rot: IDENTITY,
            bones: vec![IDENTITY; 3],
            hands: [[0.0; 3]; 2],
        };
        let bytes = frame.encode();
        assert_eq!(bytes.len(), Lod::Full.frame_len());
        assert_eq!(PoseFrame::decode(&bytes).unwrap().bones.len(), HUMANOID_BONE_COUNT);
    }

    #[test]
    fn hostile_frames_are_rejected() {
        let good = PoseFrame {
            lod: Lod::Body,
            sequence: 1,
            root_pos: [0.0; 3],
            root_rot: IDENTITY,
            bones: vec![IDENTITY; LOD1_BONE_COUNT],
            hands: [[0.0; 3]; 2],
        }
        .encode();

        assert_eq!(
            PoseFrame::decode(&good[..4]),
            Err(DecodeError::TooShort { need: 12, got: 4 })
        );

        let mut trailing = good.clone();
        trailing.push(0xAA);
        assert_eq!(PoseFrame::decode(&trailing), Err(DecodeError::TrailingBytes(1)));

        let mut reserved = good.clone();
        reserved[0] |= 0x80;
        assert_eq!(PoseFrame::decode(&reserved), Err(DecodeError::ReservedBitsSet(0x80)));

        let mut bad_lod = good.clone();
        bad_lod[0] = 3;
        assert_eq!(PoseFrame::decode(&bad_lod), Err(DecodeError::BadLod(3)));
    }
}
