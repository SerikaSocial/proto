//! Regenerates `server/proto/golden/vectors.json`.
//!
//!     cargo run -p serika-proto --features gen --bin gen-golden
//!
//! The Rust implementation is the reference: it produces the corpus, and every other
//! language's test suite asserts against it. Only rerun this when the spec in
//! `pose_codec.md` genuinely changes — regenerating to make a failing test pass defeats
//! the entire purpose of having a corpus.

use serika_proto::pose::{pack_quat, quantize_pos};
use serika_proto::{Lod, PoseFrame};
use serde_json::json;

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

/// Deterministic pseudo-random spread, so the corpus exercises more than round numbers
/// without depending on a RNG whose algorithm could change between releases.
fn wobble(seed: u32, i: usize) -> f64 {
    let x = (seed.wrapping_mul(2654435761).wrapping_add(i as u32 * 40503)) % 10007;
    x as f64 / 10007.0 * 2.0 - 1.0
}

fn quat(seed: u32, i: usize) -> [f32; 4] {
    let a = wobble(seed, i);
    let b = wobble(seed, i + 1);
    let c = wobble(seed, i + 2);
    let d = wobble(seed, i + 3);
    let len = (a * a + b * b + c * c + d * d).sqrt().max(1e-9);
    [(a / len) as f32, (b / len) as f32, (c / len) as f32, (d / len) as f32]
}

fn main() {
    let mut cases = Vec::new();

    // Scalar cases pin the quantizers independently of frame layout, so a failure points
    // at the arithmetic rather than the framing.
    let mut positions = Vec::new();
    for v in [-256.0f32, -255.999, -100.5, -1.0, -0.0001, 0.0, 0.0001, 1.0, 42.7, 255.999, 256.0, 1e9, -1e9] {
        positions.push(json!({ "input": v, "quantized": quantize_pos(v) }));
    }

    let mut quats = Vec::new();
    for q in [
        [0.0f32, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0],
        [0.5, 0.5, 0.5, 0.5],
        [-0.5, 0.5, -0.5, 0.5],
        [0.7071068, 0.0, 0.0, 0.7071068],
        [0.0, -0.7071068, 0.0, 0.7071068],
        [0.183, 0.365, 0.548, 0.730],
        // Unnormalized input: encoders must normalize before packing.
        [2.0, 0.0, 0.0, 0.0],
        // Degenerate: must fall back to identity rather than emitting NaN.
        [0.0, 0.0, 0.0, 0.0],
    ] {
        quats.push(json!({ "input": q, "packed": pack_quat(q) }));
    }

    for (name, lod, seed) in [
        ("lod0_identity", Lod::Full, 0u32),
        ("lod0_wobble", Lod::Full, 11),
        ("lod1_identity", Lod::Body, 0),
        ("lod1_wobble", Lod::Body, 23),
        ("lod2_wobble", Lod::Distant, 37),
    ] {
        let bones: Vec<[f32; 4]> = (0..lod.bone_count())
            .map(|i| if seed == 0 { [0.0, 0.0, 0.0, 1.0] } else { quat(seed, i * 4) })
            .collect();

        let frame = PoseFrame {
            lod,
            sequence: (seed % 256) as u8,
            root_pos: [
                wobble(seed, 100) as f32 * 50.0,
                wobble(seed, 101) as f32 * 3.0,
                wobble(seed, 102) as f32 * 50.0,
            ],
            root_rot: if seed == 0 { [0.0, 0.0, 0.0, 1.0] } else { quat(seed, 200) },
            bones: bones.clone(),
            hands: [
                [wobble(seed, 300) as f32, 1.1, 0.2],
                [wobble(seed, 310) as f32, 1.1, 0.2],
            ],
        };

        let encoded = frame.encode();
        assert_eq!(encoded.len(), lod.frame_len(), "{name} length mismatch");

        cases.push(json!({
            "name": name,
            "lod": lod as u8,
            "sequence": frame.sequence,
            "root_pos": frame.root_pos,
            "root_rot": frame.root_rot,
            "bones": bones,
            "hands": frame.hands,
            "encoded_hex": hex(&encoded),
        }));
    }

    let doc = json!({
        "version": 1,
        "generated_by": "cargo run -p serika-proto --features gen --bin gen-golden",
        "note": "Do not hand-edit. Do not regenerate to silence a failing test — a diff here means the wire format changed and every client in the field is now incompatible.",
        "positions": positions,
        "quaternions": quats,
        "frames": cases,
    });

    println!("{}", serde_json::to_string_pretty(&doc).unwrap());
}
