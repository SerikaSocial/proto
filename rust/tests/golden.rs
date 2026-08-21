//! Asserts the Rust codec against the checked-in golden corpus.
//!
//! The C# client runs the same corpus through the same assertions. When these two agree,
//! client and server agree — and that is the only mechanical guarantee we have that a
//! Godot build in the wild can still talk to a deployed relay.
//!
//! If this fails, the wire format changed. That is a breaking change, not a test to fix.

use serika_proto::pose::{pack_quat, quantize_pos};
use serika_proto::{Lod, PoseFrame};
use std::path::PathBuf;

fn corpus() -> serde_json::Value {
    let path = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../golden/vectors.json");
    let raw = std::fs::read_to_string(&path)
        .unwrap_or_else(|e| panic!("read {}: {e}", path.display()));
    serde_json::from_str(&raw).expect("parse vectors.json")
}

fn as_f32(v: &serde_json::Value) -> f32 {
    v.as_f64().expect("number") as f32
}

fn quat_at(v: &serde_json::Value) -> [f32; 4] {
    let a = v.as_array().expect("quaternion array");
    assert_eq!(a.len(), 4, "quaternion must have 4 components");
    [as_f32(&a[0]), as_f32(&a[1]), as_f32(&a[2]), as_f32(&a[3])]
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{b:02x}")).collect()
}

#[test]
fn corpus_is_the_expected_version() {
    // A version bump means the format moved; the C# side must move with it.
    assert_eq!(corpus()["version"].as_u64(), Some(1));
}

#[test]
fn position_quantization_matches_corpus() {
    for case in corpus()["positions"].as_array().unwrap() {
        let input = as_f32(&case["input"]);
        let expected = case["quantized"].as_u64().unwrap() as u16;
        assert_eq!(quantize_pos(input), expected, "quantize_pos({input})");
    }
}

#[test]
fn quaternion_packing_matches_corpus() {
    for case in corpus()["quaternions"].as_array().unwrap() {
        let input = quat_at(&case["input"]);
        let expected = case["packed"].as_u64().unwrap() as u32;
        assert_eq!(pack_quat(input), expected, "pack_quat({input:?})");
    }
}

#[test]
fn frames_encode_to_the_corpus_bytes() {
    for case in corpus()["frames"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();

        let lod = match case["lod"].as_u64().unwrap() {
            0 => Lod::Full,
            1 => Lod::Body,
            2 => Lod::Distant,
            other => panic!("{name}: corpus has unknown LOD {other}"),
        };

        let pos = case["root_pos"].as_array().unwrap();
        let hands = case["hands"].as_array().unwrap();
        let left = hands[0].as_array().unwrap();
        let right = hands[1].as_array().unwrap();

        let frame = PoseFrame {
            lod,
            sequence: case["sequence"].as_u64().unwrap() as u8,
            root_pos: [as_f32(&pos[0]), as_f32(&pos[1]), as_f32(&pos[2])],
            root_rot: quat_at(&case["root_rot"]),
            bones: case["bones"].as_array().unwrap().iter().map(quat_at).collect(),
            hands: [
                [as_f32(&left[0]), as_f32(&left[1]), as_f32(&left[2])],
                [as_f32(&right[0]), as_f32(&right[1]), as_f32(&right[2])],
            ],
        };

        let expected = case["encoded_hex"].as_str().unwrap();
        assert_eq!(hex(&frame.encode()), expected, "{name} encoded differently");
    }
}

#[test]
fn corpus_frames_decode_back_to_their_inputs() {
    for case in corpus()["frames"].as_array().unwrap() {
        let name = case["name"].as_str().unwrap();
        let bytes: Vec<u8> = {
            let h = case["encoded_hex"].as_str().unwrap();
            (0..h.len())
                .step_by(2)
                .map(|i| u8::from_str_radix(&h[i..i + 2], 16).unwrap())
                .collect()
        };

        let frame = PoseFrame::decode(&bytes)
            .unwrap_or_else(|e| panic!("{name} failed to decode: {e}"));

        assert_eq!(frame.lod as u8, case["lod"].as_u64().unwrap() as u8, "{name} lod");
        assert_eq!(
            frame.sequence,
            case["sequence"].as_u64().unwrap() as u8,
            "{name} sequence"
        );
        assert_eq!(frame.bones.len(), frame.lod.bone_count(), "{name} bone count");

        // Re-encoding a decoded frame must be a fixed point. If it isn't, the codec has
        // a bias that would compound every time a frame is relayed.
        assert_eq!(hex(&frame.encode()), case["encoded_hex"].as_str().unwrap(), "{name} not idempotent");
    }
}

#[test]
fn frame_sizes_match_the_spec_table() {
    // pose_codec.md §LOD payloads publishes these numbers, and the bandwidth budget is
    // derived from them. If they drift, the budget silently stops meaning anything.
    assert_eq!(Lod::Full.frame_len(), 232);
    assert_eq!(Lod::Body.frame_len(), 100);
    assert_eq!(Lod::Distant.frame_len(), 28);
}
