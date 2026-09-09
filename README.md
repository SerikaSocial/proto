# Serika Social — proto (wire codec)

The wire codec and its golden corpus — the sacred contract between the Rust relay
([`server/instanced`](https://github.com/SerikaSocial/server)) and the C# client
([`game`](https://github.com/SerikaSocial/game)).

This repo is pinned as a **git submodule** in both `server` and `game`, so there is
exactly one definition of the wire format. When the protocol changes, this repo is
pushed first, then both pins move together. A client and a relay on different `proto`
commits is precisely the desync the golden corpus exists to prevent.

## What's here

```
pose_codec.md        the schema — hand-packed, no code generator. READ THIS FIRST.
golden/
  vectors.json       the executable form of pose_codec.md (the golden corpus)
  README.md          how the corpus works and how to regenerate it
rust/
  Cargo.toml         crate `serika-proto`
  src/lib.rs         public API
  src/pose.rs        PoseFrame encode/decode (sub-byte quantization)
  src/voice.rs       VoiceFrame encode/decode
  src/bin/gen_golden.rs   regenerates vectors.json (feature `gen`)
  tests/golden.rs    Rust golden tests
```

## Why hand-packed and not FlatBuffers

This codec's entire value is *sub-byte quantization* (10-bit quaternion components,
2-bit selectors). FlatBuffers' smallest field is a byte, and a 55-bone pose would come
out at ~900 bytes instead of ~230. A schema compiler that can't encode the thing the
format exists to do is not buying us anything.

So `pose_codec.md` is the schema, and the golden corpus is the compiler.

## The one rule

The golden-vector corpus (`golden/vectors.json`) must pass in **both** test suites —
byte-identical output or the build fails:

| Implementation | Test |
|---|---|
| Rust (`instanced`) | `cargo test -p serika-proto` → `rust/tests/golden.rs` |
| C# (Godot client) | `dotnet test` → `game/Net/Codec/GoldenTests.cs` |

When both pass, client and server agree byte-for-byte. That is the only mechanical
guarantee that a Godot build already in someone's hands can still talk to a deployed
relay.

## Regenerating the corpus

When you change the codec you **must** regenerate the corpus and both suites must still
pass:

```bash
cargo run -p serika-proto --features gen --bin gen-golden > golden/vectors.json
cargo test -p serika-proto                       # Rust side
# then in the game repo:
dotnet test Net/Codec/Tests                      # C# side — must match byte-for-byte
```

Commit `vectors.json` here, push, then bump the submodule pins in `server` and `game`.

## Frame layout (summary)

All integers little-endian; bit fields packed MSB-first within each 32-bit word.

```
offset 0   u8   flags:  bits 0-1 = LOD (0|1|2), bits 2-7 reserved (must be 0)
offset 1   u8   sequence (wraps; receiver reorders/discards)
offset 2   root position — 3 × u16 quantized
offset 8   root rotation — smallest-three quaternion
offset 12  LOD-dependent payload
```

A decoder seeing a non-zero reserved bit must reject the frame rather than guess —
that's the forward-compat hook for v2. See `pose_codec.md` for the full field-by-field
spec, position/rotation quantization, and the LOD payloads.

## Related repos

- [`server`](https://github.com/SerikaSocial/server) — `instanced` consumes this crate.
- [`game`](https://github.com/SerikaSocial/game) — the C# codec mirrors this in `Net/Codec/`.
- [`docs`](https://github.com/SerikaSocial/docs) — architecture docs.
