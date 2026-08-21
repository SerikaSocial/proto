# Golden corpus

`vectors.json` is the executable form of [`../pose_codec.md`](../pose_codec.md). Every
language that speaks the wire protocol runs it through the same assertions:

| Implementation | Test |
|---|---|
| Rust (`instanced`) | `cargo test -p serika-proto` → `rust/tests/golden.rs` |
| C# (Godot client) | `dotnet test` → `game/Net/Codec/GoldenTests.cs` |

When both pass, client and server agree byte-for-byte. That is the only mechanical
guarantee that a Godot build already in someone's hands can still talk to a deployed relay.

## Contents

- **`positions`** — scalar inputs and their expected `u16`, including the clamp boundaries
  and values past ±256 m.
- **`quaternions`** — inputs and their expected packed `u32`, including identity, an
  unnormalized input, and the all-zero degenerate case.
- **`frames`** — complete poses at each LOD with their full hex encoding.

Scalar cases are separate from frames on purpose: when a frame case fails, the scalar cases
tell you immediately whether the bug is in the arithmetic or in the framing.

## Regenerating

```bash
cargo run -p serika-proto --features gen --bin gen-golden > server/proto/golden/vectors.json
```

**Only do this when `pose_codec.md` genuinely changes.** Regenerating to make a failing test
pass is exactly the failure mode the corpus exists to prevent — it converts "the client and
server disagree" into "the tests are green and the client and server disagree."

A diff in this file is a **breaking protocol change**. Every deployed client becomes
incompatible. It needs a version bump, and the relay needs to speak both versions for as
long as old builds are in the field.

## Adding cases

Add them to `rust/src/bin/gen_golden.rs` and regenerate. Good candidates are values that
have already caused a bug — the `1022` vs `1023` quaternion scale is in there precisely
because the naive choice broke identity, which is the single most common value on the wire.
