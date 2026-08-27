# PoseFrame wire codec — v1

The contract between `server/instanced` (Rust) and `game` (C#). Every byte here is
hand-packed; there is no code generator. **If you change anything in this file you must
regenerate the golden corpus and both test suites must still pass.**

## Why hand-packed and not FlatBuffers

The plan called for FlatBuffers on the control plane. It is still the right choice there —
but not here, and it turns out `flatc` isn't installed on the dev machine anyway.

The reason is precision, not tooling: this codec's entire value is *sub-byte quantization*
(10-bit quaternion components, 2-bit selectors). FlatBuffers has no way to express that —
its smallest field is a byte, and a 55-bone pose would come out at ~900 bytes instead of
230. A schema compiler that can't encode the thing the format exists to do is not buying us
anything.

So: this document is the schema, and the golden corpus is the compiler. See
[`golden/README.md`](golden/README.md).

## Frame layout

All integers little-endian. Bit fields are packed MSB-first within each 32-bit word.

```
┌──────────┬─────────────────────────────────────────────────────────────┐
│ offset   │ field                                                       │
├──────────┼─────────────────────────────────────────────────────────────┤
│ 0        │ u8   flags:  bits 0-1 = LOD (0|1|2), bits 2-7 reserved (0)  │
│ 1        │ u8   sequence (wraps; receiver reorders/discards)           │
│ 2..8     │ root position — 3 × u16 quantized                           │
│ 8..12    │ root rotation — smallest-three quaternion                   │
│ 12..     │ LOD-dependent payload (below)                               │
└──────────┴─────────────────────────────────────────────────────────────┘
```

`flags` bits 2-7 are reserved and **must be zero**. A decoder seeing a non-zero reserved bit
must reject the frame rather than guess — that's the forward-compat hook for v2.

### Position quantization

Positions are quantized to `u16` over a fixed symmetric range of **±256 m** from the instance
origin:

```
q = round((clamp(v, -256, 256) + 256) / 512 * 65535)
v = q / 65535 * 512 - 256
```

Resolution is 512/65535 ≈ **7.8 mm**, which is below the threshold where anyone notices
positional judder on a remote avatar. Worlds larger than 512 m across need a v2 with a
per-instance origin offset; the reserved flag bits exist for that.

### Smallest-three quaternion

Quaternions are unit-length, so one component is redundant. Drop the largest-magnitude one
and store the other three at 10 bits each:

```
 31    30 29        20 19        10 9         0
┌────────┬────────────┬────────────┬────────────┐
│ idx(2) │   a (10)   │   b (10)   │   c (10)   │
└────────┴────────────┴────────────┴────────────┘
```

- `idx` — which component was dropped (0=x, 1=y, 2=z, 3=w).
- If the dropped component is negative, negate the whole quaternion first. `q` and `-q` are
  the same rotation, so this is free, and it lets the decoder assume the reconstructed
  component is positive.
- The remaining three are each in `[-1/√2, +1/√2]` (they must be, since the dropped one was
  the largest). Map that range onto 10 bits:

```
a = round((v * √2 + 1) / 2 * 1022)
v = (a / 1022 * 2 - 1) / √2
```

- Reconstruct: `dropped = sqrt(max(0, 1 - a² - b² - c²))`, then renormalize.

**The scale is 1022, not 1023 — this matters.** An odd number of codes puts zero exactly on
code 511. With 1023 the range has an even number of codes, 0.0 falls *between* 511 and 512,
and the identity quaternion cannot round-trip: it comes back as 0.00069 per component, a
0.14° error. Since identity is every bone that isn't currently animated, that's a permanent
sub-degree twist on most of most avatars. Code 1023 goes unused; the 0.1% of lost precision
is a good trade.

Worst-case angular error is ~0.14° (three components at half a step), well under what 20 Hz
interpolation smooths over.

The `max(0, …)` is not decoration — rounding can push the sum just past 1.0 and `sqrt` of a
negative yields NaN, which propagates silently into the skeleton and is genuinely miserable
to debug.

> When testing round-trip accuracy, do **not** measure with `2 * acos(dot)`. For
> near-identical quaternions the dot product sits within an ulp of 1.0, where `acos` has no
> precision left and an infinite derivative — in f32 it reports a 0.05° error as 1.04°.
> Measure the error vector's length in f64 and convert with `asin`.

## LOD payloads

Bone rotations are in canonical rig order (see `HUMANOID_BONES` in the reference
implementations). Each is one smallest-three `u32`.

| LOD | When | Contents | Size |
|---|---|---|---|
| 0 | a peer within ~6 m | 55 bone rotations (body + eyes/jaw + fingers) | 12 + 220 = **232 B** |
| 1 | otherwise | first 22 bone rotations (body, no fingers) | 12 + 88 = **100 B** |
| 2 | >20 m | head rotation + 2 hand positions | 12 + 4 + 12 = **28 B** |

At LOD2 the receiver reconstructs arms with two-bone IK from the hand positions and leaves
the rest of the body on a locomotion pose driven by root velocity. Nobody can tell at 20 m.

At 20 Hz: LOD0 ≈ 4.6 KB/s per avatar, LOD1 ≈ 2.0 KB/s, LOD2 ≈ 0.56 KB/s. That's what makes
the 64 KB/s per-client budget hold ~80 players in view — the AOI priority accumulator keeps
only a handful at LOD0.

## Canonical bone order

Index 0-21 are the body bones sent at LOD1; 22-24 are the remaining body bones and 25-54 are
fingers, both LOD0 only. The order is fixed forever — inserting a bone is a v2 change, not a
patch.

```
 0 hips             14 leftLowerArm      28 leftIndexProximal
 1 spine            15 leftHand          29 leftIndexIntermediate
 2 chest            16 rightShoulder     30 leftIndexDistal
 3 upperChest       17 rightUpperArm     31 leftMiddleProximal
 4 neck             18 rightLowerArm     32 leftMiddleIntermediate
 5 head             19 rightHand         33 leftMiddleDistal
 6 leftUpperLeg     20 leftToes          34 leftRingProximal
 7 leftLowerLeg     21 rightToes         35 leftRingIntermediate
 8 leftFoot         ── LOD0 only ──      36 leftRingDistal
 9 rightUpperLeg    22 leftEye           37 leftLittleProximal
10 rightLowerLeg    23 rightEye          38 leftLittleIntermediate
11 rightFoot        24 jaw               39 leftLittleDistal
12 leftShoulder     25 leftThumbProximal 40-54 right hand, same five
13 leftUpperArm     26 leftThumbInterm.        fingers in the same order,
                    27 leftThumbDistal        ending 54 rightLittleDistal
```

25 body bones + 30 finger bones = 55, which is exactly the VRM 1.0 humanoid set, so `godot-vrm`
imports map onto it without a translation table.

> **This table was previously wrong** and is corrected here to match the shipped codec. It listed
> eyes and jaw at 19-21 and omitted RightLowerLeg, LeftShoulder and LeftUpperArm, so its indices
> 10-21 disagreed with what both implementations actually encode. Eyes and jaw sit at 22-24
> — *after* the LOD1 prefix — precisely because LOD1 has shipped: a LOD1 frame is defined as the
> first 22 entries of this table, and moving anything below index 22 would silently reinterpret
> every pose frame in the field.

**The bone order is not enforced by the golden corpus.** The relay decodes a pose frame only to
validate it and to read the root position for AOI; it forwards the payload opaquely and never
learns what any bone *is*. Slot 30 means `leftIndexDistal` solely because both ends of the wire
are the same client reading the same table, so a reordering here would leave every golden vector
byte-identical while wiring a remote avatar's fingers to its eyes. The client therefore freezes
the order in its own test suite (`game/Net/Codec/Tests/BoneOrderTests.cs`) alongside the byte
layout. Treat that file as part of this specification.

### Which LOD a sender picks

A client sends **one** pose frame per tick and the relay fans it out to every peer in range, so
the LOD cannot be chosen per receiver the way the table above suggests. The sender picks: it
upgrades to LOD0 while any peer is within conversational distance (~6 m) and the local rig
actually has finger bones, and stays at LOD1 otherwise. Rigs with no finger bones never upgrade,
since LOD0 would spend 132 extra bytes a frame transmitting 30 identity quaternions.

## Voice frames

Opus packets are **forwarded opaquely** — the server never decodes audio. The framing is
just enough for the relay to route and rank:

```
┌──────────┬─────────────────────────────────────────────────────────────┐
│ 0        │ u8   sequence                                               │
│ 1        │ u8   rms — perceptual loudness, 0-255, for speaker ranking   │
│ 2..4     │ u16  opus payload length                                    │
│ 4..      │      opus payload                                           │
└──────────┴─────────────────────────────────────────────────────────────┘
```

`rms` is client-reported and therefore **untrusted** — a client can claim to be loud to win
speaker-ranking slots. The relay clamps how often a peer may occupy a top-N slot. Ranking is
a fairness heuristic, not a security boundary.

## Test vectors

`golden/vectors.json` holds input poses and their expected hex encodings. Both
`server/proto/rust` and the C# client tests decode it and assert byte-identical output.

Encoding must be **deterministic** — no floating-point non-determinism across platforms.
All quantization uses `round-half-away-from-zero` on an `f64` intermediate, which is exact
for every value in range on both x86-64 and aarch64.
