# pico-node parity drift guard

This directory + the repo-root `parity-*` files are pico-node's leg of the
three-stack parity contract:

> **C# `Packet.*` is authoritative** ↔ TS `@packet-net/ax25` ↔ Rust pico-node.

It is the pico-node analogue of ax25-ts's `scripts/parity-check.mjs` +
`scripts/parity-exceptions.json`: a **source-inventory** drift guard, not a
wire-byte check. It asserts

```
C#-inventory  ⊆  pico-node-declared
```

and fails CI on any **undocumented** gap — the thing that makes "pico-node
silently drifted from the C# reference" impossible to repeat.

## Moving parts

| File | Owner | Role |
| --- | --- | --- |
| `parity/expected-inventory.json` | this guard | **Vendored snapshot** of the C# named-flag / quirk / preset / listener inventory (parse flags, session quirks, XID knobs, presets, `Ax25Listener` options + surface). Checked in so the guard needs no cross-repo token at CI time. |
| `parity-manifest.toml` (repo root) | sibling (manifest + vector runner) | pico-node's opted-in / declared-out vector sets and their capabilities. |
| `parity-exceptions.json` (repo root) | this guard | Reviewed, reason-carrying omissions: per-item `inventoryExceptions`, plus whole-area `declaredOut` decisions. |
| `scripts/parity-check.mjs` | this guard | The checker. Node, `node:` builtins only (no npm deps), mirroring ax25-ts. |
| `scripts/__fixtures__/parity-manifest.sample.toml` | this guard | Schema-identical fixture copy of the manifest, so the checker self-tests on this branch before the real manifest lands. |
| `.github/workflows/parity.yml` | this guard | `runs-on: [self-hosted, Linux, X64]` job running `node scripts/parity-check.mjs`. |

## What the checker does

1. **Parse `parity-manifest.toml`** (repo root by default; override with
   `--manifest <path>` or `$PARITY_MANIFEST`). If the real manifest is absent
   and none was requested, it falls back to the sample fixture, loudly labelled.
2. **Validate the manifest schema**: every `[vector_sets.*]` has a boolean `in`;
   every `in = false` set has a non-empty `reason`; every opted-in set has a
   non-empty `capabilities` array.
3. **Validate `parity-exceptions.json`**: every exception (per-item and
   `declaredOut`) carries a non-empty reason.
4. **Assert `C# ⊆ pico-node-declared`**: for each item in
   `expected-inventory.json`, it is **covered** iff its `coverage.set` is an
   opted-in manifest set (and, when `coverage.capability` is named, that set
   declares it); otherwise it must be **excepted** in `parity-exceptions.json`
   (`inventoryExceptions[section][name]`). An item that is neither is an
   **undocumented gap** → the checker prints it and exits **1**.

Exit codes: `0` in sync (possibly with documented exceptions) · `1` drift or
schema error · `2` usage / missing input.

## Run it locally

```sh
# against the real manifest (once the sibling branch has merged it):
node scripts/parity-check.mjs

# against the fixture, before the real manifest exists:
node scripts/parity-check.mjs --manifest scripts/__fixtures__/parity-manifest.sample.toml
```

## Refreshing the C# snapshot when the reference changes

`expected-inventory.json` is a **manual, reviewed** vendored copy — that is the
point: a new C# named flag does not silently appear here; someone re-extracts it
and, in the same PR, either maps it to a vector set or adds a reasoned
exception. To refresh from a `packet-net/packet.net` checkout:

- `src/Packet.Core/Ax25ParseOptions.cs` — `parseOptionFlags` = `public bool X { get; … }`; `parsePresets` = `public static Ax25ParseOptions X { get; }`.
- `src/Packet.Ax25/Session/Ax25SessionQuirks.cs` — `quirkFlags` = `public bool X { get; … }`; `quirkPresets` = `public static Ax25SessionQuirks X { get; }`.
- `src/Packet.Ax25/Xid/XidParseOptions.cs` — `xidFlags` = `public bool X { get; … }`.
- `src/Packet.Ax25/Session/Ax25Listener.cs` — `listenerOptions` = the `public T X { get; … }` members of `Ax25ListenerOptions`; `listenerSurface` = the `public (async)? T X(…)` methods + `public event T X;` events of `Ax25Listener`.

Record the source commit in `source.commit`. For every added item: set
`coverage` to an opted-in set (+ capability), or add an entry to
`parity-exceptions.json` with a one-line reason. Leaving it unmapped and
un-excepted is exactly the drift the guard catches.

## Why source-inventory, not wire-vectors

The wire-byte golden-vector corpus + Rust runner is the sibling agent's deliverable
(`parity-manifest.toml`, `vectors/`, `tests/`). This guard is the *governance*
layer around it: it guarantees every C# capability is a **conscious** decision on
the pico-node side — covered by a vector set or an explicit exception — so a
reviewer can never merge a new C# flag with a silent hole on this side.
