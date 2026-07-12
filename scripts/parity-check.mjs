#!/usr/bin/env node
/**
 * pico-node cross-stack parity drift guard.
 *
 * The C# libraries in packet-net/packet.net are the reference runtime
 * ("the C# code is the authoritative behaviour" — pico-node IMPL-CONSTRAINTS).
 * Historically a stack can silently drift behind it: a new named parse flag,
 * session quirk, XID knob, or listener capability lands in C# with no
 * counterpart here and nothing fails. This script makes that drift a CI
 * failure, in the same shape as ax25-ts scripts/parity-check.mjs.
 *
 * It is a SOURCE-INVENTORY guard, not a wire-byte check. It asserts:
 *
 *     C#-inventory  ⊆  pico-node-declared
 *
 * where "pico-node-declared" = every C# inventory item is EITHER covered by
 * an opted-in vector_set (and capability) in parity-manifest.toml, OR listed
 * in parity-exceptions.json with a reason. Anything that is neither is an
 * undocumented gap and fails the build (exit 1).
 *
 * Three inputs, all self-hosted-runner friendly (no cross-repo token):
 *   1. parity-manifest.toml           — pico-node's opted-in / declared-out sets
 *                                        (produced by the sibling manifest branch;
 *                                        a fixture copy lives in scripts/__fixtures__).
 *   2. parity/expected-inventory.json — VENDORED snapshot of the C# inventory
 *                                        (named flags / quirks / presets / listener
 *                                        options+surface). Refresh when C# changes.
 *   3. parity-exceptions.json         — reviewed, reason-carrying omissions.
 *
 * Usage:
 *   node scripts/parity-check.mjs
 *   node scripts/parity-check.mjs --manifest scripts/__fixtures__/parity-manifest.sample.toml
 *   node scripts/parity-check.mjs --inventory <path> --exceptions <path>
 *
 * Exit codes: 0 = in sync (possibly with documented exceptions); 1 = drift or
 * schema error; 2 = usage / missing-input error.
 */
import { readFileSync, existsSync } from "node:fs";
import { join, dirname, resolve } from "node:path";
import { fileURLToPath } from "node:url";

const scriptDir = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(scriptDir, "..");

const args = process.argv.slice(2);
function argValue(name, fallback) {
  const i = args.indexOf(name);
  return i >= 0 && args[i + 1] ? args[i + 1] : fallback;
}

const fixtureManifest = join(scriptDir, "__fixtures__", "parity-manifest.sample.toml");
const explicitManifest = argValue("--manifest", process.env.PARITY_MANIFEST ?? null);
let manifestPath = explicitManifest
  ? resolve(explicitManifest)
  : join(repoRoot, "parity-manifest.toml");
// When the real repo-root manifest is absent (it lands from the sibling
// branch) and none was requested explicitly, fall back to the fixture so the
// guard still runs and is green — loudly labelled so nobody mistakes it for
// the real thing.
let usingFixture = false;
if (!explicitManifest && !existsSync(manifestPath)) {
  if (!existsSync(fixtureManifest)) {
    console.error(`error: no manifest at ${manifestPath} and no fixture at ${fixtureManifest}`);
    process.exit(2);
  }
  manifestPath = fixtureManifest;
  usingFixture = true;
}
const inventoryPath = resolve(argValue("--inventory", join(repoRoot, "parity", "expected-inventory.json")));
const exceptionsPath = resolve(argValue("--exceptions", join(repoRoot, "parity-exceptions.json")));

for (const [label, p] of [["manifest", manifestPath], ["inventory", inventoryPath], ["exceptions", exceptionsPath]]) {
  if (!existsSync(p)) {
    console.error(`error: ${label} not found: ${p}`);
    process.exit(2);
  }
}

// ─── minimal TOML reader ──────────────────────────────────────────────
// Enough for parity-manifest.toml: top-level scalars, [a.b.c] tables, and
// key = <bool | int | "string" | ["a", "b"]>. No external dependency, matching
// ax25-ts's node:builtins-only posture.
function stripComment(line) {
  let inStr = false, q = "";
  for (let i = 0; i < line.length; i++) {
    const c = line[i];
    if (inStr) { if (c === q) inStr = false; continue; }
    if (c === '"' || c === "'") { inStr = true; q = c; continue; }
    if (c === "#") return line.slice(0, i);
  }
  return line;
}
function splitTopLevel(s) {
  const out = [];
  let cur = "", inStr = false, q = "";
  for (const c of s) {
    if (inStr) { cur += c; if (c === q) inStr = false; continue; }
    if (c === '"' || c === "'") { inStr = true; q = c; cur += c; continue; }
    if (c === ",") { if (cur.trim()) out.push(cur.trim()); cur = ""; continue; }
    cur += c;
  }
  if (cur.trim()) out.push(cur.trim());
  return out;
}
function parseValue(s) {
  s = s.trim();
  if (s === "true") return true;
  if (s === "false") return false;
  if (/^-?\d+$/.test(s)) return parseInt(s, 10);
  if ((s.startsWith('"') && s.endsWith('"')) || (s.startsWith("'") && s.endsWith("'"))) {
    return s.slice(1, -1);
  }
  if (s.startsWith("[")) {
    const inner = s.slice(1, s.lastIndexOf("]"));
    return inner.trim() ? splitTopLevel(inner).map(parseValue) : [];
  }
  return s; // bare fallback
}
function parseToml(text) {
  const root = {};
  let current = root;
  for (const raw of text.split(/\r?\n/)) {
    const line = stripComment(raw).trim();
    if (!line) continue;
    if (line.startsWith("[")) {
      const name = line.slice(1, line.indexOf("]")).trim();
      const parts = name.split(".").map((s) => s.trim().replace(/^["']|["']$/g, ""));
      current = root;
      for (const p of parts) {
        if (typeof current[p] !== "object" || current[p] === null) current[p] = {};
        current = current[p];
      }
      continue;
    }
    const eq = line.indexOf("=");
    if (eq < 0) continue;
    const key = line.slice(0, eq).trim().replace(/^["']|["']$/g, "");
    current[key] = parseValue(line.slice(eq + 1));
  }
  return root;
}

// ─── load inputs ──────────────────────────────────────────────────────
const manifest = parseToml(readFileSync(manifestPath, "utf8"));
const inventory = JSON.parse(readFileSync(inventoryPath, "utf8"));
const exceptions = JSON.parse(readFileSync(exceptionsPath, "utf8"));

const vectorSets = manifest.vector_sets ?? {};
const inventoryExceptions = exceptions.inventoryExceptions ?? {};
const declaredOut = exceptions.declaredOut ?? {};

console.log("pico-node parity drift guard");
console.log(`  manifest:   ${manifestPath}${usingFixture ? "  (FIXTURE fallback — real parity-manifest.toml absent)" : ""}`);
console.log(`  inventory:  ${inventoryPath}`);
console.log(`  exceptions: ${exceptionsPath}`);

let failures = 0;

// ─── 1. manifest schema validation ────────────────────────────────────
const schemaErrors = [];
if (manifest.schema === undefined) schemaErrors.push("top-level `schema` key is missing");
if (Object.keys(vectorSets).length === 0) schemaErrors.push("no [vector_sets.*] tables found");
for (const [name, set] of Object.entries(vectorSets)) {
  if (typeof set.in !== "boolean") {
    schemaErrors.push(`vector_sets.${name}: missing/invalid boolean \`in\``);
    continue;
  }
  if (set.in === false) {
    if (typeof set.reason !== "string" || !set.reason.trim()) {
      schemaErrors.push(`vector_sets.${name}: in=false requires a non-empty \`reason\``);
    }
  } else {
    if (!Array.isArray(set.capabilities) || set.capabilities.length === 0) {
      schemaErrors.push(`vector_sets.${name}: opted-in set requires non-empty \`capabilities\``);
    }
  }
}
console.log("\n[1] manifest schema");
if (schemaErrors.length) {
  for (const e of schemaErrors) console.log(`  ✗ ${e}`);
  failures += schemaErrors.length;
} else {
  const optIn = Object.entries(vectorSets).filter(([, s]) => s.in).map(([n]) => n);
  const optOut = Object.entries(vectorSets).filter(([, s]) => !s.in).map(([n]) => n);
  console.log(`  ✓ schema=${manifest.schema}, ${optIn.length} opted-in, ${optOut.length} declared-out`);
  console.log(`    opted-in:     ${optIn.join(", ")}`);
  console.log(`    declared-out: ${optOut.join(", ")}`);
}

// ─── 2. exceptions well-formedness ────────────────────────────────────
console.log("\n[2] exceptions registry");
let exceptionErrors = 0;
for (const [section, entries] of Object.entries(inventoryExceptions)) {
  for (const [name, reason] of Object.entries(entries)) {
    if (typeof reason !== "string" || !reason.trim()) {
      console.log(`  ✗ inventoryExceptions.${section}.${name}: empty reason`);
      exceptionErrors++;
    }
  }
}
for (const [name, reason] of Object.entries(declaredOut)) {
  if (typeof reason !== "string" || !reason.trim()) {
    console.log(`  ✗ declaredOut.${name}: empty reason`);
    exceptionErrors++;
  }
}
failures += exceptionErrors;
if (exceptionErrors === 0) {
  const nInvExc = Object.values(inventoryExceptions).reduce((a, e) => a + Object.keys(e).length, 0);
  console.log(`  ✓ ${nInvExc} inventory exception(s), ${Object.keys(declaredOut).length} declared-out area(s), all with reasons`);
}

// ─── 3. C# ⊆ pico-node-declared ───────────────────────────────────────
console.log("\n[3] C# inventory coverage  (C# ⊆ pico-node-declared)");
const rows = [];
let covered = 0, excepted = 0, gaps = 0;

for (const item of inventory.items ?? []) {
  const { section, name, coverage } = item;
  const exceptionReason = inventoryExceptions[section]?.[name];

  let status, detail;
  if (coverage && coverage.set) {
    const set = vectorSets[coverage.set];
    if (!set) {
      status = "GAP"; detail = `coverage set '${coverage.set}' not in manifest`;
    } else if (set.in !== true) {
      status = "GAP"; detail = `coverage set '${coverage.set}' is declared-out (in=false)`;
    } else if (coverage.capability && !(set.capabilities ?? []).includes(coverage.capability)) {
      status = "GAP"; detail = `set '${coverage.set}' does not declare capability '${coverage.capability}'`;
    } else {
      status = "OK"; detail = `${coverage.set}${coverage.capability ? " / " + coverage.capability : ""}`;
    }
  } else if (exceptionReason) {
    status = "EXCEPT"; detail = exceptionReason;
  } else {
    status = "GAP"; detail = "no coverage mapping and no exception";
  }

  // An item can be BOTH excepted and (accidentally) mapped — if mapping fails
  // but an exception exists, honour the exception rather than failing.
  if (status === "GAP" && exceptionReason) {
    status = "EXCEPT"; detail = exceptionReason;
  }

  if (status === "OK") covered++;
  else if (status === "EXCEPT") excepted++;
  else { gaps++; failures++; }
  rows.push({ status, section, name, detail });
}

// pretty per-item table
const glyph = { OK: "✓", EXCEPT: "~", GAP: "✗" };
const wSec = Math.max(7, ...rows.map((r) => r.section.length));
const wName = Math.max(4, ...rows.map((r) => r.name.length));
let lastSection = "";
for (const r of rows) {
  if (r.section !== lastSection) { console.log(""); lastSection = r.section; }
  const detail = r.status === "EXCEPT" && r.detail.length > 68 ? r.detail.slice(0, 65) + "..." : r.detail;
  console.log(`  ${glyph[r.status]} ${r.section.padEnd(wSec)}  ${r.name.padEnd(wName)}  ${detail}`);
}

// ─── verdict ──────────────────────────────────────────────────────────
console.log("\nsummary");
console.log(`  inventory items: ${rows.length}`);
console.log(`  covered:         ${covered}`);
console.log(`  excepted:        ${excepted}`);
console.log(`  gaps:            ${gaps}`);
if (schemaErrors.length) console.log(`  schema errors:   ${schemaErrors.length}`);
if (exceptionErrors) console.log(`  exception errors:${exceptionErrors}`);

console.log("");
if (failures > 0) {
  console.log(
    `PARITY DRIFT: ${failures} problem(s). For each C# inventory gap, either map it ` +
      `to an opted-in vector_set in parity-manifest.toml, or record a reviewed ` +
      `exception (with a reason) in parity-exceptions.json.`,
  );
  process.exit(1);
}
console.log(
  `Parity check passed (${covered} covered, ${excepted} documented exception(s)).` +
    (usingFixture ? "  [ran against the sample fixture manifest]" : ""),
);
