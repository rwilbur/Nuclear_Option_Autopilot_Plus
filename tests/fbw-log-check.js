// Phase-1 check of an FBW telemetry log: replays the game's control-surface job (ControlSurfaceJob_Math,
// docs/fbw/PHASE0_FACTS.md section 3) from the logged shared inputs and the jet's exported surface
// settings, and compares it with each surface's logged deflection. In Passthrough (and Off) every surface
// reads the shared inputs, so the replay must match to float precision.
//
// Usage: node tests/fbw-log-check.js <log.csv> [airframe.json]
//   airframe.json defaults to tests/airframes/<aircraft>.json, from the log's file name.
'use strict';
const fs = require('fs');
const path = require('path');

const [logPath, jsonArg] = process.argv.slice(2);
if (!logPath) { console.error('usage: node tests/fbw-log-check.js <log.csv> [airframe.json]'); process.exit(2); }
const unit = path.basename(logPath).replace(/_\d{8}_\d{6}\.csv$/, '');
const jsonPath = jsonArg || path.join(__dirname, 'airframes', unit + '.json');
const airframe = JSON.parse(fs.readFileSync(jsonPath, 'utf8'));
const field = (c, k) => { const l = c.fields.find(x => x.startsWith(k + '=')); return l === undefined ? undefined : l.slice(k.length + 1); };

const lines = fs.readFileSync(logPath, 'utf8').trim().split(/\r?\n/);
const head = lines[0].split(',');
const col = name => head.indexOf(name);
const rows = lines.slice(1).map(l => l.split(',').map(Number));
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));

let worst = 0, failed = 0;
for (const s of airframe.surfaces) {
  const name = s.path.split('/').pop();
  const ia = col(name + '_ang');
  if (ia < 0) { console.log(`${name}: not in the log`); continue; }
  const range = ['pitchRange', 'rollRange', 'yawRange'].map(k => +field(s, k));
  const servo = +field(s, 'servoSpeed');
  if (field(s, 'flap') === 'True') { console.log(`${name}: flap (follows the gear), skipped`); continue; }
  const inCols = ['pitch', 'roll', 'yaw'].map(col);

  // Unknown channel split at the start: assume each channel sits at its target (the jet at rest).
  let cur = range.map((r, c) => clamp(rows[0][inCols[c]] * r, -Math.abs(r), Math.abs(r)));
  let maxErr = 0, at = 0;
  for (let k = 1; k < rows.length; k++) {
    const dt = rows[k - 1][col('dt')];
    // the job at tick k-1 read row k-1's inputs; row k logs its result
    cur = cur.map((x, c) => {
      const step = clamp(rows[k - 1][inCols[c]] * range[c] - x, -servo * dt, servo * dt);
      return clamp(x + step, -Math.abs(range[c]), Math.abs(range[c]));
    });
    const logged = rows[k][ia];
    if (Number.isNaN(logged)) continue; // surface gone
    const err = Math.abs(cur[0] + cur[1] + cur[2] - logged);
    if (k > 60 && err > maxErr) { maxErr = err; at = k; } // the first second can carry the start-up guess
  }
  worst = Math.max(worst, maxErr);
  const ok = maxErr < 0.01;
  if (!ok) failed++;
  console.log(`${name.padEnd(14)} max |replay - logged| ${maxErr.toFixed(4)} deg${ok ? '' : `  MISMATCH (row ${at}, t ${rows[at][col('t')]})`}`);
}
const held = rows.filter(r => r[col('held')] === 1).length;
console.log(`${rows.length} rows, ${held} held (Passthrough), worst ${worst.toFixed(4)} deg: ${failed ? 'FAIL' : 'OK'}`);
process.exit(failed ? 1 : 0);
