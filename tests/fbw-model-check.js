// Phase-2 check of an FBW telemetry log (Telemetry = true, any Mode):
//  1. Aero model: our plate against the force and moment the game's aero job produced, per tick.
//  2. Rigid body: angular acceleration predicted from the game's forces + thrust + Unity damping about
//     the CG, against the measured one over the same physics step, per axis.
// Only airborne rows count (speed above 60 m/s, so the gear is off the ground).
//
// Usage: node tests/fbw-model-check.js <log.csv> [more.csv ...]
'use strict';
const fs = require('fs');
const path = require('path');

const pct = (v, p) => { const s = v.slice().sort((a, b) => a - b); return s.length ? s[Math.min(s.length - 1, Math.floor(p * s.length))] : NaN; };
let failed = 0;

for (const file of process.argv.slice(2)) {
  const lines = fs.readFileSync(file, 'utf8').trim().split(/\r?\n/);
  const head = lines[0].split(',');
  const col = n => head.indexOf(n);
  if (col('aeroErrF') < 0) { console.log(`${path.basename(file)}: no aero-check columns (an older log)`); continue; }
  const rows = lines.slice(1).map(l => l.split(',').map(Number)).filter(r => r[col('speed')] > 60);
  console.log(`== ${path.basename(file)}: ${rows.length} airborne rows`);
  if (rows.length < 60) { console.log('   too few airborne rows'); continue; }

  // 1. aero model vs the game's aero job
  const f = rows.map(r => r[col('aeroErrF')]), m = rows.map(r => r[col('aeroErrM')]), w = rows.map(r => r[col('aeroWorst')]);
  const show = v => `median ${(100 * pct(v, 0.5)).toFixed(3)}%  p95 ${(100 * pct(v, 0.95)).toFixed(3)}%  max ${(100 * pct(v, 1)).toFixed(3)}%`;
  console.log(`   aero force   ${show(f)}`);
  console.log(`   aero moment  ${show(m)}`);
  console.log(`   worst part   ${show(w)}`);
  const aeroOk = pct(m, 0.95) < 0.01;
  console.log(`   aero model ${aeroOk ? 'OK (p95 moment error under 1%)' : 'MISMATCH'}`);
  if (!aeroOk) failed++;

  // 2. rigid-body prediction vs measured, per axis: least-squares gain k (measured = k * predicted), R^2, residual
  const fits = [['roll ', 'pdot', 'pdotPred'], ['pitch', 'qdot', 'qdotPred'], ['yaw  ', 'rdot', 'rdotPred']];
  if (col('pdotProbe') >= 0) fits.push(['roll  (probe)', 'pdot', 'pdotProbe'], ['pitch (probe)', 'qdot', 'qdotProbe'], ['yaw   (probe)', 'rdot', 'rdotProbe']);
  for (const [axis, meas, pred] of fits) {
    const pairs = rows.map(r => [r[col(pred)], r[col(meas)]]).filter(([x, y]) => !Number.isNaN(x) && !Number.isNaN(y));
    let sxx = 0, sxy = 0, syy = 0, sr = 0;
    for (const [x, y] of pairs) { sxx += x * x; sxy += x * y; syy += y * y; }
    const k = sxy / sxx;
    for (const [x, y] of pairs) sr += (y - x) * (y - x);
    const r2 = 1 - sr / syy;
    const rmsMeas = Math.sqrt(syy / pairs.length), rmsRes = Math.sqrt(sr / pairs.length);
    console.log(`   ${axis} accel: measured rms ${rmsMeas.toFixed(1)} deg/s^2, prediction error rms ${rmsRes.toFixed(1)}, fit gain ${k.toFixed(3)}, R^2 ${r2.toFixed(3)}`);
  }
  if (col('mOtherM') >= 0) {
    const rms = n => Math.sqrt(rows.reduce((s, r) => s + (r[col(n)] || 0) ** 2, 0) / rows.length);
    console.log(`   scripted moment not in the itemised sum (rms N m): roll ${rms('mOtherL').toFixed(0)}  pitch ${rms('mOtherM').toFixed(0)}  yaw ${rms('mOtherN').toFixed(0)}` +
      `   (thrust moment rms: ${rms('mThrustL').toFixed(0)} / ${rms('mThrustM').toFixed(0)} / ${rms('mThrustN').toFixed(0)})`);
    const cg = ['cgX', 'cgY', 'cgZ'].map(n => pct(rows.map(r => r[col(n)]), 0.5).toFixed(3));
    console.log(`   CG from the root rigidbody (median, Unity body axes x/y/z, m): ${cg.join(' / ')}`);
  }
}
process.exit(failed ? 1 : 0);
