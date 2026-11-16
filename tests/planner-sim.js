// The flight planner's predicted path (src/planner.html, <script id="sim">) is a JavaScript copy of the
// autopilot's NAV maths. This flies the same route as SelfCheck's NavModel and checks it agrees.
// Run: node tests/planner-sim.js
const fs = require('fs'), path = require('path');
const html = fs.readFileSync(path.join(__dirname, '..', 'src', 'planner.html'), 'utf8');
const sim = html.match(/<script id="sim">([\s\S]*?)<\/script>/)[1];
const m = { exports: {} };
new Function('module', sim)(m);
const { predict } = m.exports;

const af = { gLimit: 9, corner: 150 }, set = { share: 0.8, maxBank: 90, maxTurnRate: 40 };
const wps = [[0, 0], [0, 20000], [15000, 20000], [4400, 30600]].map(([x, z]) => ({ x, z, h: 60, s: null }));
const fast = predict(wps, af, set, 250);
const slow = predict(wps.map((w, k) => ({ ...w, s: k >= 2 ? 150 : null })), af, set, 250);
const checks = [
  [fast.time.every(t => !isNaN(t)), 'every waypoint reached'],
  [fast.miss[1] < 900, `90 deg corner passed within ${fast.miss[1].toFixed(0)} m (C# model: ~600)`],
  [fast.miss[2] < 3200, `135 deg corner passed within ${fast.miss[2].toFixed(0)} m (C# model: ~2700)`],
  [slow.miss[2] < fast.miss[2] / 2, `150 m/s from waypoint 3 turns tighter: ${slow.miss[2].toFixed(0)} m`],
];
let failed = 0;
for (const [ok, what] of checks) if (!ok) { failed++; console.log('FAIL: ' + what); }
console.log(failed ? `${failed} check(s) failed.` : 'Planner sim check passed.');
process.exit(failed ? 1 : 0);
