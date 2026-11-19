// The flight planner's predicted path (src/planner.html, <script id="sim">) is a JavaScript copy of the
// autopilot's NAV (FlightController.UpdateNav + NavGuidance's L1 guidance, turn lead and arc turn). This flies
// routes at the default MaxBank 35 deg / MaxTurnRate 3 deg/s and checks the turns against fly-by geometry:
// an arc of radius R cut across a corner of angle a passes R (1/cos(a/2) - 1) from it, and rolls out on the
// new leg instead of swinging past it.
// Run: node tests/planner-sim.js
const fs = require('fs'), path = require('path');
const html = fs.readFileSync(path.join(__dirname, '..', 'src', 'planner.html'), 'utf8');
const sim = html.match(/<script id="sim">([\s\S]*?)<\/script>/)[1];
const m = { exports: {} };
new Function('module', sim)(m);
const { predict } = m.exports;

const RAD = Math.PI / 180, G = 9.81;
const af = { gLimit: 9, corner: 150 }, set = { share: 0.8, maxBank: 35, maxTurnRate: 3 };
// turn radius: the 35 deg bank or 3 deg/s, whichever is wider (this airframe's g budget allows both)
const R = v => Math.max(v / (3 * RAD), v * v / (G * Math.tan(35 * RAD)));
const cut = (v, deg) => R(v) * (1 / Math.cos(deg / 2 * RAD) - 1);

// one corner of `turn` deg between 20 km legs: closest pass, and the most heading past the new leg's course
function corner(v, turn) {
  const a = turn * RAD;
  const wps = [[0, 0], [0, 20000], [20000 * Math.sin(a), 20000 + 20000 * Math.cos(a)]].map(([x, z]) => ({ x, z, h: 60, s: null }));
  const r = predict(wps, af, set, v);
  let over = 0;
  for (let i = 1; i < r.path.length; i++) {
    const [x0, z0] = r.path[i - 1], [x1, z1] = r.path[i];
    const d = Math.atan2(x1 - x0, z1 - z0) - a;
    over = Math.max(over, Math.atan2(Math.sin(d), Math.cos(d)) / RAD);
  }
  return { miss: r.miss[1], over };
}

const wps = [[0, 0], [0, 20000], [15000, 20000], [4400, 30600]].map(([x, z]) => ({ x, z, h: 60, s: null }));
const fast = predict(wps, af, set, 250);
const slow = predict(wps.map((w, k) => ({ ...w, s: k >= 2 ? 150 : null })), af, set, 250);
const checks = [
  [fast.time.every(t => !isNaN(t)), 'every waypoint reached'],
  [slow.miss[2] < fast.miss[2] * 0.75, `150 m/s from waypoint 3 turns tighter: ${slow.miss[2].toFixed(0)} m vs ${fast.miss[2].toFixed(0)} m`],
];
for (const v of [150, 250])
  for (const turn of [45, 90, 135]) {
    const c = corner(v, turn);
    checks.push([c.over < 5, `${v} m/s, ${turn} deg corner: rolls out on the new leg (${c.over.toFixed(0)} deg past its course)`]);
    checks.push([c.miss < cut(v, turn) * 1.15, `${v} m/s, ${turn} deg corner passed within ${c.miss.toFixed(0)} m (fly-by arc: ${cut(v, turn).toFixed(0)})`]);
  }
let failed = 0;
for (const [ok, what] of checks) if (!ok) { failed++; console.log('FAIL: ' + what); }
console.log(failed ? `${failed} check(s) failed.` : 'Planner sim check passed.');
process.exit(failed ? 1 : 0);
