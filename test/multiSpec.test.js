'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const { computeSeamAvoidance } = require('../src/seamMath');
const {
  feasibleDegPieces,
  forbiddenDegPieces,
  intersectPieces,
  toCircularArcs,
  circDistDeg,
  gapForSpec,
  forbiddenArcsForSpec,
  intrusionForSpec,
  arcBoundaryPoints,
  computeContinuousFallback,
  computeMultiSpec,
  MIN_SPECS,
  MAX_SPECS,
} = require('../src/multiSpec');

function approx(actual, expected, eps = 1e-7) {
  assert.ok(Math.abs(actual - expected) <= eps, `${actual} !≈ ${expected}`);
}

function arcsToPairs(arcs) {
  return arcs.map((a) => [a.startDeg, a.endDeg]);
}

test('feasibleDegPieces: 跨零可行段拆为两段，整圈/无解正确', () => {
  // C=400 W=20 保护区 [60,120] → 禁用 36°–108°，可行段跨零（108°→36°）
  const r = computeSeamAvoidance(400, 20, [{ start: 60, end: 120 }]);
  assert.deepEqual(feasibleDegPieces(r), [
    [0, 36],
    [108, 360],
  ]);
  assert.deepEqual(forbiddenDegPieces(r), [[36, 108]]);

  const none = computeSeamAvoidance(400, 50, [
    { start: 0, end: 180 },
    { start: 150, end: 400 },
  ]);
  assert.equal(none.noSolution, true);
  assert.deepEqual(feasibleDegPieces(none), []);
  assert.deepEqual(forbiddenDegPieces(none), [[0, 360]]);

  const all = computeSeamAvoidance(400, 20, []);
  assert.deepEqual(feasibleDegPieces(all), [[0, 360]]);
  assert.deepEqual(forbiddenDegPieces(all), []);
});

test('intersectPieces: 普通、跨零与零交集', () => {
  // [0,90]∪[180,360] ∩ [0,45]∪[270,360] = [0,45]∪[270,360]
  assert.deepEqual(
    intersectPieces([
      [
        [0, 90],
        [180, 360],
      ],
      [
        [0, 45],
        [270, 360],
      ],
    ]),
    [
      [0, 45],
      [270, 360],
    ]
  );
  // 仅在一点相接不算共同解
  assert.deepEqual(intersectPieces([[[0, 90]], [[90, 180]]]), []);
  // 与整圈相交保持不变
  assert.deepEqual(intersectPieces([[[0, 360]], [[30, 60]]]), [[30, 60]]);
});

test('toCircularArcs: 零点两侧合并为跨零弧', () => {
  const { arcs } = toCircularArcs([
    [0, 36],
    [108, 360],
  ]);
  assert.equal(arcs.length, 1);
  assert.equal(arcs[0].startDeg, 108);
  assert.equal(arcs[0].endDeg, 36);
  assert.equal(arcs[0].wraps, true);
  approx(arcs[0].widthDeg, 288);

  const inner = toCircularArcs([
    [30, 90],
    [200, 260],
  ]).arcs;
  assert.deepEqual(arcsToPairs(inner), [
    [30, 90],
    [200, 260],
  ]);
  assert.equal(inner.every((a) => !a.wraps), true);

  assert.equal(toCircularArcs([[0, 360]]).full, true);
  assert.deepEqual(toCircularArcs([]).arcs, []);
});

test('circDistDeg: 最短圆周角距离', () => {
  approx(circDistDeg(10, 20), 10);
  approx(circDistDeg(350, 10), 20);
  approx(circDistDeg(0, 180), 180);
});

test('gapForSpec: 到最近禁用边界；无禁用区为 Infinity', () => {
  // 禁用 36°–108°
  const r = computeSeamAvoidance(400, 20, [{ start: 60, end: 120 }]);
  approx(gapForSpec(306, r), 90); // 距 36° 边界走短弧 90°
  approx(gapForSpec(72, r), 36);
  const empty = computeSeamAvoidance(400, 20, []);
  assert.equal(gapForSpec(123, empty), Infinity);
});

test('computeMultiSpec: 参数数量限制 2～8', () => {
  assert.equal(MIN_SPECS, 2);
  assert.equal(MAX_SPECS, 8);
  assert.throws(() => computeMultiSpec([400], 0, []), RangeError);
  assert.throws(
    () => computeMultiSpec([100, 200, 300, 400, 500, 600, 700, 800, 900], 0, []),
    RangeError
  );
});

test('computeMultiSpec: 两规格公共可行段为角度域交集，默认点最大化最小间隙', () => {
  // 共用 W=20、保护区 [60,120]mm：
  //  C=400：禁用 36°–108°，可行 108°→36°（跨零）
  //  C=200：禁用 72°–216°，可行 216°→72°（跨零）
  //  公共可行：216°→36°，宽 180°，中点 306°
  const r = computeMultiSpec([400, 200], 20, [{ start: 60, end: 120 }]);
  assert.equal(r.noSolution, false);
  assert.equal(r.fullCircle, false);
  assert.equal(r.common.length, 1);
  assert.equal(r.common[0].startDeg, 216);
  assert.equal(r.common[0].endDeg, 36);
  assert.equal(r.common[0].wraps, true);
  approx(r.common[0].widthDeg, 180);

  assert.equal(r.best.deg, 306);
  approx(r.best.minGapDeg, 90);
  // 两规格在 306° 处距各自禁用边界均为 90°
  approx(r.best.gaps[0].gapDeg, 90);
  approx(r.best.gaps[1].gapDeg, 90);
  assert.equal(r.best.gaps[0].mm, 340); // 306° 对应 C=400 规格的 340mm
  assert.equal(r.best.gaps[1].mm, 170); // C=200 规格的 170mm
  assert.equal(r.specs.length, 2);
});

test('computeMultiSpec: 公共段等宽并列时默认点取较小角度', () => {
  // 两个相同 C=360 规格：禁用 90°–180° 与 270°–360°
  // 可行段 [0,90]、[180,270] 等宽 → 中点 45° 与 225°，取 45°
  const r = computeMultiSpec(
    [360, 360],
    0,
    [
      { start: 90, end: 180 },
      { start: 270, end: 360 },
    ]
  );
  assert.equal(r.common.length, 2);
  assert.equal(r.best.deg, 45);
  approx(r.best.minGapDeg, 45);
});

test('computeMultiSpec: 所有规格均无禁用区时整圈公共、默认 0°', () => {
  const r = computeMultiSpec([400, 200], 0, []);
  assert.equal(r.fullCircle, true);
  assert.equal(r.best.deg, 0);
  assert.equal(r.best.minGapDeg, null);
  assert.deepEqual(
    r.best.gaps.map((g) => g.gapDeg),
    [null, null]
  );
});

test('computeMultiSpec: 无共同解时报告首次归零的规格与冲突区间', () => {
  // C=360：保护区 [90,180]、[180,360]，W=0 → 禁用 90°–360°，公共候选剩 [0,90]
  // C=180：两保护区在小周长上覆盖整圈 → 该规格本身无解，交集归零
  const r = computeMultiSpec(
    [360, 180],
    0,
    [
      { start: 90, end: 180 },
      { start: 180, end: 360 },
    ]
  );
  assert.equal(r.noSolution, true);
  assert.equal(r.best, null);
  assert.deepEqual(r.common, []);
  assert.ok(r.blocker);
  assert.equal(r.blocker.index, 1);
  assert.equal(r.blocker.circumference, 180);
  // 冲突区间 = 此前公共段 [0,90] ∩ 该规格禁用（整圈）= [0,90]
  assert.deepEqual(arcsToPairs(r.blocker.conflictSegments), [[0, 90]]);
  assert.deepEqual(arcsToPairs(r.blocker.priorCommon), [[0, 90]]);
  assert.deepEqual(arcsToPairs(r.blocker.blockingForbidden), [[0, 360]]);
});

test('computeMultiSpec: 首个规格自身无解时立即归零并标 0', () => {
  const r = computeMultiSpec(
    [400, 300],
    50,
    [
      { start: 0, end: 180 },
      { start: 150, end: 400 },
    ]
  );
  assert.equal(r.noSolution, true);
  assert.equal(r.blocker.index, 0);
  assert.equal(r.blocker.conflictSegments.length, 1);
  assert.equal(r.blocker.conflictSegments[0].full, true);
});

test('computeMultiSpec: 按输入顺序定位首个使交集归零的规格', () => {
  // 前两个 C=360 规格公共段 [0,90]，第三个 C=180 覆盖整圈 → blocker.index=2
  const r = computeMultiSpec(
    [360, 360, 180],
    0,
    [
      { start: 90, end: 180 },
      { start: 180, end: 360 },
    ]
  );
  assert.equal(r.noSolution, true);
  assert.equal(r.blocker.index, 2);
  assert.deepEqual(arcsToPairs(r.blocker.conflictSegments), [[0, 90]]);
});

test('computeMultiSpec: 8 个规格可正常求交，接缝加宽会缩小公共段', () => {
  const Cs = [300, 320, 340, 360, 380, 400, 420, 440];
  const zones = [{ start: 90, end: 150 }];
  const thin = computeMultiSpec(Cs, 0, zones);
  const wide = computeMultiSpec(Cs, 40, zones);
  assert.equal(thin.noSolution, false);
  assert.equal(wide.noSolution, false);
  const thinWidth = Math.max(...thin.common.map((a) => a.widthDeg));
  const wideWidth = Math.max(...wide.common.map((a) => a.widthDeg));
  assert.ok(wideWidth < thinWidth);
});

/* ---------------- 最小违约备选 ---------------- */

test('intrusionForSpec: 区外为 0，区内为到最近出口的沿弧距离', () => {
  // C=400 W=20 保护区 [60,120] → 禁用 36°–108°
  const r = computeSeamAvoidance(400, 20, [{ start: 60, end: 120 }]);
  const arcs = forbiddenArcsForSpec(r);
  assert.deepEqual(arcs.map((a) => [a.startDeg, a.endDeg]), [[36, 108]]);
  assert.equal(intrusionForSpec(0, arcs), 0);
  approx(intrusionForSpec(72, arcs), 36); // 弧中点
  approx(intrusionForSpec(36, arcs), 0); // 边界
  approx(intrusionForSpec(100, arcs), 8);

  // 跨零禁用弧 [270,90]（宽 180）
  const wrap = [
    { startDeg: 270, endDeg: 90, wraps: true, widthDeg: 180 },
  ];
  approx(intrusionForSpec(0, wrap), 90);
  approx(intrusionForSpec(280, wrap), 10);
  approx(intrusionForSpec(80, wrap), 10);
  assert.equal(intrusionForSpec(180, wrap), 0);

  // 整圈禁用：圆上到边界最短距离，对径点 180°
  const full = [{ startDeg: 0, endDeg: 0, wraps: false, widthDeg: 360, full: true }];
  assert.equal(intrusionForSpec(0, full), 0);
  approx(intrusionForSpec(180, full), 180);
  approx(intrusionForSpec(350, full), 10);
});

test('arcBoundaryPoints: 提取禁用弧边界（跨零弧保留 0 附近的两个端点）', () => {
  const r = computeSeamAvoidance(400, 20, [{ start: 60, end: 120 }]);
  assert.deepEqual(arcBoundaryPoints(forbiddenArcsForSpec(r)), [36, 108]);
  const none = computeSeamAvoidance(400, 20, []);
  assert.deepEqual(arcBoundaryPoints(forbiddenArcsForSpec(none)), []);
  const full = computeSeamAvoidance(400, 50, [
    { start: 0, end: 180 },
    { start: 150, end: 400 },
  ]);
  assert.deepEqual(arcBoundaryPoints(forbiddenArcsForSpec(full)), [0]);
});

test('computeMultiSpec: 有共同安全段时不产出备选，结果保持不变', () => {
  const r = computeMultiSpec([400, 200], 20, [{ start: 60, end: 120 }]);
  assert.equal(r.fallback, null);
  assert.equal(r.best.deg, 306);
});

test('computeMultiSpec: 禁用仅在边界接触时备选侵入深度为 0 并取较小角度', () => {
  // spec0(C360) 禁用 90°–360°（可行 [0,90]）；spec1(C180) 整圈禁用
  // 并集在 0°/90° 仅余接触点，最大侵入 0；0° 与 90° 并列取 0°
  const r = computeMultiSpec(
    [360, 180],
    0,
    [
      { start: 90, end: 180 },
      { start: 180, end: 360 },
    ]
  );
  assert.equal(r.noSolution, true);
  assert.ok(r.fallback);
  assert.equal(r.fallback.deg, 0);
  approx(r.fallback.maxIntrusionDeg, 0);
  approx(r.fallback.sumIntrusionDeg, 0);
  assert.equal(r.fallback.restrictedToDetents, false);
  assert.equal(r.fallback.detentIndex, null);
  assert.equal(r.fallback.intrusions.length, 2);
  assert.match(r.fallback.reason, /公共可行角度为空/);
});

test('computeMultiSpec: 最小违约备选最小化最大侵入（两个规格均无整圈禁用）', () => {
  // C0=134：保护区 [171,308]∪[263,457]mm、W=39 合并为一条跨零禁用弧
  //   241.791° → 147.761°（宽 265.97°）
  // C1=592：同输入产生非跨零禁用弧 80.270°–277.905°
  // 并集覆盖整圆；重叠区 (241.791,277.905) 两端对称，最优点为其中点
  const circs = [134, 592];
  const W = 39;
  const zones = [
    { start: 171, end: 308 },
    { start: 263, end: 457 },
  ];
  const r = computeMultiSpec(circs, W, zones);
  assert.equal(r.noSolution, true);
  assert.equal(r.specs.some((s) => s.result.noSolution), false);
  assert.ok(r.fallback);
  // 中点 (241.791 + 277.905)/2 = 259.848°，两规格侵入相等
  approx(r.fallback.deg, 259.8482250907624, 1e-7);
  approx(r.fallback.maxIntrusionDeg, 18.057180314643, 1e-7);
  approx(r.fallback.sumIntrusionDeg, 36.114360629286, 1e-7);
  approx(r.fallback.intrusions[0].intrusionDeg, r.fallback.intrusions[1].intrusionDeg, 1e-7);
  approx(r.fallback.intrusions[0].mm, (r.fallback.deg / 360) * 134, 1e-7);
  assert.equal(r.fallback.restrictedToDetents, false);
  assert.equal(r.fallback.detentIndex, null);
  assert.match(r.fallback.reason, /公共可行角度为空/);
});

test('computeMultiSpec: 备选连续最优解与逐点枚举一致（含各规格侵入明细）', () => {
  // 确定性 LCG 随机生成无共同解场景，逐 0.05° 枚举作为基准
  let seed = 42;
  const rnd = () => {
    seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
    return seed / 4294967296;
  };
  let tested = 0;
  for (let t = 0; t < 120 && tested < 12; t++) {
    const nspecs = 2 + Math.floor(rnd() * 4);
    const circs = Array.from({ length: nspecs }, () => 100 + rnd() * 500);
    const W = rnd() * 40;
    const zones = [];
    for (let i = 0, nz = 1 + Math.floor(rnd() * 3); i < nz; i++) {
      const a = rnd() * 500;
      zones.push({ start: a, end: a + rnd() * 200 });
    }
    const r = computeMultiSpec(circs, W, zones);
    if (!r.noSolution) continue;
    tested++;
    const arcs = r.specs.map((s) => forbiddenArcsForSpec(s.result));
    let best = null;
    for (let x = 0; x < 360; x += 0.05) {
      const ds = r.specs.map((s) => intrusionForSpec(x, arcs[s.index]));
      const mx = Math.max(...ds);
      const sm = ds.reduce((a, b) => a + b, 0);
      if (
        !best ||
        mx < best.mx - 1e-9 ||
        (Math.abs(mx - best.mx) <= 1e-9 &&
          (sm < best.sm - 1e-9 || (Math.abs(sm - best.sm) <= 1e-9 && x < best.d)))
      ) {
        best = { d: x, mx, sm };
      }
    }
    const f = r.fallback;
    assert.ok(f, `用例 ${tested} 无共同解应产出备选`);
    assert.ok(Math.abs(f.deg - best.d) <= 0.08, `角度 ${f.deg} ≈ ${best.d}`);
    assert.ok(Math.abs(f.maxIntrusionDeg - best.mx) <= 0.03);
    assert.ok(Math.abs(f.sumIntrusionDeg - best.sm) <= 0.06);
    // 明细与角度一致
    f.intrusions.forEach((g, k) => {
      approx(g.intrusionDeg, intrusionForSpec(f.deg, arcs[k]), 1e-7);
      approx(g.mm, (f.deg / 360) * circs[k], 1e-7);
      assert.equal(g.index, k);
    });
    const maxOfDetails = Math.max(...f.intrusions.map((g) => g.intrusionDeg));
    approx(maxOfDetails, f.maxIntrusionDeg, 1e-7);
  }
  assert.ok(tested >= 12, `随机用例中应至少覆盖 12 个无共同解场景（实际 ${tested}）`);
});

test('computeContinuousFallback: 整圈禁用规格的锚点为 0，备选自动退到锚点', () => {
  // 规格0 整圈禁用（单一边界锚点 0，对径点侵入 180°）；
  // 规格1 禁用 [90,180]，在锚点 0 安全 → 全局最优为 0°（max=0）
  const full = computeSeamAvoidance(400, 50, [
    { start: 0, end: 180 },
    { start: 150, end: 400 },
  ]);
  const other = computeSeamAvoidance(400, 0, [{ start: 100, end: 200 }]);
  const specs = [
    { index: 0, circumference: 400, result: full },
    { index: 1, circumference: 400, result: other },
  ];
  const specArcs = specs.map((s) => forbiddenArcsForSpec(s.result));
  const win = computeContinuousFallback(specs, specArcs);
  assert.ok(win);
  assert.equal(win.deg, 0);
  approx(win.maxIntrusionDeg, 0);
  // 对径点 180° 同时在规格1 禁用弧末端：整圈规格侵入 180°
  approx(intrusionForSpec(180, specArcs[0]), 180);
});

test('computeMultiSpec + 档位：无共同解时只在档位角度中选最小违约备选', () => {
  // 连续最优 259.848°（max≈18.057）；4 档 0/90/180/270 中 270° 的 max 最小：
  //   0°→118.209、90°→57.761、180°→97.905、270°→max(28.209,7.905)=28.209
  const circs = [134, 592];
  const W = 39;
  const zones = [
    { start: 171, end: 308 },
    { start: 263, end: 457 },
  ];
  const r4 = computeMultiSpec(circs, W, zones, { enabled: true, steps: 4 });
  assert.equal(r4.noSolution, true);
  assert.equal(r4.best, null);
  assert.ok(r4.fallback);
  assert.equal(r4.fallback.restrictedToDetents, true);
  assert.equal(r4.fallback.deg, 270);
  assert.equal(r4.fallback.detentIndex, 3);
  approx(r4.fallback.maxIntrusionDeg, 28.208955223880565, 1e-7);
  approx(r4.fallback.sumIntrusionDeg, 36.11436062928598, 1e-7);

  // 12 档（0,30,...,330）同样取 270（第 10 档）
  const r12 = computeMultiSpec(circs, W, zones, { enabled: true, steps: 12 });
  assert.equal(r12.fallback.deg, 270);
  assert.equal(r12.fallback.detentIndex, 9);
  approx(r12.fallback.maxIntrusionDeg, 28.208955223880565, 1e-7);

  // 偏移 15°、8 档：档位 15,45,...,345 中最优为 240°（max≈37.905）
  const r8 = computeMultiSpec(circs, W, zones, {
    enabled: true,
    steps: 8,
    offsetDeg: 15,
  });
  assert.equal(r8.fallback.deg, 240);
  assert.equal(r8.fallback.detentIndex, 5);
  approx(r8.fallback.maxIntrusionDeg, 37.90540540540541, 1e-7);
});

test('computeMultiSpec + 档位：备选档位并列时依次比较总侵入、较小角度', () => {
  // spec1 整圈禁用（边界 0）；规格0 禁用 90°–270°。4 档 0/90/180/270：
  // 0° 与 180° 的整圈规格侵入分别为 0、180 → 0° 最优（max 由规格0 在边界为 0）
  const r = computeMultiSpec(
    [360, 180],
    0,
    [
      { start: 90, end: 180 },
      { start: 180, end: 360 },
    ],
    { enabled: true, steps: 4 }
  );
  assert.equal(r.fallback.deg, 0);
  assert.equal(r.fallback.detentIndex, 0);
  approx(r.fallback.maxIntrusionDeg, 0);
});
