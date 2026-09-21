'use strict';

/**
 * 多规格共用角度算法。
 *
 * 输入 2～8 个筒体周长（mm），沿用同一接缝安全宽度 W 与同一组毫米制保护区，
 * 对每个规格分别调用 computeSeamAvoidance，再把可行段统一归一化到角度域
 * [0°, 360°) 求圆周交集，输出所有规格均安全的角度段。
 *
 * 默认角度：对候选 θ，取 θ 在各规格下到该规格禁用边界的圆周角距离的最小值
 * g(θ) = min_k dist(θ, ∂F_k)，在公共可行段上最大化 g；并列取较小角度。
 * 公共可行段（圆相邻并集边界之间的空隙）内 g(θ) = min(θ-s, e-θ)，
 * 最大值落在最宽公共段中点，因此可直接按“最宽段中点、并列取小角”选取，
 * 各规格的实际间隙仍逐一分量计算并随结果返回。
 *
 * 无共同解时，按输入顺序逐个累计交集，报告使交集首次归零的规格，
 * 并给出此前公共可行段与该规格禁用段的冲突区间。
 *
 * 公共可行段为空（无共同解）时，额外计算“最小违约备选”：
 * 侵入深度 d_k(θ) 为 θ 在规格 k 禁用区内到最近禁用边界（出口）的沿弧
 * 距离，区外为 0；按各规格禁用边界切分整圆，求使 max_k d_k(θ) 最小
 * 的角度，并列时依次比较总侵入深度 Σ_k d_k(θ)、较小角度。启用设备
 * 档位约束时只在档位角度中求解。有共同安全段时不产出备选（null）。
 */

const { computeSeamAvoidance } = require('./seamMath');
const { parseDetents, analyzeDetents, detentPositions } = require('./detents');

const FULL = 360;
const EPS = 1e-7; // 角度比较容差
const MIN_SPECS = 2;
const MAX_SPECS = 8;

function mod(n, m) {
  return ((n % m) + m) % m;
}

function clamp360(x) {
  return Math.max(0, Math.min(FULL, x));
}

/** 排序、裁剪并合并紧贴/重叠的 [0,360] 线性片段。 */
function normalizePieces(pieces) {
  const sorted = pieces
    .map(([a, b]) => [clamp360(a), clamp360(b)])
    .filter(([a, b]) => b - a > EPS)
    .sort((x, y) => x[0] - y[0] || x[1] - y[1]);
  const merged = [];
  for (const [a, b] of sorted) {
    const last = merged[merged.length - 1];
    if (last && a <= last[1] + EPS) {
      last[1] = Math.max(last[1], b);
    } else {
      merged.push([a, b]);
    }
  }
  return merged.map(([a, b]) => [a, b >= FULL - EPS ? FULL : b]);
}

function isFullCircle(pieces) {
  return (
    pieces.length === 1 && pieces[0][0] <= EPS && pieces[0][1] >= FULL - EPS
  );
}

/**
 * 单规格结果 → 可行角度片段（均在 [0,360] 内、不跨零、已排序）。
 * 无解返回 []；整圈可行返回 [[0,360]]。
 */
function feasibleDegPieces(result) {
  if (result.noSolution) return [];
  const pieces = [];
  for (const g of result.feasible) {
    const width =
      g.widthDeg !== undefined
        ? g.widthDeg
        : (g.wraps ? g.endDeg + FULL : g.endDeg) - g.startDeg;
    if (width <= EPS) continue;
    if (width >= FULL - EPS) return [[0, FULL]];
    const s = mod(g.startDeg, FULL);
    const e = s + width;
    if (e >= FULL - EPS) {
      if (FULL - s > EPS) pieces.push([s, FULL]);
      if (e - FULL > EPS) pieces.push([0, e - FULL]);
    } else {
      pieces.push([s, e]);
    }
  }
  return normalizePieces(pieces);
}

/** 单规格结果 → 合并禁用角度片段。无解（整圈禁用）返回 [[0,360]]。 */
function forbiddenDegPieces(result) {
  if (result.noSolution) return [[0, FULL]];
  return normalizePieces(
    (result.forbiddenMerged || []).map((f) => [f.startDeg, f.endDeg])
  );
}

function insideAny(pieces, x) {
  return pieces.some(([a, b]) => x >= a - EPS && x <= b + EPS);
}

/**
 * 圆周交集：以所有片段端点（含 0、360）切分原子区间，
 * 中点被全部片段集覆盖者保留，再合并相邻片段。
 * @param {Array<Array<[number, number]>>} sets 已归一化的 [0,360] 片段集数组
 */
function intersectPieces(sets) {
  if (sets.length === 0) return [];
  const points = new Set([0, FULL]);
  for (const set of sets) {
    for (const [a, b] of set) {
      points.add(clamp360(a));
      points.add(clamp360(b));
    }
  }
  const cuts = [...points].sort((x, y) => x - y);
  const out = [];
  for (let i = 0; i < cuts.length - 1; i++) {
    const x0 = cuts[i];
    const x1 = cuts[i + 1];
    if (x1 - x0 <= EPS) continue;
    const mid = (x0 + x1) / 2;
    if (sets.every((set) => insideAny(set, mid))) out.push([x0, x1]);
  }
  return normalizePieces(out);
}

/**
 * 把不跨零片段还原为圆周弧段：末端 360 与始端 0 的两段合并为一条跨零弧。
 * 整圈返回 {full:true, arcs:[]}。
 * @returns {{full:boolean, arcs:Array<{startDeg:number,endDeg:number,wraps:boolean,widthDeg:number}>}}
 */
function toCircularArcs(pieces) {
  const p = normalizePieces(pieces);
  if (p.length === 0) return { full: false, arcs: [] };
  if (isFullCircle(p)) {
    return {
      full: true,
      arcs: [
        { startDeg: 0, endDeg: 0, wraps: false, widthDeg: FULL, full: true },
      ],
    };
  }
  const first = p[0];
  const last = p[p.length - 1];
  const arcs = [];
  if (first[0] <= EPS && last[1] >= FULL - EPS && p.length >= 2) {
    const width = FULL - last[0] + first[1];
    arcs.push({
      startDeg: last[0],
      endDeg: first[1],
      wraps: true,
      widthDeg: width,
    });
    for (let i = 1; i < p.length - 1; i++) {
      arcs.push({
        startDeg: p[i][0],
        endDeg: p[i][1],
        wraps: false,
        widthDeg: p[i][1] - p[i][0],
      });
    }
  } else {
    for (const seg of p) {
      arcs.push({
        startDeg: seg[0],
        endDeg: seg[1],
        wraps: false,
        widthDeg: seg[1] - seg[0],
      });
    }
  }
  // 统一按起点角度排序输出
  arcs.sort((x, y) => x.startDeg - y.startDeg);
  return { full: false, arcs };
}

/** 圆上两点最短角距离。 */
function circDistDeg(a, b) {
  const d = Math.abs(mod(a - b, FULL));
  return Math.min(d, FULL - d);
}

/** 取一个规格的禁用边界点（角度，模 360 去重）。 */
function forbiddenBoundaryDeg(result) {
  const pts = [];
  for (const f of result.forbiddenMerged || []) {
    for (const p of [f.startDeg, f.endDeg]) {
      const q = mod(p, FULL);
      if (!pts.some((x) => Math.abs(x - q) <= EPS || Math.abs(x - q) >= FULL - EPS)) {
        pts.push(q);
      }
    }
  }
  return pts;
}

/** θ 到某规格最近禁用边界的角距离；该规格无禁用区时为 Infinity。 */
function gapForSpec(deg, result) {
  const pts = forbiddenBoundaryDeg(result);
  if (pts.length === 0) return Infinity;
  return pts.reduce((best, p) => Math.min(best, circDistDeg(deg, p)), Infinity);
}

/* ---------------- 最小违约备选 ---------------- */

/**
 * 某规格的禁用区圆周弧（由线性禁用片段还原，跨零合并）。
 * 无解（整圈禁用）返回一条 {full:true} 弧。
 */
function forbiddenArcsForSpec(result) {
  const pieces = forbiddenDegPieces(result);
  const conv = toCircularArcs(pieces);
  return conv.arcs;
}

/**
 * 角度 θ 在某规格禁用区内的侵入深度（角度）：区内为到最近禁用边界（出口）
 * 的沿弧距离，区外为 0；整圈禁用时为圆上到该规格边界的最短距离
 * （对径点达 180°）。
 */
function intrusionForSpec(deg, arcs) {
  let best = 0;
  for (const a of arcs) {
    if (a.full) {
      best = Math.max(best, a.startDeg === undefined ? 0 : circDistDeg(deg, a.startDeg));
      continue;
    }
    const inside = a.wraps
      ? deg >= a.startDeg - EPS || deg <= a.endDeg + EPS
      : deg >= a.startDeg - EPS && deg <= a.endDeg + EPS;
    if (!inside) continue;
    if (a.wraps) {
      // 跨零弧 [s,360]∪[0,e]：θ≥s 时 e 提升到 360+e；θ≤e 时 s 降到 s-360
      const d =
        deg >= a.startDeg - EPS
          ? Math.min(deg - a.startDeg, a.endDeg + FULL - deg)
          : Math.min(deg + FULL - a.startDeg, a.endDeg - deg);
      if (d > best) best = d;
    } else {
      const d = Math.min(deg - a.startDeg, a.endDeg - deg);
      if (d > best) best = d;
    }
  }
  return Math.max(0, best);
}

/** 圆周弧集 → 边界点（模 360 去重，升序）。整圈弧取其锚点 0。 */
function arcBoundaryPoints(arcs) {
  const pts = [];
  for (const a of arcs) {
    if (a.full) {
      if (!pts.some((x) => x <= EPS)) pts.push(0);
      continue;
    }
    for (const p of [a.startDeg, a.endDeg]) {
      const q = mod(p, FULL);
      if (!pts.some((x) => Math.abs(x - q) <= EPS)) pts.push(q);
    }
  }
  return pts.sort((x, y) => x - y);
}

/**
 * 计算全部规格在 deg 处的侵入明细。
 * @returns {Array<{index:number,circumference:number,intrusionDeg:number,mm:number}>}
 */
function intrusionDetailsAt(specs, specArcs, deg) {
  return specs.map((s) => ({
    index: s.index,
    circumference: s.circumference,
    intrusionDeg: intrusionForSpec(deg, specArcs[s.index]),
    mm: (deg / FULL) * s.circumference,
  }));
}

/** 候选打分：(最大侵入, 总侵入, 角度) 字典序越小越优。 */
function betterFallbackPoint(curr, cand) {
  if (cand.maxIntrusionDeg < curr.maxIntrusionDeg - EPS) return true;
  if (cand.maxIntrusionDeg > curr.maxIntrusionDeg + EPS) return false;
  if (cand.sumIntrusionDeg < curr.sumIntrusionDeg - EPS) return true;
  if (cand.sumIntrusionDeg > curr.sumIntrusionDeg + EPS) return false;
  return cand.deg < curr.deg - EPS;
}

function scoreFallbackPoint(specs, specArcs, deg) {
  const details = intrusionDetailsAt(specs, specArcs, deg);
  const sum = details.reduce((acc, d) => acc + d.intrusionDeg, 0);
  const max = details.reduce((m, d) => Math.max(m, d.intrusionDeg), 0);
  return { deg, maxIntrusionDeg: max, sumIntrusionDeg: sum, intrusions: details };
}

/**
 * 连续角度域的最小违约备选。
 *
 * 以各规格禁用边界切分整圆，并把每段禁用弧的中点（各规格“距边界最远”
 * 的折线折点）一并作为切分点；每个原子区间内，各规格侵入深度均为斜率
 * ±1（或 0）的线性函数，故 f(θ)=max_k d_k(θ) 的最小值只可能落在切分点
 * 或“上升包络与下降包络的交点”。逐格枚举这些候选后按
 * （最大侵入、总侵入、角度）字典序取最优。
 * @param {Array} specs computeMultiSpec 内部规格数组
 * @param {Array} specArcs 各规格禁用圆周弧（与 specs 对齐）
 */
function computeContinuousFallback(specs, specArcs) {
  const active = specs.filter((s) => specArcs[s.index].length > 0);
  if (active.length === 0) return null; // 所有规格均无禁用区（理论上不会走到）

  const cutsRaw = [0, FULL];
  const specPts = new Map();
  for (const s of active) {
    const pts = arcBoundaryPoints(specArcs[s.index]);
    specPts.set(s.index, pts);
    for (const p of pts) cutsRaw.push(p);
    // 提升整圈：…,b0-360,b1-360,b0,b1,b0+360,b1+360,…（边界数为 1 时自然退化为整圈）
    let lifts;
    if (pts.length === 1) {
      lifts = [pts[0] - FULL, pts[0], pts[0] + FULL];
    } else {
      lifts = [];
      for (const p of pts) lifts.push(p - FULL, p, p + FULL);
      lifts.sort((a, b) => a - b);
    }
    for (let i = 0; i < lifts.length - 1; i++) {
      const mid = (lifts[i] + lifts[i + 1]) / 2;
      if (mid > EPS && mid < FULL - EPS) cutsRaw.push(mod(mid, FULL));
    }
  }

  const cuts = [];
  for (const x of cutsRaw.sort((a, b) => a - b)) {
    if (!cuts.some((y) => Math.abs(y - x) <= EPS)) cuts.push(x);
  }

  const nearestLift = (deg, pts) => {
    let best = null;
    for (const p0 of pts) {
      for (const L of [p0 - FULL, p0, p0 + FULL]) {
        const d = Math.abs(deg - L);
        if (best === null || d < best.d - EPS) best = { L, d };
      }
    }
    return best.L;
  };

  const candidates = [];
  const seenDeg = (d) => candidates.some((x) => Math.abs(x - d) <= EPS);

  for (let i = 0; i < cuts.length - 1; i++) {
    const x0 = cuts[i];
    const x1 = cuts[i + 1];
    if (x1 - x0 <= EPS) continue;
    const mid = (x0 + x1) / 2;

    const fallingL = []; // 最近边界在右侧：d_k(θ) = L - θ，斜率 -1
    const risingL = []; // 最近边界在左侧：d_k(θ) = θ - L，斜率 +1
    for (const s of active) {
      const arcs = specArcs[s.index];
      const depth = intrusionForSpec(mid, arcs);
      if (depth <= EPS) continue; // 该格中该规格不违约
      const L = nearestLift(mid, specPts.get(s.index));
      if (L > mid + EPS) fallingL.push(L);
      else if (L < mid - EPS) risingL.push(L);
    }
    // 无违约规格意味着存在公共安全段（调用方保证不会出现）
    if (fallingL.length === 0 && risingL.length === 0) continue;

    // 上升包络 θ - min(L升) 与下降包络 max(L降) - θ 的交点
    if (fallingL.length > 0 && risingL.length > 0) {
      const lMin = Math.min(...risingL);
      const rMax = Math.max(...fallingL);
      const cross = (lMin + rMax) / 2;
      if (cross > x0 + EPS && cross < x1 - EPS) candidates.push(mod(cross, FULL));
    }
  }
  for (const c of cuts) {
    const d = mod(c, FULL);
    if (!seenDeg(d)) candidates.push(d);
  }

  let winner = null;
  for (const deg of candidates) {
    const score = scoreFallbackPoint(specs, specArcs, mod(deg, FULL));
    if (!winner || betterFallbackPoint(winner, score)) winner = score;
  }
  return winner;
}

/**
 * 设备档位角度域的最小违约备选：只在档位角度中按
 * （最大侵入、总侵入、角度）字典序选最优。
 * @returns {object|null} 附 detentIndex 的备选
 */
function computeDetentFallback(specs, specArcs, detentCfg) {
  const positions = detentPositions(detentCfg);
  let winner = null;
  positions.forEach((deg, index) => {
    const score = scoreFallbackPoint(specs, specArcs, deg);
    score.detentIndex = index;
    if (!winner || betterFallbackPoint(winner, score)) winner = score;
  });
  return winner;
}

/**
 * 构造最小违约备选返回结构。
 * @param {Array} specs
 * @param {Array} specArcs
 * @param {object|null} detentCfg 启用时的档位配置
 * @returns {{deg:number,maxIntrusionDeg:number,sumIntrusionDeg:number,intrusions:Array,
 *            restrictedToDetents:boolean,detentIndex:number|null,reason:string}|null}
 */
function buildFallback(specs, specArcs, detentCfg) {
  const win = detentCfg && detentCfg.enabled
    ? computeDetentFallback(specs, specArcs, detentCfg)
    : computeContinuousFallback(specs, specArcs);
  if (!win) return null;
  return {
    deg: win.deg,
    maxIntrusionDeg: win.maxIntrusionDeg,
    sumIntrusionDeg: win.sumIntrusionDeg,
    intrusions: win.intrusions,
    restrictedToDetents: !!(detentCfg && detentCfg.enabled),
    detentIndex: win.detentIndex === undefined ? null : win.detentIndex,
    reason:
      '公共可行角度为空：各规格禁用区的并集覆盖整圆（无对所有规格同时安全的角度）',
  };
}

/**
 * 构造首次归零说明。
 * @param {object} spec 当前规格 {index, circumference, result}
 * @param {Array<[number,number]>} prior 与此规格求交前的公共可行片段
 * @param {Array<[number,number]>} forbidden 当前规格的禁用片段
 */
function buildBlocker(spec, prior, forbidden) {
  const overlapPieces = intersectPieces([prior, forbidden]);
  const blocking = forbidden.filter(([a, b]) =>
    prior.some(([pa, pb]) => b > pa + EPS && a < pb - EPS)
  );
  return {
    index: spec.index,
    circumference: spec.circumference,
    priorCommon: toCircularArcs(prior).arcs,
    blockingForbidden: blocking.map(([a, b]) => ({ startDeg: a, endDeg: b })),
    conflictSegments: toCircularArcs(overlapPieces).arcs,
  };
}

/**
 * 多规格共用角度主入口。
 * @param {number[]} circumferences 2～8 个周长 (mm)
 * @param {number} seamWidth 共用接缝安全宽度 (mm)
 * @param {Array<{start:number,end:number}>} zones 共用印刷保护区 (mm)
 * @param {object} [detentsOpt] 设备档位约束
 *        {enabled:boolean, steps?:2～360 整数, offsetDeg?:number}
 */
function computeMultiSpec(circumferences, seamWidth, zones, detentsOpt) {
  if (
    !Array.isArray(circumferences) ||
    circumferences.length < MIN_SPECS ||
    circumferences.length > MAX_SPECS
  ) {
    throw new RangeError(
      `circumferences 必须为含 ${MIN_SPECS}～${MAX_SPECS} 个周长的数组`
    );
  }
  const detentCfg = parseDetents(detentsOpt);

  const specs = circumferences.map((C, index) => ({
    index,
    circumference: C,
    result: computeSeamAvoidance(C, seamWidth, zones),
  }));
  const feasibleSets = specs.map((s) => feasibleDegPieces(s.result));
  const forbiddenSets = specs.map((s) => forbiddenDegPieces(s.result));

  // 按输入顺序累计交集，定位使公共交集首次归零的规格
  let acc = [[0, FULL]];
  let blocker = null;
  for (const spec of specs) {
    const next = intersectPieces([acc, feasibleSets[spec.index]]);
    if (next.length === 0) {
      blocker = buildBlocker(spec, acc, forbiddenSets[spec.index]);
      break;
    }
    acc = next;
  }

  const common = blocker ? [] : toCircularArcs(acc).arcs;
  const fullCircle = !blocker && isFullCircle(acc);

  // 公共可行段为空时计算“最小违约备选”（启用档位时只在档位角度中求解）
  let fallback = null;
  if (blocker) {
    const specArcs = specs.map((s) => forbiddenArcsForSpec(s.result));
    fallback = buildFallback(specs, specArcs, detentCfg);
  }

  let best = null;
  let detents = detentCfg.enabled
    ? {
        enabled: true,
        steps: detentCfg.steps,
        offsetDeg: detentCfg.offsetDeg,
        stepDeg: FULL / detentCfg.steps,
        positions: Array.from(
          { length: detentCfg.steps },
          (_, k) => mod(detentCfg.offsetDeg + (k * FULL) / detentCfg.steps, FULL)
        ).sort((a, b) => a - b),
        available: [],
        best: null,
        noAvailable: false,
        nearest: [],
      }
    : { enabled: false };
  if (!blocker) {
    const gapsAt = (deg) =>
      specs.map((s) => {
        const gap = gapForSpec(deg, s.result);
        return {
          index: s.index,
          circumference: s.circumference,
          gapDeg: Number.isFinite(gap) ? gap : null, // null：该规格无禁用边界
          mm: (deg / FULL) * s.circumference,
        };
      });

    // 瓶颈间隙：各规格到禁用边界距离的最小值；全部规格均无禁用区时为 null（无约束）
    const minGapOfGaps = (gaps) => {
      const finite = gaps.filter((g) => g.gapDeg !== null);
      return finite.length
        ? finite.reduce((m, g) => (g.gapDeg < m ? g.gapDeg : m), Infinity)
        : null;
    };
    const minGapSpecOfGaps = (gaps, minGap) => {
      let idx = null;
      for (const g of gaps) {
        if (g.gapDeg !== null && g.gapDeg <= minGap + EPS && (idx === null || g.index < idx)) {
          idx = g.index;
        }
      }
      return idx;
    };

    if (detentCfg.enabled) {
      // 档位与共用段相交；默认档位沿用“瓶颈间隙最大、角度较小优先”
      const analysis = analyzeDetents(
        detentCfg,
        common,
        (deg) => minGapOfGaps(gapsAt(deg)),
        gapsAt
      );
      detents = {
        enabled: true,
        steps: analysis.steps,
        offsetDeg: analysis.offsetDeg,
        stepDeg: analysis.stepDeg,
        positions: analysis.positions,
        available: analysis.available.map((c) => ({
          index: c.index,
          deg: c.deg,
          minGapDeg: c.minGapDeg,
          gaps: c.detail,
        })),
        best: null,
        noAvailable: analysis.noAvailable,
        nearest: analysis.nearest,
      };
      if (analysis.best) {
        const win = analysis.best;
        detents.best = {
          index: win.index,
          deg: win.deg,
          minGapDeg: win.minGapDeg,
          minGapSpecIndex:
            win.minGapDeg === null ? null : minGapSpecOfGaps(win.detail, win.minGapDeg),
          gaps: win.detail,
        };
        best = {
          deg: win.deg,
          minGapDeg: win.minGapDeg,
          minGapSpecIndex: detents.best.minGapSpecIndex,
          gaps: win.detail,
          detentIndex: win.index,
        };
      }
      // noAvailable（连续有解但无可用档位）时 best 保持 null
    } else if (fullCircle) {
      // 所有规格均无禁用区：整圈等距，按“角度最小”取 0
      best = {
        deg: 0,
        minGapDeg: null,
        minGapSpecIndex: null,
        gaps: gapsAt(0),
      };
    } else {
      // 每条公共弧的中点为该弧上最小间隙最大点；弧越宽中点越优
      const candidates = common.map((arc) => {
        const endLifted = arc.wraps ? arc.endDeg + FULL : arc.endDeg;
        const deg = mod((arc.startDeg + endLifted) / 2, FULL);
        const gaps = gapsAt(deg);
        return { deg, minGap: minGapOfGaps(gaps), gaps, width: arc.widthDeg };
      });
      candidates.sort((x, y) => {
        if (x.minGap === null) return y.minGap === null ? x.deg - y.deg : -1;
        if (y.minGap === null) return 1;
        if (Math.abs(x.minGap - y.minGap) > EPS) return y.minGap - x.minGap;
        return x.deg - y.deg;
      });
      const win = candidates[0];
      best = {
        deg: win.deg,
        minGapDeg: win.minGap,
        minGapSpecIndex: win.minGap === null ? null : minGapSpecOfGaps(win.gaps, win.minGap),
        gaps: win.gaps,
      };
    }
  }

  return {
    mode: 'multi',
    seamWidth,
    specs,
    common,
    fullCircle,
    best,
    fallback,
    noSolution: blocker !== null,
    blocker,
    detents,
  };
}

module.exports = {
  EPS,
  MIN_SPECS,
  MAX_SPECS,
  mod,
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
  computeDetentFallback,
  buildFallback,
  computeMultiSpec,
};
