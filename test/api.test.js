'use strict';

const test = require('node:test');
const assert = require('node:assert/strict');
const http = require('node:http');

const { app } = require('../server');

let server;
let baseUrl;

test.before(async () => {
  await new Promise((resolve) => {
    server = app.listen(0, '127.0.0.1', resolve);
  });
  const addr = server.address();
  baseUrl = `http://127.0.0.1:${addr.port}`;
});

test.after(async () => {
  await new Promise((resolve) => server.close(resolve));
});

async function postJson(p, body) {
  const res = await fetch(baseUrl + p, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body),
  });
  const json = await res.json();
  return { status: res.status, json };
}

const baseSingle = { circumference: 400, seamWidth: 20, zones: [{ start: 60, end: 120 }] };
const baseMulti = {
  circumferences: [400, 200],
  seamWidth: 20,
  zones: [{ start: 60, end: 120 }],
};

test('GET /api/health 正常', async () => {
  const res = await fetch(baseUrl + '/api/health');
  assert.equal(res.status, 200);
  assert.deepEqual(await res.json(), { ok: true });
});

test('单规格：缺省/禁用档位返回禁用结构且结果不变', async () => {
  const a = await postJson('/api/compute', baseSingle);
  assert.equal(a.status, 200);
  assert.deepEqual(a.json.detents, { enabled: false });

  const b = await postJson('/api/compute', { ...baseSingle, detents: { enabled: false, steps: 3 } });
  assert.equal(b.status, 200);
  assert.equal(b.json.detents.enabled, false);
  assert.equal(b.json.best.mm, a.json.best.mm);
});

test('单规格：合法档位参数返回档位结果', async () => {
  const r = await postJson('/api/compute', {
    ...baseSingle,
    detents: { enabled: true, steps: 12, offsetDeg: 0 },
  });
  assert.equal(r.status, 200);
  assert.equal(r.json.detents.enabled, true);
  assert.equal(r.json.detents.steps, 12);
  assert.ok(Array.isArray(r.json.detents.available));
  assert.ok(typeof r.json.best.detentIndex === 'number');
});

test('单规格：steps 越界/非整数/非数字 → 400', async () => {
  for (const steps of [1, 361, 12.5, '12', NaN]) {
    const r = await postJson('/api/compute', {
      ...baseSingle,
      detents: { enabled: true, steps },
    });
    assert.equal(r.status, 400, `steps=${steps} 应 400`);
    assert.match(r.json.error, /steps/);
  }
});

test('单规格：offsetDeg 非数值 → 400；detents 非对象 → 400', async () => {
  const r1 = await postJson('/api/compute', {
    ...baseSingle,
    detents: { enabled: true, steps: 12, offsetDeg: 'x' },
  });
  assert.equal(r1.status, 400);
  assert.match(r1.json.error, /offsetDeg/);

  for (const bad of [null, [], 'on', 42]) {
    const r = await postJson('/api/compute', { ...baseSingle, detents: bad });
    assert.equal(r.status, 400, `detents=${JSON.stringify(bad)} 应 400`);
  }
});

test('单规格：偏移超出 ±360° 自动按整圈归一，不报错', async () => {
  const r = await postJson('/api/compute', {
    ...baseSingle,
    detents: { enabled: true, steps: 12, offsetDeg: 375 },
  });
  assert.equal(r.status, 200);
  assert.equal(r.json.detents.offsetDeg, 15);
});

test('多规格：合法档位返回逐规格间隙；非法参数 → 400', async () => {
  const ok = await postJson('/api/compute-multi', {
    ...baseMulti,
    detents: { enabled: true, steps: 8 },
  });
  assert.equal(ok.status, 200);
  assert.equal(ok.json.detents.enabled, true);
  for (const d of ok.json.detents.available) {
    assert.equal(d.gaps.length, 2);
  }

  const bad = await postJson('/api/compute-multi', {
    ...baseMulti,
    detents: { enabled: true, steps: 0 },
  });
  assert.equal(bad.status, 400);
  assert.match(bad.json.error, /steps/);

  const badType = await postJson('/api/compute-multi', {
    ...baseMulti,
    detents: 'yes',
  });
  assert.equal(badType.status, 400);
});

test('多规格：连续公共段有解但无可用档位时 noAvailable=true 且 best=null', async () => {
  const r = await postJson('/api/compute-multi', {
    circumferences: [360, 360],
    seamWidth: 0,
    zones: [
      { start: 0, end: 30 },
      { start: 40, end: 360 },
    ],
    detents: { enabled: true, steps: 12, offsetDeg: 15 },
  });
  assert.equal(r.status, 200);
  assert.equal(r.json.noSolution, false);
  assert.equal(r.json.best, null);
  assert.equal(r.json.detents.noAvailable, true);
  assert.equal(r.json.detents.nearest.length, 1);
});

test('多规格：无共同解时返回最小违约备选 fallback；有共同解时为 null', async () => {
  const noCommon = await postJson('/api/compute-multi', {
    circumferences: [360, 180],
    seamWidth: 0,
    zones: [
      { start: 90, end: 180 },
      { start: 180, end: 360 },
    ],
  });
  assert.equal(noCommon.status, 200);
  assert.equal(noCommon.json.noSolution, true);
  assert.equal(noCommon.json.best, null);
  assert.ok(noCommon.json.fallback);
  const f = noCommon.json.fallback;
  assert.equal(f.deg, 0);
  assert.equal(f.maxIntrusionDeg, 0);
  assert.equal(typeof f.sumIntrusionDeg, 'number');
  assert.equal(f.restrictedToDetents, false);
  assert.equal(f.detentIndex, null);
  assert.equal(Array.isArray(f.intrusions), true);
  assert.equal(f.intrusions.length, 2);
  for (const item of f.intrusions) {
    assert.equal(typeof item.index, 'number');
    assert.equal(typeof item.circumference, 'number');
    assert.equal(typeof item.intrusionDeg, 'number');
    assert.equal(typeof item.mm, 'number');
  }
  assert.match(f.reason, /公共可行角度为空/);
  assert.ok(noCommon.json.blocker);

  const ok = await postJson('/api/compute-multi', baseMulti);
  assert.equal(ok.status, 200);
  assert.equal(ok.json.noSolution, false);
  assert.equal(ok.json.fallback, null);
});

test('多规格：启用档位且无共同解时 fallback 只取档位角度并带 detentIndex', async () => {
  const r = await postJson('/api/compute-multi', {
    circumferences: [134, 592],
    seamWidth: 39,
    zones: [
      { start: 171, end: 308 },
      { start: 263, end: 457 },
    ],
    detents: { enabled: true, steps: 4 },
  });
  assert.equal(r.status, 200);
  assert.equal(r.json.noSolution, true);
  assert.equal(r.json.fallback.restrictedToDetents, true);
  assert.equal(r.json.fallback.deg, 270);
  assert.equal(r.json.fallback.detentIndex, 3);
  // 备选角度必须是档位之一
  assert.ok(r.json.detents.positions.includes(r.json.fallback.deg));
  assert.ok(Math.abs(r.json.fallback.maxIntrusionDeg - 28.208955223880565) < 1e-9);
});
