/*
 * Load Path – bridge-building physics game
 * Build mode, test mode (static truss analysis per frame) and collapse mode (Verlet).
 * World units: metres, kN. World y points up; screen y points down.
 */
(() => {
  'use strict';

  const TS = window.TrussSolver;

  // ------------------------------------------------------------------
  // CONFIG: every tunable engineering and scoring constant, in one place.
  // Units: kN and m, so stresses are kN/m² (1 MPa = 1000 kN/m²).
  // tools/generate-solutions.js reads this object too, so par prices always
  // follow these numbers. Rerun it after changing anything here.
  // ------------------------------------------------------------------
  const CONFIG = {
    materials: {
      // E: Young's modulus (kN/m²)   A: cross-section area (m²)
      // I: second moment of area in the bridge's plane (m⁴)
      // Z: plastic section modulus (m³), bending capacity M_p = fy·Z (road only)
      // fy: yield strength (kN/m²)    cost: $ per metre
      // weight: self-weight in kN/m
      //
      // Road: an IPE 240 steel deck beam (A 39.1 cm², I 3890 cm⁴, Wpl 367 cm³) in
      // S275 steel, bending about its strong axis in the bridge's plane (the deck
      // slab braces it sideways). weight = beam 0.31 kN/m + deck slab.
      // φ·M_p = 0.9 × 275 MPa × 367 cm³ = 90.8 kN·m, so with the 80 kN truck a
      // road segment of 4 m passes the bending check and about 4.5 m fails.
      road:  { name: 'Road',  E: 200e6, A: 39.1e-4, I: 3890e-8, Z: 367e-6, fy: 275e3, cost: 150, weight: 2.0 },
      // Steel: a square hollow section about 100×100×2.5 mm (A 10 cm², I 158 cm⁴,
      // r ≈ 40 mm). Its Euler load π²EI/L² is half its yield load at L = 5 m.
      steel: { name: 'Steel', E: 200e6, A: 1.0e-3, I: 1.583e-6, fy: 250e3, cost: 100, weight: 0.6 }
    },
    // Compression with buckling: 'aisc' = AISC 360 §E3 column curve (real members
    // are slightly crooked and hold residual stresses, so they fail below the ideal
    // Euler load); 'euler' = ideal Euler load capped at yield (less realistic).
    bucklingCurve: 'aisc',
    // Load factors for the strength checks: dead · self-weight + live · impact · truck.
    // 1.0 everywhere reproduces the classic game. Bridge codes (AASHTO LRFD) use
    // dead 1.25, live 1.75 and impact 1.33; deflection always uses the unfactored
    // self-weight plus the truck × impact.
    loadFactors: { dead: 1.0, live: 1.0 },
    impactFactor: 1.0,
    phi: 0.9,                  // safety (resistance) factor applied to every capacity
    deflectionLimitRatio: 400, // deck nodes may deflect at most span / 400
    jointCost: 100,            // $ per joint you place (anchors are free), about 1 m of steel
    loadStep: 0.5,             // m between truck positions in the moving-load check
    maxMemberLength: 4,        // m
    parStars: [1.0, 1.25],     // cost ≤ 1.0×par → 3 stars, ≤ 1.25×par → 2 stars, any pass → 1
    colorBands: [0.6, 0.9]     // utilization colours: green < 0.6 ≤ yellow < 0.9 ≤ orange ≤ 1 < red
  };

  const MATERIALS = {};
  for (const [id, m] of Object.entries(CONFIG.materials)) MATERIALS[id] = Object.assign({ id }, m);
  const MAX_LEN = CONFIG.maxMemberLength;
  const DEFLECTION_SCALE = 25;  // visual exaggeration of displacements
  const TRUCK_SPEED = 3;        // m/s
  const TRUCK_START = -3.5;     // m (on the left cliff)
  const SOLVE_STEP = 0.04;      // m of truck travel per animation analysis step
  const GRAVITY = 9.81;
  const COLLAPSE_TIME = 3.6;    // s of collapse animation before the result
  const COLLAPSE_STIFFNESS = 0.3; // constraint relaxation during collapse
  const TEAR_STRAIN = 0.025;    // members tear past 2.5% strain while collapsing
  const STORE_KEY = 'loadpath.bridge.v1';

  const COLORS = {
    accent: '#8ed8ff',
    orange: '#ffb86b',
    danger: '#ff6b6b',
    roadFill: '#4a5675',
    roadEdge: '#9fb0cc',
    joint: '#0f1424',
    rock: '#18213a',
    rockEdge: 'rgba(142,216,255,0.45)'
  };

  // ------------------------------------------------------------------
  // Levels: deck anchors at (0,0) and (gap,0) are added automatically.
  // rules: which checks apply. Strength (with φ), the moving load and joint
  // cost always apply; buckling and deflection are switched on per level.
  // ------------------------------------------------------------------
  const LEVELS = [
    {
      id: 'l1', name: 'First Crossing', gap: 8, budget: 3500,
      truck: { name: 'Van', weight: 30, scale: 0.85 },
      anchors: [[0, -2], [8, -2]], piers: [],
      rules: { buckling: false, deflection: false },
      tip: 'Start with the road deck, then brace it with triangles.'
    },
    {
      id: 'l2', name: 'Mind the Gap', gap: 12, budget: 6000,
      truck: { name: 'Pickup', weight: 40, scale: 0.95 },
      anchors: [[0, -2], [12, -2]], piers: [],
      rules: { buckling: false, deflection: false },
      tip: 'Every joint costs money too. Fewer, longer members can be cheaper.'
    },
    {
      id: 'l3', name: 'Long Span', gap: 16, budget: 8500,
      truck: { name: 'Delivery truck', weight: 50, scale: 1 },
      anchors: [[0, -2], [16, -2]], piers: [],
      rules: { buckling: true, deflection: false },
      tip: 'Long members in compression now buckle. Their strength falls fast with length.'
    },
    {
      id: 'l4', name: 'Heavy Haul', gap: 16, budget: 8400,
      truck: { name: 'Heavy truck', weight: 75, scale: 1.15 },
      anchors: [[0, -2], [16, -2]], piers: [],
      rules: { buckling: true, deflection: false },
      tip: 'Same gap, one and a half times the load.'
    },
    {
      id: 'l5', name: 'Midway Pier', gap: 20, budget: 10000,
      truck: { name: 'Delivery truck', weight: 60, scale: 1.05 },
      anchors: [[0, -2], [20, -2]], piers: [{ x: 10, top: -2 }],
      rules: { buckling: true, deflection: true },
      tip: 'The deck may now sag at most span/400. Stiffness matters, not just strength.'
    },
    {
      id: 'l6', name: 'The Big One', gap: 24, budget: 13000,
      truck: { name: 'Heavy truck', weight: 80, scale: 1.15 },
      anchors: [[0, -3], [24, -3]], piers: [{ x: 12, top: -2 }],
      rules: { buckling: true, deflection: true },
      tip: 'Brace the pier well: everything comes down through it.'
    }
  ];
  const WATER_Y = -7;

  // <generated-solutions>
  // Written by tools/generate-solutions.js. Do not edit by hand: rerun it.
  // Cheapest passing design found per level (best found, not proven optimal).
  // Par = the cost of this design. Each entry: [x1, y1, x2, y2, material]
  const SOLUTIONS = {
    l1: [[0,0,4,0,'road'],[4,0,8,0,'road'],[4,0,7,-1,'steel'],[7,-1,8,0,'steel'],[8,-2,7,-1,'steel']],
    l2: [[0,0,4,0,'road'],[4,0,8,0,'road'],[8,0,12,0,'road'],[0,0,1,-1,'steel'],[1,-1,4,0,'steel'],[8,0,11,-1,'steel'],[11,-1,12,0,'steel'],[0,-2,1,-1,'steel'],[12,-2,11,-1,'steel']],
    l3: [[0,0,3,0,'road'],[3,0,6,0,'road'],[6,0,10,0,'road'],[10,0,13,0,'road'],[13,0,16,0,'road'],[3,0,3,-2,'steel'],[13,0,13,-2,'steel'],[3,-2,6,0,'steel'],[10,0,13,-2,'steel'],[0,-2,3,0,'steel'],[0,-2,3,-2,'steel'],[16,-2,13,0,'steel'],[16,-2,13,-2,'steel']],
    l4: [[0,0,4,0,'road'],[4,0,8,0,'road'],[8,0,12,0,'road'],[12,0,16,0,'road'],[0,0,2,-2,'steel'],[2,-2,4,0,'steel'],[4,0,7,-1,'steel'],[7,-1,8,0,'steel'],[8,0,9,-1,'steel'],[9,-1,12,0,'steel'],[12,0,14,-2,'steel'],[14,-2,16,0,'steel'],[7,-1,9,-1,'steel'],[0,-2,2,-2,'steel'],[16,-2,14,-2,'steel']],
    l5: [[0,0,3,0,'road'],[3,0,7,0,'road'],[7,0,10,0,'road'],[10,0,13,0,'road'],[13,0,17,0,'road'],[17,0,20,0,'road'],[0,-2,3,0,'steel'],[20,-2,17,0,'steel'],[10,-2,7,0,'steel'],[10,-2,10,0,'steel'],[10,-2,13,0,'steel']],
    l6: [[0,0,4,0,'road'],[4,0,8,0,'road'],[8,0,12,0,'road'],[0,0,2,-2,'steel'],[2,-2,4,0,'steel'],[8,0,10,-2,'steel'],[10,-2,12,0,'steel'],[12,0,16,0,'road'],[16,0,20,0,'road'],[20,0,24,0,'road'],[12,0,14,-2,'steel'],[14,-2,16,0,'steel'],[20,0,22,-2,'steel'],[22,-2,24,0,'steel'],[0,-3,2,-2,'steel'],[24,-3,22,-2,'steel'],[12,-2,12,0,'steel'],[12,-2,10,-2,'steel'],[12,-2,14,-2,'steel']]
  };
  // </generated-solutions>
  const PIER_HALF = 0.6;

  // ------------------------------------------------------------------
  // DOM
  // ------------------------------------------------------------------
  const $ = (id) => document.getElementById(id);
  const canvas = $('game');
  const ctx = canvas.getContext('2d');
  const stage = $('stage');
  const tooltip = $('tooltip');
  const toastEl = $('toast');
  const legend = $('legend');
  const btnTest = $('btnTest');
  const btnUndo = $('btnUndo');
  const btnClear = $('btnClear');
  const btnSolution = $('btnSolution');
  const btnTC = $('btnTC');
  const toolButtons = Array.from(document.querySelectorAll('.tool[data-tool]'));

  // ------------------------------------------------------------------
  // Persistence (localStorage may be unavailable: always guarded)
  // ------------------------------------------------------------------
  function loadStore() {
    try {
      const raw = localStorage.getItem(STORE_KEY);
      const data = raw ? JSON.parse(raw) : null;
      if (data && typeof data === 'object') {
        data.progress = data.progress || {};
        data.designs = data.designs || {};
        return data;
      }
    } catch (e) { /* ignore */ }
    return { progress: {}, designs: {}, seenHelp: false, lastLevel: 0 };
  }
  function saveStore() {
    try { localStorage.setItem(STORE_KEY, JSON.stringify(store)); } catch (e) { /* ignore */ }
  }
  const store = loadStore();

  // ------------------------------------------------------------------
  // State
  // ------------------------------------------------------------------
  let levelIndex = 0;
  let level = LEVELS[0];
  let joints = [];   // {id, x, y, anchor, deck}
  let members = [];  // {id, a, b, mat}
  let nextId = 1;
  let history = [];
  let mode = 'build'; // build | test | collapse | result
  let tool = 'road';
  let showTC = true;
  let drag = null;    // {from: joint, to: {x,y}, valid, reason, cost}
  let hover = null;   // {type:'member', index} in current render list
  let pointer = null; // last pointer screen pos {x,y,type}
  let sim = null;     // test / collapse simulation state
  let mech = null;    // mechanism visualisation {mode: Map(jointId -> [dx,dy]), t}
  let flashDeck = 0;  // timer to highlight road members when the deck is broken
  let time = 0;

  const view = { scale: 30, ox: 0, oy: 0, w: 0, h: 0, dpr: 1 };

  // ------------------------------------------------------------------
  // Geometry helpers
  // ------------------------------------------------------------------
  const dist = (ax, ay, bx, by) => Math.hypot(bx - ax, by - ay);
  const jointById = (id) => joints.find((j) => j.id === id);
  const jointAt = (x, y) => joints.find((j) => j.x === x && j.y === y);
  const memberLength = (m) => { const a = jointById(m.a), b = jointById(m.b); return dist(a.x, a.y, b.x, b.y); };
  const memberCost = (m) => memberLength(m) * MATERIALS[m.mat].cost;
  // Every non-anchor joint costs CONFIG.jointCost (orphan joints are always pruned)
  const jointCount = () => joints.filter((j) => !j.anchor).length;
  const totalCost = () => members.reduce((s, m) => s + memberCost(m), 0) + jointCount() * CONFIG.jointCost;
  const fmtMoney = (v) => '$' + Math.round(v).toLocaleString('en-US');
  const fmtMm = (m) => (m * 1000).toFixed(1) + ' mm';
  const fmtPct = (u) => Math.round(u * 100) + '%';

  // ---- Level rules -------------------------------------------------
  /** Longest clear span between deck supports (cliff edges and piers), m. */
  function deflectionSpan(L) {
    const xs = [0, ...L.piers.map((p) => p.x), L.gap].sort((a, b) => a - b);
    let span = 0;
    for (let i = 1; i < xs.length; i++) span = Math.max(span, xs[i] - xs[i - 1]);
    return span;
  }
  function levelRules(L) {
    return {
      buckling: !!L.rules.buckling,
      deflection: !!L.rules.deflection,
      phi: CONFIG.phi,
      loadStep: CONFIG.loadStep,
      deflectionLimit: deflectionSpan(L) / CONFIG.deflectionLimitRatio,
      deckMomentCapacity: MATERIALS.road.fy * MATERIALS.road.Z, // M_p = fy·Z
      bucklingCurve: CONFIG.bucklingCurve,
      deadFactor: CONFIG.loadFactors.dead,
      liveFactor: CONFIG.loadFactors.live,
      impactFactor: CONFIG.impactFactor
    };
  }
  const capFor = (matId, len) => TS.memberCapacity(MATERIALS[matId], len, levelRules(level));
  /** Bending utilization of a road segment of length L with this level's truck at midspan. */
  const deckBendingRatio = (L) => {
    const road = MATERIALS.road;
    const M = CONFIG.loadFactors.live * CONFIG.impactFactor * level.truck.weight * L / 4 +
      CONFIG.loadFactors.dead * road.weight * L * L / 8;
    return M / (CONFIG.phi * road.fy * road.Z);
  };
  function ruleNames(L) {
    return ['strength'].concat(L.rules.buckling ? ['buckling'] : [], L.rules.deflection ? ['deflection'] : []);
  }
  const capitalise = (w) => w[0].toUpperCase() + w.slice(1);
  function anchorPoints(L) {
    return [[0, 0, true], [L.gap, 0, true], ...L.anchors.map(([x, y]) => [x, y, false]), ...L.piers.map((p) => [p.x, p.top, false])];
  }

  /**
   * Road path the truck drives: from the left deck anchor to the right one,
   * always moving right, preferring the straightest road.
   * points: [{x,y}], edges: [{a,b,mat}] (indices into points)
   * Returns [{edge, from, to}] or null.
   */
  function deckPathOf(points, edges, start, end) {
    const visited = new Set();
    const path = [];
    function dfs(j) {
      if (j === end) return true;
      visited.add(j);
      const options = [];
      edges.forEach((e, i) => {
        if (e.mat !== 'road') return;
        const other = e.a === j ? e.b : e.b === j ? e.a : -1;
        if (other >= 0 && points[other].x > points[j].x && !visited.has(other)) options.push({ i, other });
      });
      options.sort((p, q) => Math.abs(points[p.other].y) - Math.abs(points[q.other].y) || points[q.other].x - points[p.other].x);
      for (const o of options) {
        path.push({ edge: o.i, from: j, to: o.other });
        if (dfs(o.other)) return true;
        path.pop();
      }
      return false;
    }
    return start >= 0 && end >= 0 && dfs(start) ? path : null;
  }

  /** Cost of a design list [[x1,y1,x2,y2,mat], ...] for a level (members + joints). */
  function designCost(L, list) {
    const anchors = new Set(anchorPoints(L).map(([x, y]) => x + ',' + y));
    const js = new Set();
    let c = 0;
    for (const [x1, y1, x2, y2, mat] of list) {
      c += dist(x1, y1, x2, y2) * MATERIALS[mat].cost;
      for (const k of [x1 + ',' + y1, x2 + ',' + y2]) if (!anchors.has(k)) js.add(k);
    }
    return c + js.size * CONFIG.jointCost;
  }

  /** Full evaluation of a design list (used to verify the stored par designs). */
  function evaluateDesign(L, list) {
    const pts = [], idx = new Map();
    const add = (x, y, fixed) => {
      const k = x + ',' + y;
      if (!idx.has(k)) { idx.set(k, pts.length); pts.push({ x, y, fixX: fixed, fixY: fixed }); }
      return idx.get(k);
    };
    for (const [x, y] of anchorPoints(L)) add(x, y, true);
    const edges = list.map(([x1, y1, x2, y2, mat]) => ({ a: add(x1, y1, false), b: add(x2, y2, false), mat }));
    const path = deckPathOf(pts, edges, idx.get('0,0'), idx.get(L.gap + ',0'));
    if (!path) return { ok: false, passed: false };
    return TS.evaluateBridge({
      nodes: pts,
      members: edges.map((e) => ({ a: e.a, b: e.b, material: MATERIALS[e.mat] })),
      deck: path.map((p) => ({ a: p.from, b: p.to, member: p.edge })),
      truckWeight: L.truck.weight,
      rules: levelRules(L)
    });
  }

  const parCache = new Map();
  /** Par price: the cost of the stored cheapest passing design. */
  function parFor(L) {
    if (!parCache.has(L.id)) parCache.set(L.id, SOLUTIONS[L.id] ? designCost(L, SOLUTIONS[L.id]) : L.budget);
    return parCache.get(L.id);
  }
  function starsFor(L, cost) {
    const par = parFor(L);
    if (cost <= par * CONFIG.parStars[0] + 0.01) return 3;
    if (cost <= par * CONFIG.parStars[1] + 0.01) return 2;
    return 1;
  }

  function pointSegDist(px, py, ax, ay, bx, by) {
    const dx = bx - ax, dy = by - ay;
    const l2 = dx * dx + dy * dy;
    let t = l2 ? ((px - ax) * dx + (py - ay) * dy) / l2 : 0;
    t = Math.max(0, Math.min(1, t));
    return dist(px, py, ax + t * dx, ay + t * dy);
  }

  /** True if (x,y) is inside solid ground (cliff or pier), boundary included. */
  function inRock(x, y, eps = 0) {
    if (x <= 0 - eps && y <= 0 - eps) return true;
    if (x >= level.gap + eps && y <= 0 - eps) return true;
    for (const p of level.piers) {
      if (Math.abs(x - p.x) <= PIER_HALF - eps && y <= p.top - eps) return true;
    }
    return false;
  }
  const isRockPoint = (x, y) => inRock(x, y, 0);
  const strictlyInRock = (x, y) => inRock(x, y, 0.02);

  const buildBounds = () => ({ minX: -3, maxX: level.gap + 3, minY: WATER_Y + 1, maxY: 6 });

  // ------------------------------------------------------------------
  // View / coordinates
  // ------------------------------------------------------------------
  function resize() {
    const r = stage.getBoundingClientRect();
    view.dpr = Math.min(window.devicePixelRatio || 1, 2);
    view.w = Math.max(1, r.width);
    view.h = Math.max(1, r.height);
    canvas.width = Math.round(view.w * view.dpr);
    canvas.height = Math.round(view.h * view.dpr);
    fitView();
  }

  function fitView() {
    const minX = TRUCK_START - 2.2, maxX = level.gap - TRUCK_START + 2.2;
    const minY = WATER_Y - 1, maxY = 6.5;
    const pad = 12;
    const sx = (view.w - pad * 2) / (maxX - minX);
    const sy = (view.h - pad * 2) / (maxY - minY);
    view.scale = Math.max(4, Math.min(sx, sy));
    view.ox = view.w / 2 - ((minX + maxX) / 2) * view.scale;
    view.oy = view.h / 2 + ((minY + maxY) / 2) * view.scale;
  }

  const sx = (x) => view.ox + x * view.scale;
  const sy = (y) => view.oy - y * view.scale;
  const toWorld = (px, py) => ({ x: (px - view.ox) / view.scale, y: (view.oy - py) / view.scale });

  // ------------------------------------------------------------------
  // Level loading / design (de)serialisation
  // ------------------------------------------------------------------
  function loadLevel(i) {
    levelIndex = Math.max(0, Math.min(LEVELS.length - 1, i));
    level = LEVELS[levelIndex];
    store.lastLevel = levelIndex;
    saveStore();

    joints = [];
    members = [];
    nextId = 1;
    for (const [x, y, deck] of anchorPoints(level)) addJoint(x, y, true, deck);

    const saved = store.designs[level.id];
    if (Array.isArray(saved)) applyDesign(saved);

    history = [];
    sim = null; mech = null; drag = null; hover = null;
    setMode('build');
    fitView();
    updateHud();
    hideTooltip();
    const prev = levelIndex > 0 ? ruleNames(LEVELS[levelIndex - 1]) : [];
    const added = ruleNames(level).filter((r) => prev.indexOf(r) < 0);
    const rulesText = levelIndex === 0
      ? 'Rules: strength'
      : added.length ? `New rule: ${added.join(', ')}` : `Rules: ${ruleNames(level).join(', ')}`;
    toast(`Level ${levelIndex + 1}: ${level.name}. ${rulesText}. ${level.tip}`, 'info', 6000);
  }

  /** Replace the current members with a design given as [x1, y1, x2, y2, mat] rows. */
  function applyDesign(list) {
    members = [];
    pruneJoints();
    for (const s of list) {
      if (!Array.isArray(s) || s.length !== 5 || !MATERIALS[s[4]]) continue;
      const [x1, y1, x2, y2, mat] = s;
      if (dist(x1, y1, x2, y2) > MAX_LEN + 1e-9 || (x1 === x2 && y1 === y2)) continue;
      const a = jointAt(x1, y1) || addJoint(x1, y1, false, false);
      const b = jointAt(x2, y2) || addJoint(x2, y2, false, false);
      if (!members.some((m) => sameMember(m, a.id, b.id))) members.push({ id: nextId++, a: a.id, b: b.id, mat });
    }
    if (totalCost() > level.budget + 1e-6) { members = []; pruneJoints(); }
  }

  function showSolution() {
    if (mode !== 'build') return;
    const sol = SOLUTIONS[level.id];
    if (!sol) return;
    pushHistory();
    applyDesign(sol);
    afterEdit();
    const cost = totalCost();
    toast(`Par design loaded: ${fmtMoney(cost)}. Undo brings back your own design.`, 'info', 5500);
  }

  function addJoint(x, y, anchor, deck) {
    const j = { id: nextId++, x, y, anchor, deck };
    joints.push(j);
    return j;
  }
  const sameMember = (m, a, b) => (m.a === a && m.b === b) || (m.a === b && m.b === a);

  function pruneJoints() {
    const used = new Set();
    for (const m of members) { used.add(m.a); used.add(m.b); }
    joints = joints.filter((j) => j.anchor || used.has(j.id));
  }

  function serialiseDesign() {
    return members.map((m) => {
      const a = jointById(m.a), b = jointById(m.b);
      return [a.x, a.y, b.x, b.y, m.mat];
    });
  }
  function saveDesign() {
    store.designs[level.id] = serialiseDesign();
    saveStore();
  }

  function pushHistory() {
    history.push(JSON.stringify({ joints, members, nextId }));
    if (history.length > 200) history.shift();
  }
  function undo() {
    if (mode !== 'build' || !history.length) return;
    const s = JSON.parse(history.pop());
    joints = s.joints; members = s.members; nextId = s.nextId;
    mech = null;
    afterEdit();
  }
  function afterEdit() {
    mech = null;
    saveDesign();
    updateHud();
  }

  // ------------------------------------------------------------------
  // Building
  // ------------------------------------------------------------------
  /** End point for a drag: the snapped grid point, or the closest valid-length grid point. */
  function dragEndPoint(from, w) {
    let gx = Math.round(w.x), gy = Math.round(w.y);
    if (dist(from.x, from.y, gx, gy) <= MAX_LEN + 1e-9) return { x: gx, y: gy };
    let best = null, bestD = Infinity;
    for (let dx = -MAX_LEN; dx <= MAX_LEN; dx++) {
      for (let dy = -MAX_LEN; dy <= MAX_LEN; dy++) {
        if (!dx && !dy) continue;
        if (Math.hypot(dx, dy) > MAX_LEN + 1e-9) continue;
        const px = from.x + dx, py = from.y + dy;
        const d = dist(px, py, w.x, w.y);
        if (d < bestD) { bestD = d; best = { x: px, y: py }; }
      }
    }
    return best;
  }

  function validateMember(from, to, mat) {
    const len = dist(from.x, from.y, to.x, to.y);
    if (len === 0) return { ok: false, reason: '' };
    const target = jointAt(to.x, to.y);
    const newJoint = !target;
    const cost = len * MATERIALS[mat].cost + (newJoint ? CONFIG.jointCost : 0);
    const res = (ok, reason) => ({ ok, reason, cost, len, newJoint });
    if (len > MAX_LEN + 1e-9) return res(false, `Members can be at most ${MAX_LEN} m long`);

    const b = buildBounds();
    if (!target) {
      if (to.x < b.minX || to.x > b.maxX || to.y < b.minY || to.y > b.maxY) return res(false, 'Out of the build area');
      if (isRockPoint(to.x, to.y)) return res(false, "Can't put a joint inside the ground");
      for (const m of members) {
        const p = jointById(m.a), q = jointById(m.b);
        if (pointSegDist(to.x, to.y, p.x, p.y, q.x, q.y) < 1e-6) return res(false, 'That point lies on an existing member');
      }
    } else if (members.some((m) => sameMember(m, from.id, target.id))) {
      return res(false, 'Those joints are already connected');
    }

    // Must not pass through another joint (it would not be connected there)
    for (const j of joints) {
      if (j === from || j === target) continue;
      if (pointSegDist(j.x, j.y, from.x, from.y, to.x, to.y) < 1e-6) return res(false, "Members can't pass through a joint. Connect to it instead");
    }
    // Must not run through solid ground
    const n = Math.ceil(len / 0.1);
    for (let i = 1; i < n; i++) {
      const t = i / n;
      if (strictlyInRock(from.x + (to.x - from.x) * t, from.y + (to.y - from.y) * t)) return res(false, "Members can't go through the ground");
    }
    if (totalCost() + cost > level.budget + 1e-6) return res(false, `Over budget by ${fmtMoney(totalCost() + cost - level.budget)}`);
    return res(true, '');
  }

  function commitDrag() {
    if (!drag || !drag.to) return;
    const v = validateMember(drag.from, drag.to, tool);
    if (!v.ok) { if (v.reason) toast(v.reason, 'error'); return; }
    pushHistory();
    const b = jointAt(drag.to.x, drag.to.y) || addJoint(drag.to.x, drag.to.y, false, false);
    members.push({ id: nextId++, a: drag.from.id, b: b.id, mat: tool });
    afterEdit();
  }

  function memberNear(w, tolM) {
    let best = -1, bestD = tolM;
    members.forEach((m, i) => {
      const a = jointById(m.a), b = jointById(m.b);
      const d = pointSegDist(w.x, w.y, a.x, a.y, b.x, b.y);
      if (d < bestD) { bestD = d; best = i; }
    });
    return best;
  }
  function jointNear(w, tolM) {
    let best = null, bestD = tolM;
    for (const j of joints) {
      const d = dist(w.x, w.y, j.x, j.y);
      if (d < bestD) { bestD = d; best = j; }
    }
    return best;
  }

  function deleteAt(w, tolM) {
    const j = jointNear(w, tolM * 0.8);
    if (j && !j.anchor) {
      pushHistory();
      members = members.filter((m) => m.a !== j.id && m.b !== j.id);
      pruneJoints();
      afterEdit();
      return;
    }
    const i = memberNear(w, tolM);
    if (i >= 0) {
      pushHistory();
      members.splice(i, 1);
      pruneJoints();
      afterEdit();
    }
  }

  function clearAll() {
    if (mode !== 'build' || !members.length) return;
    pushHistory();
    members = [];
    pruneJoints();
    afterEdit();
    toast('Cleared. Press Undo to bring it back.', 'info');
  }

  // ------------------------------------------------------------------
  // Analysis model
  // ------------------------------------------------------------------
  function buildModel() {
    const used = new Set();
    for (const m of members) { used.add(m.a); used.add(m.b); }
    const nodeJoints = joints.filter((j) => j.anchor || used.has(j.id));
    const index = new Map(nodeJoints.map((j, i) => [j.id, i]));
    const nodes = nodeJoints.map((j) => ({ x: j.x, y: j.y, fixX: j.anchor, fixY: j.anchor }));
    const mems = members.map((m) => ({ a: index.get(m.a), b: index.get(m.b), EA: MATERIALS[m.mat].E * MATERIALS[m.mat].A, mat: m.mat }));
    return { nodeJoints, index, nodes, mems };
  }

  /** Road path from the left deck anchor to the right one, always moving right. */
  function findDeckPath() {
    const index = new Map(joints.map((j, i) => [j.id, i]));
    const edges = members.map((m) => ({ a: index.get(m.a), b: index.get(m.b), mat: m.mat }));
    const start = joints.findIndex((j) => j.deck && j.x === 0);
    const end = joints.findIndex((j) => j.deck && j.x === level.gap);
    const path = deckPathOf(joints, edges, start, end);
    return path && path.map((p) => ({ member: p.edge, from: joints[p.from].id, to: joints[p.to].id }));
  }

  function startTest() {
    if (mode !== 'build') return;
    hideTooltip();
    if (!members.some((m) => m.mat === 'road')) {
      toast('Build a road deck from the left cliff to the right cliff first.', 'error');
      return;
    }
    const path = findDeckPath();
    if (!path) {
      flashDeck = 2.5;
      toast('The road has a gap. Road members must connect both cliffs, left to right.', 'error', 4500);
      return;
    }
    const model = buildModel();
    // Moving-load check over the whole span, before anything is animated
    const env = TS.evaluateBridge({
      nodes: model.nodes,
      members: model.mems.map((m) => ({ a: m.a, b: m.b, material: MATERIALS[m.mat] })),
      deck: path.map((p) => ({ a: model.index.get(p.from), b: model.index.get(p.to), member: p.member })),
      truckWeight: level.truck.weight,
      rules: levelRules(level)
    });
    if (!env.ok) { showMechanism(env.mechanism, model); return; }

    // The first failure along the drive. x === null means before the truck is
    // on the bridge (self-weight alone). Ties: strength, then bending, then deflection.
    const events = [];
    if (env.strengthFail) events.push({ kind: 'strength', x: env.strengthFail.x, member: env.strengthFail.member });
    if (env.bendingFail) events.push({ kind: 'bending', x: env.bendingFail.x, member: env.bendingFail.member });
    if (env.deflFail) events.push({ kind: 'deflection', x: env.deflFail.x });
    const ex = (e) => (e.x === null ? -Infinity : e.x);
    const order = { strength: 0, bending: 1, deflection: 2 };
    events.sort((p, q) => ex(p) - ex(q) || order[p.kind] - order[q.kind]);

    const n = model.nodes.length;
    sim = {
      model, env,
      solver: env.solver, caps: env.caps, baseLoads: env.baseLoads, segs: env.segs,
      truckX: TRUCK_START,
      W: level.truck.weight,
      disp: new Float64Array(2 * n),
      forces: new Float64Array(model.mems.length),
      utils: new Float64Array(model.mems.length),
      bend: new Float64Array(model.mems.length),
      maxUtil: 0,
      defl: 0,
      maxDefl: 0,
      event: events[0] || null
    };
    setMode('test');
    analyze(sim.truckX);
    saveDesign();
    if (sim.event && sim.event.x === null) triggerEvent();
  }

  function segmentAt(x) {
    return sim ? sim.env.segmentAt(x) : null;
  }

  /** Live analysis for the animation (colours, deflected shape, HUD). */
  function analyze(x) {
    const r = sim.solver.solve(sim.env.loadsAt(x));
    const sameLoads = sim.env.deadFactor === 1 && sim.env.liveFactor === sim.env.impactFactor;
    sim.disp = sameLoads ? r.displacements : sim.solver.solve(sim.env.loadsAt(x, true)).displacements;
    sim.forces = r.forces;
    for (let i = 0; i < r.forces.length; i++) {
      const u = TS.utilization(r.forces[i], sim.caps[i]);
      sim.utils[i] = u;
      if (u > sim.maxUtil) sim.maxUtil = u;
    }
    // Live deck bending: point load P at distance a from one end gives M = P·a·(L−a)/L,
    // which peaks at P·L/4 at the midpoint (the value the check uses)
    sim.bend.fill(0);
    const on = segmentAt(x);
    if (on) {
      const mi = on.seg.member;
      const L = sim.solver.lengths[mi];
      const a = on.t * L;
      const w = MATERIALS.road.weight;
      // Moment under the truck: P·a·(L−a)/L + w·a·(L−a)/2, both factored
      const M = sim.env.liveFactor * sim.W * a * (L - a) / L + sim.env.deadFactor * w * a * (L - a) / 2;
      sim.bend[mi] = TS.beamColumnRatio(sim.utils[mi], M / sim.env.momentCapacity);
      if (sim.bend[on.seg.member] > sim.maxUtil) sim.maxUtil = sim.bend[on.seg.member];
    }
    let d = 0;
    for (const k of sim.env.deckNodes) d = Math.max(d, Math.abs(r.displacements[2 * k + 1]));
    sim.defl = d;
    sim.maxDefl = Math.max(sim.maxDefl, d);
  }

  /** The truck has reached the first failing position found by the moving-load check. */
  function triggerEvent() {
    const ev = sim.event;
    if (ev.x !== null) sim.truckX = ev.x;
    analyze(ev.x === null ? null : ev.x);
    if (ev.kind === 'strength') startCollapse(ev.member, 'axial');
    else if (ev.kind === 'bending') startCollapse(ev.member, 'bending');
    else tooFlexible();
  }

  function updateTest(dt) {
    const target = sim.truckX + TRUCK_SPEED * dt;
    while (sim.truckX < target) {
      const next = Math.min(target, sim.truckX + SOLVE_STEP);
      if (sim.event && sim.event.x !== null && next >= sim.event.x) { triggerEvent(); return; }
      sim.truckX = next;
      analyze(sim.truckX);
    }
    if (sim.truckX >= level.gap - TRUCK_START) win();
  }

  // ------------------------------------------------------------------
  // Mechanism (unstable structure) feedback
  // ------------------------------------------------------------------
  function showMechanism(result, model) {
    const modeMap = new Map();
    let max = 0;
    for (let i = 0; i < model.nodes.length; i++) {
      const dx = result.mode[2 * i] || 0, dy = result.mode[2 * i + 1] || 0;
      max = Math.max(max, Math.hypot(dx, dy));
    }
    if (!(max > 0) || !isFinite(max)) max = 1;
    model.nodeJoints.forEach((j, i) => {
      const dx = result.mode[2 * i] / max, dy = result.mode[2 * i + 1] / max;
      if (isFinite(dx) && isFinite(dy)) modeMap.set(j.id, [dx, dy]);
    });
    mech = { mode: modeMap, t: 0 };
    toast("This bridge isn't stable. Try adding triangles. The wobbling joints are free to move.", 'error', 5500);
  }

  // ------------------------------------------------------------------
  // Collapse: Verlet integration with distance constraints
  // ------------------------------------------------------------------
  function startCollapse(brokenIndex, cause) {
    const { model, disp } = sim;
    const particles = model.nodes.map((nd, i) => {
      const x = nd.x + disp[2 * i] * DEFLECTION_SCALE;
      const y = nd.y + disp[2 * i + 1] * DEFLECTION_SCALE;
      return { x, y, px: x, py: y, fixed: nd.fixX, mass: 0.2 };
    });
    model.mems.forEach((m, i) => {
      const w = MATERIALS[m.mat].weight * sim.solver.lengths[i];
      particles[m.a].mass += w / 2;
      particles[m.b].mass += w / 2;
    });
    particles.forEach((p) => { p.baseMass = p.mass; });

    const constraints = model.mems.map((m, i) => {
      const a = particles[m.a], b = particles[m.b];
      return { a: m.a, b: m.b, rest: dist(a.x, a.y, b.x, b.y), mat: m.mat, member: i, broken: false, debris: false };
    });

    const m = model.mems[brokenIndex];
    const f = sim.forces[brokenIndex];
    sim.failure = {
      mat: m.mat,
      length: sim.solver.lengths[brokenIndex],
      force: f,
      util: sim.utils[brokenIndex],
      capacity: f >= 0 ? sim.caps[brokenIndex].tension : sim.caps[brokenIndex].compression,
      buckling: f < 0 && sim.caps[brokenIndex].bucklingGoverns,
      cause: cause || 'axial'
    };
    if (cause === 'bending') {
      // The segment snaps as a beam: show it red and report the moment check
      sim.failure.util = sim.env.bendingRatio[brokenIndex];
      sim.failure.moment = sim.env.bendingMoment[brokenIndex];
      sim.failure.momentCapacity = sim.env.momentCapacity;
      sim.failure.axialRatio = sim.env.deckAxialRatio[brokenIndex];
      sim.failure.momentRatio = sim.env.deckMomentRatio[brokenIndex];
      sim.utils[brokenIndex] = Math.max(sim.utils[brokenIndex], sim.failure.util);
    }
    sim.maxUtil = Math.max(sim.maxUtil, sim.failure.util);

    // Truck: attached to the deck segment it is on, or parked on a cliff
    const on = segmentAt(sim.truckX);
    const truck = { attached: null, free: null, parked: !on, v: TRUCK_SPEED };
    if (on) truck.attached = { segIndex: on.index, t: on.t };

    sim.collapse = {
      particles, constraints, truck,
      t: 0, acc: 0,
      cascade: [],          // [{member, at}]
      frozenUtils: Float64Array.from(sim.utils),
      frozenForces: Float64Array.from(sim.forces)
    };
    // The failure releases energy: kick the broken member's joints and the
    // joints under the truck downward so a locked straight deck snaps through.
    const kick = (i, v) => { const p = particles[i]; if (!p.fixed) p.py = p.y + v / 120; };
    kick(m.a, 2.5); kick(m.b, 2.5);
    if (on) { kick(on.seg.a, 3); kick(on.seg.b, 3); }
    breakMember(brokenIndex);
    setMode('collapse');
  }

  function breakMember(memberIndex) {
    const C = sim.collapse;
    const c = C.constraints.find((k) => k.member === memberIndex && !k.debris);
    if (!c || c.broken) return;
    c.broken = true;
    const P = C.particles;
    const a = P[c.a], b = P[c.b];
    const mx = (a.x + b.x) / 2, my = (a.y + b.y) / 2;
    const half = c.rest / 2;
    const w = MATERIALS[c.mat].weight * c.rest / 4 + 0.1;
    // Each half stays pinned to its joint and dangles from the break point
    const kick = 0.015;
    const pa = { x: mx, y: my, px: mx + (Math.random() - 0.5) * kick, py: my + (Math.random() - 0.5) * kick, fixed: false, mass: w, baseMass: w, debris: true };
    const pb = { x: mx, y: my, px: mx + (Math.random() - 0.5) * kick, py: my + (Math.random() - 0.5) * kick, fixed: false, mass: w, baseMass: w, debris: true };
    P.push(pa, pb);
    C.constraints.push(
      { a: c.a, b: P.length - 2, rest: half, mat: c.mat, member: c.member, broken: false, debris: true },
      { a: P.length - 1, b: c.b, rest: half, mat: c.mat, member: c.member, broken: false, debris: true }
    );
    sim.collapse.lastBreak = { x: mx, y: my, t: C.t };
    scheduleCascade();
  }

  /** Re-solve the damaged structure statically; overloaded members fail in turn. */
  function scheduleCascade() {
    const C = sim.collapse;
    if (C.t > 1.2) return;
    const brokenSet = new Set(C.constraints.filter((k) => k.broken && !k.debris).map((k) => k.member));
    const queued = new Set(C.cascade.map((q) => q.member));
    const keep = [];
    sim.model.mems.forEach((m, i) => { if (!brokenSet.has(i)) keep.push(i); });
    const mems = keep.map((i) => sim.model.mems[i]);
    const solver = mems.length ? TS.createSolver(sim.model.nodes, mems) : { ok: false };
    const tr = C.truck.attached;
    if (!solver.ok) {
      // A mechanism can't carry the truck: the deck under it tears and the
      // Verlet simulation takes over from here.
      if (tr) {
        const deckMember = sim.segs[tr.segIndex].member;
        if (!brokenSet.has(deckMember) && !queued.has(deckMember)) C.cascade.push({ member: deckMember, at: C.t + 0.3 });
      }
      return;
    }
    const F = Float64Array.from(sim.baseLoads);
    if (tr) {
      const s = sim.segs[tr.segIndex];
      F[2 * s.a + 1] -= sim.W * (1 - tr.t);
      F[2 * s.b + 1] -= sim.W * tr.t;
    }
    const r = solver.solve(F);
    const over = [];
    keep.forEach((mi, k) => {
      const cap = capFor(sim.model.mems[mi].mat, solver.lengths[k]);
      const u = TS.utilization(r.forces[k], cap);
      C.frozenUtils[mi] = u;
      C.frozenForces[mi] = r.forces[k];
      if (u > 1 && !queued.has(mi)) over.push({ mi, u });
    });
    over.sort((p, q) => q.u - p.u);
    if (over.length) C.cascade.push({ member: over[0].mi, at: C.t + 0.12 });
  }

  function updateCollapse(dt) {
    const C = sim.collapse;
    const h = 1 / 120;
    C.acc += Math.min(dt, 1 / 20);
    while (C.acc >= h) {
      C.acc -= h;
      C.t += h;
      verletStep(h);
    }
    for (let i = C.cascade.length - 1; i >= 0; i--) {
      if (C.t >= C.cascade[i].at) {
        const q = C.cascade.splice(i, 1)[0];
        breakMember(q.member);
      }
    }
    if (C.t >= COLLAPSE_TIME && mode === 'collapse') fail();
  }

  function verletStep(h) {
    const C = sim.collapse;
    const P = C.particles;
    const g = GRAVITY * h * h;
    const tr = C.truck;

    // Truck weight rides on the particles of its deck segment
    for (const p of P) p.mass = p.baseMass;
    if (tr.attached) {
      const s = sim.segs[tr.attached.segIndex];
      const truckMass = sim.W;
      P[s.a].mass += truckMass * (1 - tr.attached.t);
      P[s.b].mass += truckMass * tr.attached.t;
    }

    for (const p of P) {
      if (p.fixed) continue;
      let vx = (p.x - p.px) * 0.995;
      let vy = (p.y - p.py) * 0.995;
      if (p.y < WATER_Y) { vx *= 0.85; vy *= 0.85; }
      p.px = p.x; p.py = p.y;
      p.x += vx;
      p.y += vy - (p.y < WATER_Y ? g * 0.15 : g);
    }

    // Members are slightly soft once the bridge has failed, so a hanging deck
    // stretches under the truck and tears instead of locking rigid.
    for (let it = 0; it < 10; it++) {
      for (const c of C.constraints) {
        if (c.broken) continue;
        const a = P[c.a], b = P[c.b];
        const wa = a.fixed ? 0 : 1 / a.mass;
        const wb = b.fixed ? 0 : 1 / b.mass;
        const ws = wa + wb;
        if (!ws) continue;
        const dx = b.x - a.x, dy = b.y - a.y;
        const d = Math.hypot(dx, dy) || 1e-9;
        const diff = ((d - c.rest) / d / ws) * (c.debris ? 1 : COLLAPSE_STIFFNESS);
        a.x += dx * diff * wa; a.y += dy * diff * wa;
        b.x -= dx * diff * wb; b.y -= dy * diff * wb;
      }
      for (const p of P) if (!p.fixed) collide(p);
    }

    // Tear members that are stretched or crushed too far for too long
    for (const c of C.constraints) {
      if (c.broken || c.debris) continue;
      const a = P[c.a], b = P[c.b];
      const strain = Math.abs(Math.hypot(b.x - a.x, b.y - a.y) - c.rest) / c.rest;
      c.over = strain > TEAR_STRAIN ? (c.over || 0) + h : 0;
      if (c.over > 0.04) breakMember(c.member);
    }

    // Truck
    if (tr.attached) {
      const seg = sim.segs[tr.attached.segIndex];
      const c = C.constraints.find((k) => k.member === seg.member && !k.debris);
      const a = P[seg.a], b = P[seg.b];
      const ang = Math.atan2(b.y - a.y, b.x - a.x);
      if (c.broken || Math.abs(ang) > 1.1) {
        detachTruck(h);
      } else {
        // Brakes on, but gravity pulls it along a sloping deck
        tr.v += -GRAVITY * Math.sin(ang) * h;
        tr.v *= 0.985;
        const L = Math.hypot(b.x - a.x, b.y - a.y) || 1;
        tr.attached.t += (tr.v * h) / L;
        if (tr.attached.t > 1 || tr.attached.t < 0) {
          const dir = tr.attached.t > 1 ? 1 : -1;
          const ni = tr.attached.segIndex + dir;
          const ns = sim.segs[ni];
          const nc = ns && C.constraints.find((k) => k.member === ns.member && !k.debris);
          if (ns && nc && !nc.broken) {
            tr.attached.segIndex = ni;
            tr.attached.t = dir > 0 ? 0 : 1;
          } else if (!ns) {
            tr.attached.t = Math.max(0, Math.min(1, tr.attached.t));
            tr.v = 0;
          } else {
            detachTruck(h);
          }
        }
      }
    } else if (tr.free) {
      const f = tr.free;
      f.vy -= GRAVITY * h;
      if (f.y < WATER_Y) { f.vx *= 0.96; f.vy *= 0.9; f.av *= 0.95; f.vy -= -GRAVITY * 0.8 * h; }
      f.x += f.vx * h; f.y += f.vy * h; f.angle += f.av * h;
      if (f.y < WATER_Y - 3.2) { f.y = WATER_Y - 3.2; f.vy = 0; }
      const half = 1.2 * level.truck.scale;
      if (f.x - half < 0 && f.y < 0 && f.x > -3) { f.x = half; f.vx = Math.abs(f.vx) * 0.3; }
      if (f.x + half > level.gap && f.y < 0 && f.x < level.gap + 3) { f.x = level.gap - half; f.vx = -Math.abs(f.vx) * 0.3; }
    }
  }

  function detachTruck(h) {
    const C = sim.collapse;
    const tr = C.truck;
    const pose = truckPose();
    const seg = sim.segs[tr.attached.segIndex];
    const P = C.particles;
    const a = P[seg.a], b = P[seg.b];
    const t = tr.attached.t;
    const vx = ((a.x - a.px) * (1 - t) + (b.x - b.px) * t) / h;
    const vy = ((a.y - a.py) * (1 - t) + (b.y - b.py) * t) / h;
    tr.free = {
      x: pose.x, y: pose.y, angle: pose.angle,
      vx: vx + Math.cos(pose.angle) * tr.v * 0.6,
      vy: vy,
      av: (Math.random() - 0.5) * 1.6 + (pose.angle > 0 ? 0.6 : -0.6)
    };
    tr.attached = null;
  }

  function collide(p) {
    if (p.y < WATER_Y - 3) { p.y = WATER_Y - 3; p.py = p.y; p.px = p.x; }
    if (!strictlyInRock(p.x, p.y)) return;
    // Push out of the nearest rock face
    const cands = [];
    if (p.x <= 0 && p.y <= 0) cands.push({ x: 0.001, y: p.y, d: -p.x }, { x: p.x, y: 0.001, d: -p.y });
    if (p.x >= level.gap && p.y <= 0) cands.push({ x: level.gap - 0.001, y: p.y, d: p.x - level.gap }, { x: p.x, y: 0.001, d: -p.y });
    for (const pr of level.piers) {
      if (Math.abs(p.x - pr.x) <= PIER_HALF && p.y <= pr.top) {
        cands.push({ x: pr.x - PIER_HALF - 0.001, y: p.y, d: p.x - (pr.x - PIER_HALF) });
        cands.push({ x: pr.x + PIER_HALF + 0.001, y: p.y, d: pr.x + PIER_HALF - p.x });
        cands.push({ x: p.x, y: pr.top + 0.001, d: pr.top - p.y });
      }
    }
    if (!cands.length) return;
    cands.sort((u, v) => u.d - v.d);
    p.x = cands[0].x; p.y = cands[0].y;
    p.px = p.x + (p.px - p.x) * 0.3; // friction
    p.py = p.y;
  }

  // ------------------------------------------------------------------
  // Win / fail
  // ------------------------------------------------------------------
  const inspectAction = { label: 'Inspect', cls: 'chip', fn: () => closeModal('resultModal') };

  function win() {
    setMode('result');
    const env = sim.env;
    const cost = totalCost();
    const par = parFor(level);
    const stars = starsFor(level, cost);
    const prev = store.progress[level.id];
    const isBest = !prev || cost < prev.bestCost;
    store.progress[level.id] = {
      stars: Math.max(stars, prev ? prev.stars : 0),
      bestCost: prev ? Math.min(prev.bestCost, cost) : cost
    };
    saveDesign();
    saveStore();

    const stats = [
      ['Cost', `${fmtMoney(cost)} / ${fmtMoney(level.budget)}`],
      ['Par', fmtMoney(par)],
      ['Max utilization', fmtPct(env.maxRatio)]
    ];
    if (level.rules.deflection) stats.push(['Max deflection', `${fmtMm(env.maxDefl)} / ${fmtMm(env.deflectionLimit)}`]);
    stats.push(['Best cost', fmtMoney(store.progress[level.id].bestCost) + (isBest ? ' (new)' : '')]);

    const hasNext = levelIndex < LEVELS.length - 1;
    showResult({
      title: 'Bridge complete!',
      stars,
      text: stars === 3
        ? (cost < par - 0.5 ? 'Under par! You beat the best design we found.' : 'Right on par. Lean and strong: that is real engineering.')
        : `It held! Get it to ${fmtMoney(stars === 2 ? par * CONFIG.parStars[0] : par * CONFIG.parStars[1])} or less for another star.`,
      stats,
      actions: [
        inspectAction,
        { label: 'Improve design', cls: 'chip', fn: backToBuild },
        hasNext
          ? { label: 'Next level →', cls: 'btn-primary', fn: () => { closeModal('resultModal'); loadLevel(levelIndex + 1); } }
          : { label: 'All levels', cls: 'btn-primary', fn: () => { closeModal('resultModal'); openLevels(); } }
      ]
    });
  }

  function fail() {
    setMode('result');
    const f = sim.failure;
    const kind = f.force >= 0 ? 'tension' : 'compression';
    const what = `${f.length.toFixed(1)} m ${MATERIALS[f.mat].name.toLowerCase()} member`;
    const where = sim.event && sim.event.x !== null ? ` with the truck at x = ${sim.event.x.toFixed(1)} m` : ' under its own weight';
    const capNote = `(φ = ${CONFIG.phi} applied)`;
    let text;
    let title = 'Bridge collapsed';
    if (f.cause === 'bending') {
      title = 'Deck bending';
      text = `A ${f.length.toFixed(1)} m road segment failed as a beam with the truck at its midpoint (x = ${sim.event.x.toFixed(1)} m). ` +
        `Bending M = P·L/4 + w·L²/8 = ${f.moment.toFixed(1)} kN·m uses ${fmtPct(f.momentRatio)} of φ·M = ${f.momentCapacity.toFixed(1)} kN·m, ` +
        `and its axial force uses ${fmtPct(f.axialRatio)} of its capacity. Combined (AISC H1-1) that is ${fmtPct(f.util)}. ` +
        'Add a deck joint to shorten the segment, or take axial load out of the deck.';
    } else if (f.buckling) {
      text = `A ${what} buckled${where}: ${Math.abs(f.force).toFixed(1)} kN of compression against a buckling capacity of ${f.capacity.toFixed(1)} kN ${capNote}. ` +
        'Compression strength drops fast with length (ideally with 1/L², so half the length is up to four times as strong). Shorten it or brace it with an extra joint.';
    } else if (kind === 'tension') {
      text = `A ${what} snapped in tension${where}: ${f.force.toFixed(1)} kN against a capacity of ${f.capacity.toFixed(1)} kN ${capNote}. ` +
        'A deeper truss or more load paths will share the load.';
    } else {
      text = `A ${what} was crushed${where}: ${Math.abs(f.force).toFixed(1)} kN of compression against a capacity of ${f.capacity.toFixed(1)} kN ${capNote}. ` +
        'A deeper truss reduces chord forces.';
    }
    showResult({
      title,
      stars: 0,
      text,
      stats: [
        ['Cost', `${fmtMoney(totalCost())} / ${fmtMoney(level.budget)}`],
        ['Truck', `${level.truck.name}, ${level.truck.weight} kN`],
        ['Failed at', `${fmtPct(f.util)} utilization`]
      ],
      actions: [
        inspectAction,
        { label: 'Levels', cls: 'chip', fn: () => { closeModal('resultModal'); backToBuild(); openLevels(); } },
        { label: 'Fix the bridge', cls: 'btn-primary', fn: backToBuild }
      ]
    });
  }

  /** Serviceability failure: nothing broke, but the deck sagged past span/limit. */
  function tooFlexible() {
    const env = sim.env;
    const d = env.deflFail;
    sim.flex = { node: d.node, value: d.value };
    setMode('result');
    const nd = sim.model.nodes[d.node];
    const where = d.x === null ? 'under its own weight' : `with the truck at x = ${d.x.toFixed(1)} m`;
    showResult({
      title: 'Too flexible',
      stars: 0,
      text: `Nothing broke, but the deck at x = ${nd.x} m sagged ${fmtMm(d.value)} ${where}. ` +
        `The limit is span/${CONFIG.deflectionLimitRatio} = ${fmtMm(env.deflectionLimit)} for a ${deflectionSpan(level)} m span. ` +
        'A deeper truss, more diagonals or a support closer to the sag will stiffen it.',
      stats: [
        ['Cost', `${fmtMoney(totalCost())} / ${fmtMoney(level.budget)}`],
        ['Deflection', `${fmtMm(d.value)} / ${fmtMm(env.deflectionLimit)}`],
        ['Max utilization', fmtPct(env.maxRatio)]
      ],
      actions: [
        inspectAction,
        { label: 'Fix the bridge', cls: 'btn-primary', fn: backToBuild }
      ],
      delay: 1100
    });
  }

  function backToBuild() {
    closeModal('resultModal');
    sim = null;
    setMode('build');
    hideTooltip();
  }

  // ------------------------------------------------------------------
  // Mode / HUD
  // ------------------------------------------------------------------
  function setMode(m) {
    mode = m;
    const building = m === 'build';
    toolButtons.forEach((b) => { b.disabled = !building; });
    btnUndo.disabled = !building;
    btnClear.disabled = !building;
    btnSolution.disabled = !building;
    btnTest.textContent = building ? 'Test bridge' : 'Back to build';
    btnTest.classList.toggle('stop', !building);
    legend.hidden = building;
    drag = null;
    updateHud();
  }

  function updateHud() {
    const cost = totalCost();
    $('levelNumber').textContent = `Level ${levelIndex + 1} of ${LEVELS.length}`;
    $('levelName').textContent = level.name;
    $('costValue').textContent = fmtMoney(cost);
    $('budgetValue').textContent = fmtMoney(level.budget);
    const par = parFor(level);
    $('parValue').textContent = `· Par ${fmtMoney(par)}`;
    const r = cost / level.budget;
    const fill = $('budgetFill');
    fill.style.width = Math.min(100, r * 100) + '%';
    fill.classList.toggle('warn', cost > par * CONFIG.parStars[1] + 0.01);
    fill.classList.toggle('over', r > 1);
    $('parTick').style.left = Math.min(100, (par / level.budget) * 100) + '%';
    const note = legend.querySelector('.legend-note');
    if (mode === 'build') {
      $('statLabel').textContent = level.truck.name;
      $('statValue').textContent = `${level.truck.weight} kN`;
    } else if (sim) {
      const showEnv = mode === 'result' && !sim.collapse;
      $('statLabel').textContent = showEnv ? 'Worst utilization' : 'Peak utilization';
      $('statValue').textContent = fmtPct(Math.min(showEnv ? sim.env.maxRatio : sim.maxUtil, 9.99));
      const deflText = level.rules.deflection
        ? `Deck deflection ${fmtMm(showEnv ? sim.env.maxDefl : sim.defl)} / ${fmtMm(sim.env.deflectionLimit)} · `
        : '';
      note.textContent = `${deflText}${showEnv ? 'Colours: worst case over all truck positions' : 'Deflection drawn ×' + DEFLECTION_SCALE}`;
    }
    btnUndo.disabled = mode !== 'build' || !history.length;
    btnClear.disabled = mode !== 'build' || !members.length;
  }

  // ------------------------------------------------------------------
  // Rendering
  // ------------------------------------------------------------------
  const BAND_COLORS = { low: '#5fd38d', mid: '#ffd84d', high: '#ff9f43', fail: '#ff4a4a' };
  /** Utilization (worst force ÷ capacity) → colour band. */
  function utilColor(u) {
    if (u > 1) return BAND_COLORS.fail;
    if (u >= CONFIG.colorBands[1]) return BAND_COLORS.high;
    if (u >= CONFIG.colorBands[0]) return BAND_COLORS.mid;
    return BAND_COLORS.low;
  }

  /** Current display positions for joints, by joint id. */
  function displayPositions() {
    const pos = new Map();
    if (sim && sim.collapse) {
      sim.model.nodeJoints.forEach((j, i) => pos.set(j.id, sim.collapse.particles[i]));
    } else if (sim) {
      sim.model.nodeJoints.forEach((j, i) => pos.set(j.id, {
        x: j.x + sim.disp[2 * i] * DEFLECTION_SCALE,
        y: j.y + sim.disp[2 * i + 1] * DEFLECTION_SCALE
      }));
    } else if (mech) {
      const amp = 0.35 * Math.sin(mech.t * 7) * Math.min(1, mech.t * 3);
      for (const j of joints) {
        const d = mech.mode.get(j.id);
        pos.set(j.id, d ? { x: j.x + d[0] * amp, y: j.y + d[1] * amp } : j);
      }
    } else {
      for (const j of joints) pos.set(j.id, j);
    }
    return pos;
  }

  function render() {
    const c = ctx;
    c.setTransform(view.dpr, 0, 0, view.dpr, 0, 0);
    c.clearRect(0, 0, view.w, view.h);
    drawBackground(c);
    drawTerrain(c);

    const pos = displayPositions();
    const lists = renderList(pos);
    for (const item of lists) drawMemberItem(c, item);
    if (hover && lists[hover.index]) drawHover(c, lists[hover.index]);
    drawJoints(c, pos);
    if (sim && sim.flex && !sim.collapse) drawFlexMarker(c, pos);
    drawTruck(c);
    if (drag) drawDrag(c);
    drawWaterFront(c);
    if (mode === 'build' && !members.length && !drag) drawHint(c);
  }

  function drawBackground(c) {
    const w0 = toWorld(0, view.h), w1 = toWorld(view.w, 0);
    c.lineWidth = 1;
    for (let x = Math.floor(w0.x); x <= Math.ceil(w1.x); x++) {
      c.strokeStyle = x % 4 === 0 ? 'rgba(142,216,255,0.10)' : 'rgba(142,216,255,0.045)';
      c.beginPath(); c.moveTo(Math.round(sx(x)) + 0.5, 0); c.lineTo(Math.round(sx(x)) + 0.5, view.h); c.stroke();
    }
    for (let y = Math.floor(w0.y); y <= Math.ceil(w1.y); y++) {
      c.strokeStyle = y % 4 === 0 ? 'rgba(142,216,255,0.10)' : 'rgba(142,216,255,0.045)';
      c.beginPath(); c.moveTo(0, Math.round(sy(y)) + 0.5); c.lineTo(view.w, Math.round(sy(y)) + 0.5); c.stroke();
    }
    // Water body (behind terrain)
    const wy = sy(WATER_Y);
    const grad = c.createLinearGradient(0, wy, 0, view.h);
    grad.addColorStop(0, 'rgba(70,140,220,0.22)');
    grad.addColorStop(1, 'rgba(30,60,120,0.35)');
    c.fillStyle = grad;
    c.fillRect(0, wy, view.w, view.h - wy);
  }

  function rockRects() {
    const bottom = toWorld(0, view.h).y - 1;
    const left = toWorld(0, 0).x - 1, right = toWorld(view.w, 0).x + 1;
    const rects = [
      { x0: left, x1: 0, top: 0, bottom },
      { x0: level.gap, x1: right, top: 0, bottom }
    ];
    for (const p of level.piers) rects.push({ x0: p.x - PIER_HALF, x1: p.x + PIER_HALF, top: p.top, bottom });
    return rects;
  }

  function drawTerrain(c) {
    for (const r of rockRects()) {
      const X0 = sx(r.x0), X1 = sx(r.x1), Y0 = sy(r.top), Y1 = sy(r.bottom);
      c.fillStyle = COLORS.rock;
      c.fillRect(X0, Y0, X1 - X0, Y1 - Y0);
      // Section hatching, blueprint style
      c.save();
      c.beginPath(); c.rect(X0, Y0, X1 - X0, Y1 - Y0); c.clip();
      c.strokeStyle = 'rgba(142,216,255,0.07)';
      c.lineWidth = 1;
      const step = 10;
      for (let k = X0 - (Y1 - Y0); k < X1; k += step) {
        c.beginPath(); c.moveTo(k, Y1); c.lineTo(k + (Y1 - Y0), Y0); c.stroke();
      }
      c.restore();
      c.strokeStyle = COLORS.rockEdge;
      c.lineWidth = 1.5;
      c.beginPath();
      c.moveTo(X0, Y1); c.lineTo(X0, Y0); c.lineTo(X1, Y0); c.lineTo(X1, Y1);
      c.stroke();
    }
    // Road surface on the cliff tops
    c.fillStyle = COLORS.roadFill;
    const rh = Math.max(3, 0.18 * view.scale);
    c.fillRect(sx(TRUCK_START - 3), sy(0), sx(0) - sx(TRUCK_START - 3), rh);
    c.fillRect(sx(level.gap), sy(0), sx(level.gap - TRUCK_START + 3) - sx(level.gap), rh);
  }

  function drawWaterFront(c) {
    const wy = sy(WATER_Y);
    c.strokeStyle = 'rgba(142,216,255,0.5)';
    c.lineWidth = 1.5;
    c.beginPath();
    for (let x = 0; x <= view.w; x += 6) {
      const y = wy + Math.sin(x * 0.04 + time * 1.6) * 2 + Math.sin(x * 0.013 - time) * 1.5;
      if (x === 0) c.moveTo(x, y); else c.lineTo(x, y);
    }
    c.stroke();
  }

  /** Flat list of drawable members with screen geometry. */
  function renderList(pos) {
    const out = [];
    if (sim && sim.collapse) {
      const C = sim.collapse;
      for (const k of C.constraints) {
        if (k.broken) continue;
        const a = C.particles[k.a], b = C.particles[k.b];
        out.push({
          ax: a.x, ay: a.y, bx: b.x, by: b.y, mat: k.mat,
          util: k.debris ? 1.2 : C.frozenUtils[k.member],
          force: k.debris ? 0 : C.frozenForces[k.member],
          member: k.debris ? -1 : k.member,
          debris: k.debris
        });
      }
      return out;
    }
    members.forEach((m, i) => {
      const a = pos.get(m.a), b = pos.get(m.b);
      const item = { ax: a.x, ay: a.y, bx: b.x, by: b.y, mat: m.mat, member: i };
      if (sim && mode === 'result') {
        // After a test: worst force over every truck position ÷ capacity
        const e = sim.env;
        item.util = e.ratio[i];
        item.force = e.governs[i] === 'C' ? -e.worstC[i] : e.worstT[i];
        item.envelope = true;
      } else if (sim) {
        item.util = Math.max(sim.utils[i], sim.bend[i]);
        item.force = sim.forces[i];
        item.bend = sim.bend[i];
      }
      out.push(item);
    });
    return out;
  }

  function memberWidth(mat) {
    return mat === 'road' ? Math.max(5, 0.3 * view.scale) : Math.max(3, 0.13 * view.scale);
  }

  function drawMemberItem(c, it) {
    const x1 = sx(it.ax), y1 = sy(it.ay), x2 = sx(it.bx), y2 = sy(it.by);
    const w = memberWidth(it.mat);
    c.lineCap = 'round';
    const analysed = it.util !== undefined;

    if (analysed) {
      const compression = it.force < 0;
      if (showTC && compression && !it.debris) {
        c.setLineDash([6, 5]);
        c.strokeStyle = 'rgba(255,255,255,0.9)';
        c.lineWidth = w + 4;
        c.lineCap = 'butt';
        line(c, x1, y1, x2, y2);
        c.setLineDash([]);
        c.lineCap = 'round';
        c.strokeStyle = COLORS.joint;
        c.lineWidth = w + 1.5;
        line(c, x1, y1, x2, y2);
      }
      c.strokeStyle = utilColor(it.util);
      c.lineWidth = w;
      line(c, x1, y1, x2, y2);
      if (it.mat === 'road') {
        c.strokeStyle = 'rgba(15,20,36,0.55)';
        c.lineWidth = Math.max(1, w * 0.15);
        c.setLineDash([w * 0.8, w * 0.8]);
        line(c, x1, y1, x2, y2);
        c.setLineDash([]);
      }
      return;
    }

    if (it.mat === 'road') {
      const flash = flashDeck > 0 && Math.sin(time * 14) > 0;
      c.strokeStyle = flash ? COLORS.danger : COLORS.roadEdge;
      c.lineWidth = w + 2;
      line(c, x1, y1, x2, y2);
      c.strokeStyle = COLORS.roadFill;
      c.lineWidth = w;
      line(c, x1, y1, x2, y2);
      c.strokeStyle = COLORS.orange;
      c.lineWidth = Math.max(1, w * 0.12);
      c.setLineDash([w * 0.9, w * 0.9]);
      line(c, x1, y1, x2, y2);
      c.setLineDash([]);
    } else {
      c.strokeStyle = COLORS.accent;
      c.lineWidth = w;
      line(c, x1, y1, x2, y2);
    }
  }

  function line(c, x1, y1, x2, y2) {
    c.beginPath(); c.moveTo(x1, y1); c.lineTo(x2, y2); c.stroke();
  }

  function drawHover(c, it) {
    c.save();
    c.strokeStyle = 'rgba(255,255,255,0.35)';
    c.lineWidth = memberWidth(it.mat) + 8;
    c.lineCap = 'round';
    line(c, sx(it.ax), sy(it.ay), sx(it.bx), sy(it.by));
    c.restore();
    drawMemberItem(c, it);
  }

  function drawJoints(c, pos) {
    const r = Math.max(3.5, 0.12 * view.scale);
    const mechMoving = (id) => mech && mech.mode.has(id) && Math.hypot(...mech.mode.get(id)) > 0.2;
    if (sim && sim.collapse) {
      // only real joints (not debris tips)
      sim.model.nodeJoints.forEach((j, i) => {
        const p = sim.collapse.particles[i];
        drawJoint(c, p.x, p.y, r, j.anchor, j.deck, false);
      });
      return;
    }
    for (const j of joints) {
      const p = pos.get(j.id) || j;
      drawJoint(c, p.x, p.y, r, j.anchor, j.deck, mechMoving(j.id));
    }
    if (drag && mode === 'build') {
      const p = drag.from;
      c.strokeStyle = COLORS.accent;
      c.lineWidth = 2;
      c.beginPath(); c.arc(sx(p.x), sy(p.y), r + 5, 0, Math.PI * 2); c.stroke();
    }
  }

  function drawFlexMarker(c, pos) {
    const j = sim.model.nodeJoints[sim.flex.node];
    const p = pos.get(j.id) || j;
    const X = sx(p.x), Y = sy(p.y);
    const pulse = 0.5 + 0.5 * Math.sin(time * 6);
    c.strokeStyle = BAND_COLORS.fail;
    c.lineWidth = 2.5;
    c.beginPath(); c.arc(X, Y, 10 + pulse * 6, 0, Math.PI * 2); c.stroke();
    drawLabel(c, `${fmtMm(sim.flex.value)} > ${fmtMm(sim.env.deflectionLimit)}`, X, Y + 40, BAND_COLORS.fail);
  }

  function drawJoint(c, x, y, r, anchor, deck, alarm) {
    const X = sx(x), Y = sy(y);
    if (anchor) {
      const s = Math.max(7, 0.32 * view.scale);
      c.fillStyle = COLORS.orange;
      c.beginPath();
      c.moveTo(X, Y);
      c.lineTo(X - s * 0.8, Y + s * 1.1);
      c.lineTo(X + s * 0.8, Y + s * 1.1);
      c.closePath();
      c.globalAlpha = 0.9;
      c.fill();
      c.globalAlpha = 1;
      c.fillStyle = COLORS.joint;
      c.strokeStyle = COLORS.orange;
      c.lineWidth = 2;
      c.beginPath(); c.arc(X, Y, r + (deck ? 1.5 : 0.5), 0, Math.PI * 2); c.fill(); c.stroke();
      return;
    }
    if (alarm) {
      c.fillStyle = 'rgba(255,107,107,0.25)';
      c.beginPath(); c.arc(X, Y, r * 3.2, 0, Math.PI * 2); c.fill();
    }
    c.fillStyle = COLORS.joint;
    c.strokeStyle = alarm ? COLORS.danger : '#dfe9f7';
    c.lineWidth = 2;
    c.beginPath(); c.arc(X, Y, r, 0, Math.PI * 2); c.fill(); c.stroke();
  }

  function drawDrag(c) {
    const { from, to } = drag;
    if (!to) return;
    const v = validateMember(from, to, tool);
    const ok = v.ok;
    const w = memberWidth(tool);
    c.globalAlpha = 0.85;
    drawMemberItem(c, { ax: from.x, ay: from.y, bx: to.x, by: to.y, mat: tool });
    c.globalAlpha = 1;
    if (!ok && v.len) {
      c.strokeStyle = COLORS.danger;
      c.lineWidth = Math.max(2, w * 0.4);
      line(c, sx(from.x), sy(from.y), sx(to.x), sy(to.y));
    }
    const X = sx(to.x), Y = sy(to.y);
    c.strokeStyle = ok ? COLORS.accent : COLORS.danger;
    c.lineWidth = 2;
    c.beginPath(); c.arc(X, Y, Math.max(5, 0.16 * view.scale), 0, Math.PI * 2); c.stroke();

    if (v.len) {
      const cap = capFor(tool, v.len);
      const label = ok
        ? `${v.len.toFixed(2)} m · ${fmtMoney(v.cost)}${v.newJoint ? ' incl. joint' : ''} · comp. ${Math.round(cap.compression)} kN` +
          (tool === 'road' && deckBendingRatio(v.len) > 1 ? ' · deck will bend!' : '')
        : v.reason;
      drawLabel(c, label, X, Y - 22, ok ? COLORS.accent : COLORS.danger);
    }
  }

  function drawLabel(c, text, x, y, color) {
    c.font = '500 12px Inter, system-ui, sans-serif';
    const w = c.measureText(text).width + 14;
    const lx = Math.max(4, Math.min(view.w - w - 4, x - w / 2));
    const ly = Math.max(4, y - 12);
    c.fillStyle = 'rgba(15,20,36,0.9)';
    c.strokeStyle = color;
    c.lineWidth = 1;
    roundRect(c, lx, ly, w, 22, 6);
    c.fill(); c.stroke();
    c.fillStyle = color;
    c.textBaseline = 'middle';
    c.fillText(text, lx + 7, ly + 11);
  }

  function roundRect(c, x, y, w, h, r) {
    c.beginPath();
    c.moveTo(x + r, y);
    c.arcTo(x + w, y, x + w, y + h, r);
    c.arcTo(x + w, y + h, x, y + h, r);
    c.arcTo(x, y + h, x, y, r);
    c.arcTo(x, y, x + w, y, r);
    c.closePath();
  }

  function drawHint(c) {
    const a = joints.find((j) => j.deck && j.x === 0);
    const pulse = 0.5 + 0.5 * Math.sin(time * 4);
    c.strokeStyle = `rgba(255,184,107,${0.3 + 0.5 * pulse})`;
    c.lineWidth = 2;
    c.beginPath(); c.arc(sx(a.x), sy(a.y), 12 + pulse * 8, 0, Math.PI * 2); c.stroke();
    drawLabel(c, 'Drag from an orange anchor to place a road member', sx(a.x) + 150, sy(a.y) - 50, COLORS.orange);
  }

  // ---- Truck ----
  function truckPose() {
    if (sim && sim.collapse) {
      const tr = sim.collapse.truck;
      if (tr.free) return { x: tr.free.x, y: tr.free.y, angle: tr.free.angle, onDeck: true };
      if (tr.attached) {
        const s = sim.segs[tr.attached.segIndex];
        const P = sim.collapse.particles;
        const a = P[s.a], b = P[s.b], t = tr.attached.t;
        return { x: a.x + (b.x - a.x) * t, y: a.y + (b.y - a.y) * t, angle: Math.atan2(b.y - a.y, b.x - a.x), onDeck: true };
      }
    }
    const x = sim ? sim.truckX : TRUCK_START;
    const on = sim && segmentAt(x);
    if (on) {
      const s = on.seg, i = s.a, j = s.b, nd = sim.model.nodes;
      const ax = nd[i].x + sim.disp[2 * i] * DEFLECTION_SCALE, ay = nd[i].y + sim.disp[2 * i + 1] * DEFLECTION_SCALE;
      const bx = nd[j].x + sim.disp[2 * j] * DEFLECTION_SCALE, by = nd[j].y + sim.disp[2 * j + 1] * DEFLECTION_SCALE;
      return { x: ax + (bx - ax) * on.t, y: ay + (by - ay) * on.t, angle: Math.atan2(by - ay, bx - ax), onDeck: true };
    }
    return { x, y: 0, angle: 0 };
  }

  function drawTruck(c) {
    const pose = truckPose();
    const k = level.truck.scale;
    const s = view.scale;
    // Sit on top of the road member (bridge) or the road surface (cliffs)
    const onCliff = !pose.onDeck;
    const lift = onCliff ? 0 : memberWidth('road') / 2;
    c.save();
    c.translate(sx(pose.x), sy(pose.y));
    c.rotate(-pose.angle);
    c.translate(0, -lift);
    c.scale(s * k, s * k);
    // geometry in metres, y down, origin at road contact under the load point
    const wheelR = 0.32;
    // trailer / box
    c.fillStyle = COLORS.orange;
    roundRect(c, -1.55, -1.75, 2.15, 1.25, 0.08);
    c.fill();
    c.fillStyle = 'rgba(15,20,36,0.25)';
    c.fillRect(-1.55, -0.62, 2.15, 0.12);
    // cab
    c.fillStyle = '#ffd3a1';
    c.beginPath();
    c.moveTo(0.7, -0.5); c.lineTo(0.7, -1.35); c.lineTo(1.22, -1.35); c.lineTo(1.55, -0.95); c.lineTo(1.55, -0.5);
    c.closePath(); c.fill();
    c.fillStyle = COLORS.joint;
    c.beginPath();
    c.moveTo(0.82, -0.98); c.lineTo(0.82, -1.25); c.lineTo(1.16, -1.25); c.lineTo(1.4, -0.98);
    c.closePath(); c.fill();
    // chassis
    c.fillStyle = '#2a3350';
    c.fillRect(-1.6, -0.52, 3.2, 0.16);
    // wheels
    for (const wx of [-1.1, -0.55, 1.15]) {
      c.fillStyle = '#10162a';
      c.beginPath(); c.arc(wx, -wheelR, wheelR, 0, Math.PI * 2); c.fill();
      c.fillStyle = '#9fb0cc';
      c.beginPath(); c.arc(wx, -wheelR, wheelR * 0.4, 0, Math.PI * 2); c.fill();
    }
    // load label
    c.fillStyle = '#2a1a06';
    c.font = '600 0.42px "Space Grotesk", system-ui, sans-serif';
    c.textAlign = 'center';
    c.textBaseline = 'middle';
    c.fillText(`${level.truck.weight} kN`, -0.47, -1.13);
    c.textAlign = 'start';
    c.restore();
  }

  // ------------------------------------------------------------------
  // Tooltip
  // ------------------------------------------------------------------
  function hideTooltip() { tooltip.hidden = true; hover = null; }

  function updateHover(px, py, touch) {
    if (drag) { hideTooltip(); return; }
    const pos = displayPositions();
    const list = renderList(pos);
    const tolPx = touch ? 18 : 9;
    let best = -1, bestD = tolPx;
    list.forEach((it, i) => {
      if (it.debris) return;
      const d = pointSegDist(px, py, sx(it.ax), sy(it.ay), sx(it.bx), sy(it.by));
      if (d < bestD) { bestD = d; best = i; }
    });
    if (best < 0) { hideTooltip(); return; }
    hover = { index: best };
    const it = list[best];
    const mi = it.member;
    const m = members[mi];
    if (!m) { hideTooltip(); return; }
    const mat = MATERIALS[m.mat];
    const L = memberLength(m);
    const cap = capFor(m.mat, L);
    const buck = (on) => (on ? ' (buckling)' : '');
    let html;
    if (it.util === undefined) {
      html = `<b>${mat.name}</b> · ${L.toFixed(2)} m · ${fmtMoney(L * mat.cost)}<br>` +
        `Tension capacity ${Math.round(cap.tension)} kN<br>` +
        `Compression capacity ${Math.round(cap.compression)} kN${buck(cap.bucklingGoverns)}` +
        (m.mat === 'road' ? `<br>Bending alone under the ${level.truck.weight} kN truck: ${fmtPct(deckBendingRatio(L))}` +
          (deckBendingRatio(L) > 1 ? ' <b class="t-compression">too long</b>' : '') : '') +
        `<br><span class="muted">φ = ${CONFIG.phi} included</span>`;
    } else if (it.envelope) {
      const e = sim.env;
      const gov = e.governs[mi] === 'C' ? 'compression' : 'tension';
      html = `<b>${mat.name}</b> · ${L.toFixed(2)} m<br>` +
        `<span class="t-tension">Worst tension</span> ${e.worstT[mi].toFixed(1)} / ${Math.round(cap.tension)} kN<br>` +
        `<span class="t-compression">Worst compression</span> ${e.worstC[mi].toFixed(1)} / ${Math.round(cap.compression)} kN${buck(cap.bucklingGoverns)}<br>` +
        (e.bendingMoment[mi] > 0 ? `Bending ${e.bendingMoment[mi].toFixed(1)} / ${e.momentCapacity.toFixed(1)} kN·m · axial + bending ${fmtPct(e.bendingRatio[mi])}<br>` : '') +
        `Utilization <b>${fmtPct(e.ratio[mi])}</b> (${e.bendingRatio[mi] > e.axialRatio[mi] ? 'bending' : gov})`;
    } else {
      const f = it.force;
      const tension = f >= 0;
      const c2 = tension ? cap.tension : cap.compression;
      html = `<b>${mat.name}</b> · ${L.toFixed(2)} m<br>` +
        `<span class="${tension ? 't-tension' : 't-compression'}">${tension ? 'Tension' : 'Compression'}</span> ${Math.abs(f).toFixed(1)} kN<br>` +
        `Utilization <b>${fmtPct(it.util)}</b> of ${Math.round(c2)} kN${buck(!tension && cap.bucklingGoverns)}` +
        (it.bend > 0 ? `<br>Deck bending ${fmtPct(it.bend)}` : '');
    }
    tooltip.innerHTML = html;
    tooltip.hidden = false;
    const tw = tooltip.offsetWidth, th = tooltip.offsetHeight;
    let tx = px + 14, ty = py + 14;
    if (tx + tw > view.w - 8) tx = px - tw - 14;
    if (ty + th > view.h - 8) ty = py - th - 14;
    tooltip.style.left = Math.max(8, tx) + 'px';
    tooltip.style.top = Math.max(8, ty) + 'px';
  }

  // ------------------------------------------------------------------
  // Toast
  // ------------------------------------------------------------------
  let toastTimer = 0;
  function toast(msg, type = 'info', ms = 3200) {
    toastEl.textContent = msg;
    toastEl.className = 'toast ' + type;
    toastEl.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(() => { toastEl.hidden = true; }, ms);
  }

  // ------------------------------------------------------------------
  // Input
  // ------------------------------------------------------------------
  function localPoint(e) {
    const r = canvas.getBoundingClientRect();
    return { x: e.clientX - r.left, y: e.clientY - r.top };
  }
  const pickTol = (e) => (e.pointerType === 'touch' ? 22 : 12) / view.scale;

  canvas.addEventListener('pointerdown', (e) => {
    const p = localPoint(e);
    pointer = { ...p, type: e.pointerType };
    if (mode !== 'build') { updateHover(p.x, p.y, e.pointerType === 'touch'); return; }
    const w = toWorld(p.x, p.y);
    if (e.button === 2 || tool === 'delete') {
      e.preventDefault();
      deleteAt(w, pickTol(e));
      updateHover(p.x, p.y, e.pointerType === 'touch');
      return;
    }
    if (e.button !== 0) return;
    const j = jointNear(w, Math.max(pickTol(e), 0.45));
    if (!j) {
      toast('Start a member from a joint or from an orange anchor.', 'warn');
      return;
    }
    try { canvas.setPointerCapture(e.pointerId); } catch (err) { /* ignore */ }
    drag = { from: j, to: null, pointerId: e.pointerId };
    hideTooltip();
  });

  canvas.addEventListener('pointermove', (e) => {
    const p = localPoint(e);
    pointer = { ...p, type: e.pointerType };
    if (drag && drag.pointerId === e.pointerId) {
      const end = dragEndPoint(drag.from, toWorld(p.x, p.y));
      drag.to = end && (end.x !== drag.from.x || end.y !== drag.from.y) ? end : null;
      return;
    }
    if (e.pointerType !== 'touch') updateHover(p.x, p.y, false);
  });

  canvas.addEventListener('pointerup', (e) => {
    if (drag && drag.pointerId === e.pointerId) {
      commitDrag();
      drag = null;
    }
  });
  canvas.addEventListener('pointercancel', () => { drag = null; });
  canvas.addEventListener('pointerleave', (e) => { if (e.pointerType !== 'touch' && !drag) hideTooltip(); });
  canvas.addEventListener('contextmenu', (e) => e.preventDefault());

  toolButtons.forEach((b) => b.addEventListener('click', () => selectTool(b.dataset.tool)));
  function selectTool(t) {
    if (mode !== 'build') return;
    tool = t;
    toolButtons.forEach((b) => {
      const on = b.dataset.tool === t;
      b.classList.toggle('active', on);
      b.setAttribute('aria-checked', on ? 'true' : 'false');
    });
    canvas.style.cursor = t === 'delete' ? 'not-allowed' : 'crosshair';
  }

  btnUndo.addEventListener('click', undo);
  btnClear.addEventListener('click', clearAll);
  btnSolution.addEventListener('click', showSolution);
  btnTC.addEventListener('click', () => {
    showTC = !showTC;
    btnTC.classList.toggle('on', showTC);
    btnTC.setAttribute('aria-pressed', String(showTC));
    legend.querySelector('.legend-keys').style.visibility = showTC ? 'visible' : 'hidden';
  });
  btnTest.addEventListener('click', () => {
    if (mode === 'build') startTest();
    else backToBuild();
  });

  window.addEventListener('keydown', (e) => {
    if (document.querySelector('.modal:not([hidden])')) {
      if (e.key === 'Escape') document.querySelectorAll('.modal:not([hidden])').forEach((m) => closeModal(m.id));
      return;
    }
    if ((e.ctrlKey || e.metaKey) && e.key.toLowerCase() === 'z') { e.preventDefault(); undo(); return; }
    if (e.key === '1') selectTool('road');
    else if (e.key === '2') selectTool('steel');
    else if (e.key === '3' || e.key === 'd' || e.key === 'Delete') selectTool('delete');
    else if (e.key === 'Escape') drag = null;
    else if (e.key === ' ') { e.preventDefault(); btnTest.click(); }
  });

  // ------------------------------------------------------------------
  // Modals
  // ------------------------------------------------------------------
  function openModal(id) { $(id).hidden = false; const f = $(id).querySelector('button'); if (f) f.focus(); }
  function closeModal(id) { $(id).hidden = true; }
  document.querySelectorAll('.modal').forEach((m) => {
    m.addEventListener('click', (e) => {
      if (e.target === m && m.id !== 'resultModal') closeModal(m.id);
      if (e.target.closest('[data-close]')) {
        closeModal(m.id);
        if (m.id === 'helpModal') { store.seenHelp = true; saveStore(); }
      }
    });
  });
  $('btnHelp').addEventListener('click', () => openModal('helpModal'));
  $('btnLevels').addEventListener('click', openLevels);

  function starsHtml(n) {
    return [0, 1, 2].map((i) => `<span class="star${i < n ? ' on' : ''}">★</span>`).join('');
  }

  function isUnlocked(i) {
    return i === 0 || !!store.progress[LEVELS[i - 1].id];
  }

  function openLevels() {
    const grid = $('levelGrid');
    grid.innerHTML = '';
    LEVELS.forEach((L, i) => {
      const prog = store.progress[L.id];
      const b = document.createElement('button');
      b.type = 'button';
      b.className = 'level-card' + (i === levelIndex ? ' current' : '');
      b.disabled = !isUnlocked(i);
      const extra = L.piers.length ? ' · mid pier' : '';
      b.innerHTML =
        `<span class="num">Level ${i + 1}${b.disabled ? ' · locked' : ''}</span>` +
        `<span class="name">${L.name}</span>` +
        `<span class="meta">${L.gap} m gap · ${L.truck.weight} kN truck${extra}<br>` +
        `Par ${fmtMoney(parFor(L))} · budget ${fmtMoney(L.budget)}` +
        (prog ? `<br>Best ${fmtMoney(prog.bestCost)}` : '') + `</span>` +
        `<span class="rules">${ruleNames(L).map(capitalise).join(' · ')}</span>` +
        `<div class="card-stars">${starsHtml(prog ? prog.stars : 0)}</div>`;
      b.addEventListener('click', () => {
        closeModal('levelModal');
        if (mode !== 'build') { closeModal('resultModal'); sim = null; }
        loadLevel(i);
      });
      grid.appendChild(b);
    });
    openModal('levelModal');
  }

  function showResult({ title, stars, text, stats, actions, delay }) {
    $('resultTitle').textContent = title;
    $('resultStars').innerHTML = stars ? starsHtml(stars) : '';
    $('resultStars').hidden = !stars;
    $('resultText').textContent = text;
    $('resultStats').innerHTML = stats.map(([k, v]) => `<dt>${k}</dt><dd>${v}</dd>`).join('');
    const act = $('resultActions');
    act.innerHTML = '';
    for (const a of actions) {
      const b = document.createElement('button');
      b.type = 'button';
      b.className = a.cls;
      b.textContent = a.label;
      b.addEventListener('click', a.fn);
      act.appendChild(b);
    }
    // Let the collapse / arrival be seen for a moment before the card appears
    const wait = delay !== undefined ? delay : stars ? 500 : 150;
    setTimeout(() => { if (mode === 'result') openModal('resultModal'); }, wait);
  }

  // ------------------------------------------------------------------
  // Truss guide: a small illustrated textbook. Every diagram is solved with
  // the game's own truss solver (equal loads P at the deck joints, pin at the
  // left, roller at the right), so colours, line weights and values are real.
  // ------------------------------------------------------------------
  const Guide = (() => {
    const SPAN = 12;

    function builder() {
      const nodes = [], members = [], idx = new Map();
      const n = (x, y) => {
        const k = x + ',' + y;
        if (!idx.has(k)) { idx.set(k, nodes.length); nodes.push([x, y]); }
        return idx.get(k);
      };
      const m = (x1, y1, x2, y2, role) => members.push([n(x1, y1), n(x2, y2), role]);
      const done = (loadXs, loadY, extra) => Object.assign({
        nodes, members, pin: n(0, 0), roller: n(SPAN, 0), loads: loadXs.map((x) => n(x, loadY))
      }, extra);
      return { n, m, done };
    }
    const deckJoints = [2, 4, 6, 8, 10];

    /** Pratt / Howe through truss (deck on the bottom chord). */
    function prattHowe(howe, h = 2) {
      const B = builder();
      for (let x = 0; x < SPAN; x += 2) B.m(x, 0, x + 2, 0, 'Bottom chord');
      for (let x = 2; x < SPAN - 2; x += 2) B.m(x, h, x + 2, h, 'Top chord');
      for (let x = 2; x < SPAN; x += 2) B.m(x, 0, x, h, 'Vertical');
      B.m(0, 0, 2, h, 'End post'); B.m(SPAN, 0, SPAN - 2, h, 'End post');
      for (let x = 2; x < SPAN - 2; x += 2) {
        const leftHalf = x + 2 <= SPAN / 2;
        // Pratt: diagonals slope down toward mid-span. Howe: up toward mid-span.
        if (leftHalf !== howe) B.m(x, h, x + 2, 0, 'Diagonal'); else B.m(x, 0, x + 2, h, 'Diagonal');
      }
      return B.done(deckJoints, 0, { loadFrom: 'below' });
    }
    function warren() {
      const B = builder(), p = 3, h = 2.6;
      for (let x = 0; x < SPAN; x += p) B.m(x, 0, x + p, 0, 'Bottom chord');
      for (let x = p / 2; x + p < SPAN; x += p) B.m(x, h, x + p, h, 'Top chord');
      for (let x = 0; x < SPAN; x += p) { B.m(x, 0, x + p / 2, h, 'Diagonal'); B.m(x + p / 2, h, x + p, 0, 'Diagonal'); }
      return B.done([3, 6, 9], 0, { loadFrom: 'below' });
    }
    function warrenVerticals() {
      const B = builder(), h = 2;
      for (let x = 0; x < SPAN; x += 2) B.m(x, 0, x + 2, 0, 'Bottom chord');
      B.m(2, h, 6, h, 'Top chord'); B.m(6, h, 10, h, 'Top chord');
      for (let x = 0; x < SPAN; x += 4) { B.m(x, 0, x + 2, h, 'Diagonal'); B.m(x + 2, h, x + 4, 0, 'Diagonal'); }
      for (const x of [2, 6, 10]) B.m(x, 0, x, h, 'Vertical');
      return B.done(deckJoints, 0, { loadFrom: 'below' });
    }
    function kTruss() {
      const B = builder(), h = 3, mid = 1.5;
      for (let x = 0; x < SPAN; x += 2) B.m(x, 0, x + 2, 0, 'Bottom chord');
      for (let x = 2; x < 10; x += 2) B.m(x, h, x + 2, h, 'Top chord');
      B.m(0, 0, 2, h, 'End post'); B.m(SPAN, 0, 10, h, 'End post');
      B.m(2, 0, 2, h, 'Vertical'); B.m(10, 0, 10, h, 'Vertical');
      for (const x of [4, 6, 8]) { B.m(x, 0, x, mid, 'Half vertical'); B.m(x, mid, x, h, 'Half vertical'); }
      for (const [x, to] of [[4, 2], [6, 4], [8, 10], [6, 8]]) { B.m(x, mid, to, h, 'K diagonal'); B.m(x, mid, to, 0, 'K diagonal'); }
      return B.done(deckJoints, 0, { loadFrom: 'below' });
    }
    function deckTruss() {
      const B = builder(), h = 2;
      for (let x = 0; x < SPAN; x += 2) B.m(x, 0, x + 2, 0, 'Top chord (deck)');
      for (let x = 2; x < SPAN - 2; x += 2) B.m(x, -h, x + 2, -h, 'Bottom chord');
      for (let x = 2; x < SPAN; x += 2) B.m(x, 0, x, -h, 'Post');
      B.m(0, 0, 2, -h, 'End diagonal'); B.m(SPAN, 0, SPAN - 2, -h, 'End diagonal');
      for (let x = 2; x < SPAN - 2; x += 2) {
        // Diagonals slope down from the deck toward mid-span, so they pull
        if (x + 2 <= SPAN / 2) B.m(x, 0, x + 2, -h, 'Diagonal'); else B.m(x, -h, x + 2, 0, 'Diagonal');
      }
      return B.done(deckJoints, 0, { loadFrom: 'above' });
    }
    function bowstring() {
      const B = builder(), rise = 3;
      const ya = (x) => +(rise * (1 - ((x - 6) / 6) ** 2)).toFixed(4); // parabola
      for (let x = 0; x < SPAN; x += 2) B.m(x, 0, x + 2, 0, 'Tie (bottom chord)');
      for (let x = 0; x < SPAN; x += 2) B.m(x, ya(x), x + 2, ya(x + 2), 'Arch (top chord)');
      for (let x = 2; x < SPAN; x += 2) B.m(x, 0, x, ya(x), 'Hanger');
      for (let x = 2; x < SPAN - 2; x += 2) {
        if (x + 2 <= SPAN / 2) B.m(x, ya(x), x + 2, 0, 'Diagonal'); else B.m(x, 0, x + 2, ya(x + 2), 'Diagonal');
      }
      return B.done(deckJoints, 0, { loadFrom: 'below' });
    }

    function solve(t) {
      const nodes = t.nodes.map(([x, y], i) => ({ x, y, fixX: i === t.pin, fixY: i === t.pin || i === t.roller }));
      const F = new Float64Array(nodes.length * 2);
      for (const i of t.loads) F[2 * i + 1] -= 1;
      const r = TS.solveTruss(nodes, t.members.map(([a, b]) => ({ a, b, EA: 1e5 })), F);
      return r.ok ? r.forces : null;
    }
    const maxChord = (t, forces) => Math.max(...t.members.map((m, i) => (/chord/i.test(m[2]) ? Math.abs(forces[i]) : 0)));

    /** SVG force diagram. Blue = tension, orange = compression, grey dashed = zero force. */
    function svg(t, { labels = false, compact = false } = {}) {
      const forces = solve(t);
      const fmax = Math.max(...forces.map(Math.abs));
      const S = 40, pad = 1.1;
      const xs = t.nodes.map((p) => p[0]), ys = t.nodes.map((p) => p[1]);
      const minX = Math.min(...xs) - pad, maxX = Math.max(...xs) + pad;
      const minY = Math.min(...ys) - pad - 0.4, maxY = Math.max(...ys) + pad;
      const X = (x) => ((x - minX) * S).toFixed(1), Y = (y) => ((maxY - y) * S).toFixed(1);
      const parts = [];
      t.members.forEach(([a, b, role], i) => {
        const f = forces[i], [x1, y1] = t.nodes[a], [x2, y2] = t.nodes[b];
        const zero = Math.abs(f) < 1e-3 * fmax; // under 0.1% of the largest force
        const cls = zero ? 'g-zero' : f > 0 ? 'g-t' : 'g-c';
        const w = zero ? 1.5 : 2 + (6 * Math.abs(f)) / fmax;
        const what = zero ? 'zero force' : `${f > 0 ? 'tension' : 'compression'} ${Math.abs(f).toFixed(2)} P`;
        parts.push(`<line class="${cls}" x1="${X(x1)}" y1="${Y(y1)}" x2="${X(x2)}" y2="${Y(y2)}" stroke-width="${w.toFixed(1)}"><title>${role}: ${what}</title></line>`);
      });
      if (labels) {
        t.members.forEach(([a, b], i) => {
          const f = forces[i], [x1, y1] = t.nodes[a], [x2, y2] = t.nodes[b];
          const txt = Math.abs(f) < 1e-3 * fmax ? '0' : (f > 0 ? '+' : '−') + Math.abs(f).toFixed(2);
          parts.push(`<text class="g-label" x="${X((x1 + x2) / 2)}" y="${Y((y1 + y2) / 2)}">${txt}</text>`);
        });
      }
      for (const [x, y] of t.nodes) parts.push(`<circle class="g-joint" cx="${X(x)}" cy="${Y(y)}" r="3.5"/>`);
      // Supports: pin (fixed triangle) and roller (triangle on wheels)
      for (const [i, roller] of [[t.pin, false], [t.roller, true]]) {
        const [x, y] = t.nodes[i], cx = +X(x), cy = +Y(y);
        parts.push(`<path class="g-support" d="M${cx} ${cy + 4} l-9 15 h18 z"/>`);
        if (roller) parts.push(`<circle class="g-support" cx="${cx - 5}" cy="${cy + 23}" r="3"/><circle class="g-support" cx="${cx + 5}" cy="${cy + 23}" r="3"/>`);
        parts.push(`<line class="g-ground" x1="${cx - 14}" y1="${cy + (roller ? 27 : 20)}" x2="${cx + 14}" y2="${cy + (roller ? 27 : 20)}"/>`);
      }
      // Loads P at the deck joints
      for (const i of t.loads) {
        const [x, y] = t.nodes[i], cx = +X(x), cy = +Y(y);
        const [y0, y1] = t.loadFrom === 'above' ? [cy - 30, cy - 6] : [cy + 6, cy + 30];
        parts.push(`<line class="g-load" x1="${cx}" y1="${y0}" x2="${cx}" y2="${y1 - 5}"/><path class="g-load-head" d="M${cx - 4} ${y1 - 6} L${cx + 4} ${y1 - 6} L${cx} ${y1} z"/>`);
      }
      const w = ((maxX - minX) * S).toFixed(0), h = ((maxY - minY) * S).toFixed(0);
      return `<svg class="g-svg${compact ? ' compact' : ''}" viewBox="0 0 ${w} ${h}" role="img" aria-label="Force diagram">${parts.join('')}</svg>`;
    }

    // Fixed diagram for "why triangles"
    const squareSvg =
      '<svg class="g-svg compact" viewBox="0 0 420 150" role="img" aria-label="A square folds, a triangle does not">' +
      '<g transform="translate(30 20)">' +
      '<path class="g-ghost" d="M0 100 L40 0 L140 0 L100 100 Z"/>' +
      '<path class="g-frame" d="M0 100 L0 0 L100 0 L100 100"/>' +
      '<circle class="g-joint" cx="0" cy="0" r="4"/><circle class="g-joint" cx="100" cy="0" r="4"/>' +
      '<circle class="g-joint" cx="0" cy="100" r="4"/><circle class="g-joint" cx="100" cy="100" r="4"/>' +
      '<text class="g-caption" x="50" y="125">square: folds</text></g>' +
      '<g transform="translate(250 20)">' +
      '<path class="g-frame" d="M0 100 L0 0 L100 0 L100 100 M0 100 L100 0"/>' +
      '<circle class="g-joint" cx="0" cy="0" r="4"/><circle class="g-joint" cx="100" cy="0" r="4"/>' +
      '<circle class="g-joint" cx="0" cy="100" r="4"/><circle class="g-joint" cx="100" cy="100" r="4"/>' +
      '<text class="g-caption" x="50" y="125">braced: rigid</text></g>' +
      '<line class="g-ground" x1="20" y1="120" x2="160" y2="120"/><line class="g-ground" x1="240" y1="120" x2="380" y2="120"/></svg>';

    const TRUSSES = [
      {
        id: 'pratt', name: 'Pratt', title: 'Pratt truss', build: () => prattHowe(false), year: 'Thomas and Caleb Pratt, 1844',
        how: 'The diagonals slope down toward mid-span. Under gravity they are pulled (tension), while the interior verticals are pushed (compression). As in every simply supported truss, the top chord is squeezed and the bottom chord is stretched.',
        good: 'The longest web members, the diagonals, are in tension, where length does not weaken them. The short verticals take the compression. That makes the Pratt very efficient in steel, and it became the standard railway and road truss.',
        watch: 'The verticals next to the supports are hangers in tension. Under a symmetric load the middle vertical carries nothing: it is a zero-force member (grey).',
        game: 'From level 3, buckling is on, and a Pratt keeps long members out of compression. If you build it under the deck, reverse the diagonals (see Deck truss).'
      },
      {
        id: 'howe', name: 'Howe', title: 'Howe truss', build: () => prattHowe(true), year: 'William Howe, 1840',
        how: 'The mirror image of the Pratt: the diagonals slope up toward mid-span, so they are in compression and the verticals are in tension.',
        good: 'It was designed for timber. Wooden diagonals are good at taking compression, and thin iron rods made the tension verticals. The chord forces are the same as in a Pratt.',
        watch: 'In steel, the long diagonals in compression are the members most likely to buckle, which is why steel bridges usually prefer the Pratt.',
        game: 'On levels 1 and 2 (no buckling) a Howe works as well as a Pratt. From level 3, its long compression diagonals lose strength with 1/L².'
      },
      {
        id: 'warren', name: 'Warren', title: 'Warren truss', build: warren, year: 'James Warren and Willoughby Monzani, 1848',
        how: 'Equilateral or isosceles triangles with no verticals. The diagonals alternate between compression and tension. The diagonal forces shrink toward mid-span while the chord forces grow.',
        good: 'It needs the fewest members and joints for a given span, and the members are all similar in length. That makes it simple, light and cheap to build.',
        watch: 'The panels are long, so the deck spans far between joints and the compression chord has long unbraced lengths.',
        game: 'Few joints means low joint cost. Shallow Warren trusses under the deck are the starting point for the par designs on levels 1, 2 and 6. Watch the deck bending on long road segments.'
      },
      {
        id: 'warrenv', name: 'Warren with verticals', title: 'Warren truss with verticals', build: warrenVerticals, year: 'Common variant',
        how: 'A Warren truss with verticals added at the top chord joints. Here they act as hangers in tension, lifting the deck load at those joints up into the top chord.',
        good: 'The deck gets support points between the diagonal joints, so each road segment spans less and the load enters the truss at more points.',
        watch: 'Every vertical adds a member and a joint to pay for. Verticals that carry no load only add cost.',
        game: 'Useful when a road segment is close to its bending limit under a heavy truck: a vertical halves the segment span.'
      },
      {
        id: 'k', name: 'K truss', title: 'K truss', build: kTruss, year: 'Deep trusses and towers',
        how: 'Each interior vertical is split at mid-height, and two short diagonals meet there to form a K. In each K one diagonal pulls while the other pushes.',
        good: 'Every compression member is short (half verticals and short K diagonals), so a deep truss can be built without long members that buckle.',
        watch: 'It has many members and joints. It pays off only when the truss must be deep and buckling is the main problem.',
        game: 'Expensive with joint cost. Worth trying only if long compression members keep failing and a shallower truss will not do.'
      },
      {
        id: 'deck', name: 'Deck truss', title: 'Deck truss', build: deckTruss, year: 'Truss under the road',
        how: 'The truss sits under the road, so the deck itself is the top chord in compression and the bottom chord is in tension. With the diagonals sloping down from the deck toward mid-span they pull, and the verticals become short posts in compression.',
        good: 'Nothing is above the road, and the deck works as part of the truss. When you flip a truss under the deck, flip its diagonal pattern too, or the diagonals end up in compression.',
        watch: 'It needs clear space below the road. With supports only at deck level (as here) the end diagonals carry everything down to the supports, so they work hard.',
        game: 'Every level has anchors on the cliff faces, and five of the six par designs are trusses under the deck. Tying the bottom chord into those lower anchors gives the load a second, shorter path into the cliff.'
      },
      {
        id: 'bowstring', name: 'Bowstring arch', title: 'Bowstring arch truss', build: bowstring, year: 'Squire Whipple, 1841',
        how: 'The top chord is curved like an arch, and the bottom chord ties its ends together so the arch cannot spread. When the arch follows a parabola, the shape of the bending moment under an even load, the arch and the tie carry everything.',
        good: 'Under an even load the diagonals carry almost nothing (grey here) and the hangers simply lift the deck. Very little material is wasted.',
        watch: 'A single moving load such as the truck is not even, so the diagonals do work as it passes. A curved chord also needs many joints.',
        game: 'On a 1 m grid the curve becomes short straight chords, and each corner costs a joint. It can still pay off on long spans.'
      }
    ];

    function basicsHtml() {
      const shallow = prattHowe(false, 1), deep = prattHowe(false, 3);
      const fs = maxChord(shallow, solve(shallow)), fd = maxChord(deep, solve(deep));
      const rows = [1, 2, 3, 4, 5].map((L) => {
        const yieldOnly = TS.memberCapacity(MATERIALS.steel, L, { buckling: false, phi: CONFIG.phi }).compression;
        const withBuckling = TS.memberCapacity(MATERIALS.steel, L, { buckling: true, phi: CONFIG.phi }).compression;
        const allowed = L <= MAX_LEN ? '' : ' class="muted"';
        return `<tr${allowed}><td>${L} m</td><td>${Math.round(yieldOnly)} kN</td><td>${Math.round(withBuckling)} kN</td></tr>`;
      }).join('');
      return `
        <h3>Why triangles?</h3>
        ${squareSvg}
        <p>A pin-jointed square can fold into a parallelogram without any member changing length: it is a <b>mechanism</b>. A diagonal splits it into two triangles, and a triangle can only change shape if a member stretches or shortens. Every stable truss is built from triangles. A plane truss with <i>j</i> joints and <i>r</i> support reactions needs at least <i>m</i> = 2<i>j</i> − <i>r</i> members. In LoadPath, an unstable bridge wobbles the joints that are free to move.</p>
        <h3>A truss is a beam with holes</h3>
        <div class="g-pair">
          <figure>${svg(shallow, { compact: true })}<figcaption>1 m deep: largest chord force ${fs.toFixed(1)} P</figcaption></figure>
          <figure>${svg(deep, { compact: true })}<figcaption>3 m deep: largest chord force ${fd.toFixed(1)} P</figcaption></figure>
        </div>
        <p>Under gravity the top chord is squeezed and the bottom chord stretched, just like the top and bottom of a bending beam. The chords resist the bending moment <i>M</i> as a pair of forces: chord force ≈ <i>M</i> / <i>h</i>. Make the truss three times deeper and the chord forces drop to a third. The trade-off is longer web members.</p>
        <h3>Tension is easy, compression buckles</h3>
        <p>A member in tension can use its full strength at any length. A member in compression can bow sideways and <b>buckle</b> before it yields. Euler's ideal buckling load is <i>P</i><sub>cr</sub> = π²<i>EI</i> / <i>L</i>², so a member half as long is four times as strong. Real members are never perfectly straight, so design codes use a <b>column curve</b> that sits below Euler's ideal, and so does this game (AISC 360). Steel compression capacity in this game (φ = ${CONFIG.phi} included):</p>
        <table class="g-table"><thead><tr><th>Length</th><th>Levels 1–2</th><th>Levels 3+ (buckling)</th></tr></thead><tbody>${rows}</tbody></table>
        <p class="muted small">Greyed rows are longer than the ${MAX_LEN} m member limit.</p>
        <h3>Zero-force members</h3>
        <p>At an unloaded joint where two members are in line and a third meets them, the third member carries nothing (look at the Pratt's middle vertical). Zero-force members still matter: they brace other members and keep the truss stable, so check before deleting one.</p>
        <h3>Reading the diagrams</h3>
        <p><span class="g-key g-key-t"></span>Tension <span class="g-key g-key-c"></span>Compression <span class="g-key g-key-z"></span>Zero force. Thicker means more force. Each arrow is an equal load <i>P</i> at a deck joint. Values are in multiples of <i>P</i>; hover a member to read it.</p>`;
    }

    function trussHtml(t, labels) {
      const model = t.build();
      return `
        <p class="g-year">${t.year}</p>
        ${svg(model, { labels })}
        <p class="g-caption-line"><span class="g-key g-key-t"></span>Tension <span class="g-key g-key-c"></span>Compression <span class="g-key g-key-z"></span>Zero force · equal loads <i>P</i> at the deck joints · hover a member or turn on Show values</p>
        <h3>How it carries load</h3><p>${t.how}</p>
        <h3>Strengths</h3><p>${t.good}</p>
        <h3>Watch out for</h3><p>${t.watch}</p>
        <h3>In LoadPath</h3><p>${t.game}</p>`;
    }

    let current = 'basics';
    let labels = false;
    function render() {
      const nav = $('guideNav');
      nav.innerHTML = '';
      for (const item of [{ id: 'basics', name: 'Basics' }, ...TRUSSES]) {
        const b = document.createElement('button');
        b.type = 'button';
        b.className = 'g-nav' + (item.id === current ? ' active' : '');
        b.textContent = item.name;
        b.setAttribute('aria-current', item.id === current ? 'page' : 'false');
        b.addEventListener('click', () => { current = item.id; render(); $('guideBody').scrollTop = 0; });
        nav.appendChild(b);
      }
      const t = TRUSSES.find((x) => x.id === current);
      $('guideTitle').textContent = t ? t.title : 'Truss basics';
      $('guideLabelsWrap').hidden = !t;
      $('guideContent').innerHTML = t ? trussHtml(t, labels) : basicsHtml();
    }
    $('guideLabels').addEventListener('change', (e) => { labels = e.target.checked; render(); });
    return {
      open() { render(); openModal('guideModal'); }
    };
  })();
  $('btnGuide').addEventListener('click', () => Guide.open());

  // ------------------------------------------------------------------
  // Main loop
  // ------------------------------------------------------------------
  let last = performance.now();
  function frame(now) {
    const dt = Math.min(0.05, (now - last) / 1000);
    last = now;
    time += dt;
    if (flashDeck > 0) flashDeck -= dt;
    if (mech) { mech.t += dt; if (mech.t > 4) mech = null; }

    if (mode === 'test') { updateTest(dt); updateHud(); }
    else if (sim && sim.collapse && (mode === 'collapse' || mode === 'result')) { updateCollapse(dt); updateHud(); }

    // keep the tooltip live while the truck moves
    if (mode !== 'build' && pointer && !tooltip.hidden && pointer.type !== 'touch') updateHover(pointer.x, pointer.y, false);

    render();
    requestAnimationFrame(frame);
  }

  // ------------------------------------------------------------------
  // Boot
  // ------------------------------------------------------------------
  // Developer check: every stored par design must pass under the current CONFIG
  for (const L of LEVELS) {
    const sol = SOLUTIONS[L.id];
    const res = sol ? evaluateDesign(L, sol) : null;
    if (!res || !res.passed) {
      console.warn(`LoadPath: the par design for ${L.id} does not pass under the current CONFIG. ` +
        'Rerun: node tools/generate-solutions.js');
    }
    if (sol && designCost(L, sol) > L.budget) console.warn(`LoadPath: par for ${L.id} is over its budget.`);
  }

  new ResizeObserver(resize).observe(stage);
  resize();
  const startLevel = Number.isInteger(store.lastLevel) && isUnlocked(store.lastLevel) ? store.lastLevel : 0;
  loadLevel(startLevel);
  selectTool('road');
  if (!store.seenHelp) openModal('helpModal');
  requestAnimationFrame(frame);
})();
