/*
 * truss-solver.js
 * 2D pin-jointed truss analysis using the direct stiffness method.
 *
 * Units used by the game: metres and kilonewtons (EA in kN, forces in kN).
 * Sign convention: member force > 0 is TENSION, < 0 is COMPRESSION.
 *
 * Works in the browser (window.TrussSolver) and in Node (module.exports).
 */
(function (root, factory) {
  if (typeof module === 'object' && module.exports) module.exports = factory();
  else root.TrussSolver = factory();
})(typeof self !== 'undefined' ? self : this, function () {
  'use strict';

  /**
   * Assemble and factorise the global stiffness matrix for a truss.
   *
   * Because the geometry (and therefore K) does not change while the truck
   * drives across, K is LU-factorised once and each load case afterwards is
   * just a cheap forward/back substitution.
   *
   * @param {Array<{x:number,y:number,fixX?:boolean,fixY?:boolean}>} nodes
   * @param {Array<{a:number,b:number,EA:number}>} members  a, b are node indices
   * @returns {{ok:true, solve:Function, lengths:Float64Array} |
   *           {ok:false, reason:string, dof:number, node:number, mode:Float64Array}}
   */
  function createSolver(nodes, members) {
    const n = nodes.length;
    const m = members.length;

    // Map each unrestrained degree of freedom (2 per node: x, y) to a row of
    // the reduced system. Restrained DOFs (supports) are simply removed.
    const dofMap = new Int32Array(2 * n).fill(-1);
    const dofOwner = []; // reduced dof -> global dof
    let nf = 0;
    for (let i = 0; i < n; i++) {
      if (!nodes[i].fixX) { dofMap[2 * i] = nf++; dofOwner.push(2 * i); }
      if (!nodes[i].fixY) { dofMap[2 * i + 1] = nf++; dofOwner.push(2 * i + 1); }
    }

    // Member geometry: length and direction cosines.
    const L = new Float64Array(m);
    const C = new Float64Array(m);
    const S = new Float64Array(m);
    const k = new Float64Array(m);
    for (let e = 0; e < m; e++) {
      const A = nodes[members[e].a];
      const B = nodes[members[e].b];
      const dx = B.x - A.x;
      const dy = B.y - A.y;
      const len = Math.hypot(dx, dy);
      if (!(len > 0)) throw new Error('Member ' + e + ' has zero length');
      L[e] = len;
      C[e] = dx / len;
      S[e] = dy / len;
      k[e] = members[e].EA / len; // axial stiffness EA/L
    }

    // Assemble K (dense; bridges here have well under 100 DOFs).
    // Element stiffness in global coordinates:
    //   k * [ cc  cs -cc -cs ]
    //       [ cs  ss -cs -ss ]
    //       [-cc -cs  cc  cs ]
    //       [-cs -ss  cs  ss ]
    const K = new Float64Array(nf * nf);
    for (let e = 0; e < m; e++) {
      const c = C[e], s = S[e];
      const g = [2 * members[e].a, 2 * members[e].a + 1, 2 * members[e].b, 2 * members[e].b + 1];
      const v = [c, s, -c, -s]; // ke = k * v vᵀ
      for (let i = 0; i < 4; i++) {
        const r = dofMap[g[i]];
        if (r < 0) continue;
        for (let j = 0; j < 4; j++) {
          const q = dofMap[g[j]];
          if (q < 0) continue;
          K[r * nf + q] += k[e] * v[i] * v[j];
        }
      }
    }

    // LU factorisation with partial pivoting (in place, Doolittle form).
    let maxDiag = 0;
    for (let i = 0; i < nf; i++) maxDiag = Math.max(maxDiag, Math.abs(K[i * nf + i]));
    const tol = Math.max(maxDiag, 1e-30) * 1e-9;
    const perm = new Int32Array(nf);
    for (let i = 0; i < nf; i++) perm[i] = i;

    for (let col = 0; col < nf; col++) {
      let piv = col;
      let best = Math.abs(K[col * nf + col]);
      for (let r = col + 1; r < nf; r++) {
        const val = Math.abs(K[r * nf + col]);
        if (val > best) { best = val; piv = r; }
      }

      if (best < tol) {
        // Singular: the structure is a mechanism. Recover a mechanism mode
        // (a null vector of K) by setting this DOF to 1 and back-substituting
        // through the already-eliminated upper rows. The game uses it to
        // show which joints are free to move.
        const x = new Float64Array(nf);
        x[col] = 1;
        for (let r = col - 1; r >= 0; r--) {
          let sum = K[r * nf + col];
          for (let j = r + 1; j < col; j++) sum += K[r * nf + j] * x[j];
          x[r] = -sum / K[r * nf + r];
        }
        const mode = new Float64Array(2 * n);
        for (let i = 0; i < nf; i++) mode[dofOwner[i]] = x[i];
        const g = dofOwner[col];
        return { ok: false, reason: 'mechanism', dof: g, node: g >> 1, mode: mode };
      }

      if (piv !== col) {
        for (let j = 0; j < nf; j++) {
          const t = K[col * nf + j]; K[col * nf + j] = K[piv * nf + j]; K[piv * nf + j] = t;
        }
        const t = perm[col]; perm[col] = perm[piv]; perm[piv] = t;
      }

      const d = K[col * nf + col];
      for (let r = col + 1; r < nf; r++) {
        const f = (K[r * nf + col] /= d);
        if (f === 0) continue;
        for (let j = col + 1; j < nf; j++) K[r * nf + j] -= f * K[col * nf + j];
      }
    }

    /**
     * Solve K·u = F for one load case.
     * @param {ArrayLike<number>} loads  length 2n: [Fx0, Fy0, Fx1, Fy1, ...] in kN
     * @returns {{displacements:Float64Array, forces:Float64Array}}
     */
    function solve(loads) {
      const y = new Float64Array(nf);
      for (let i = 0; i < nf; i++) y[i] = loads[dofOwner[perm[i]]] || 0;
      // Forward substitution (L has unit diagonal)
      for (let i = 0; i < nf; i++) {
        let s = y[i];
        for (let j = 0; j < i; j++) s -= K[i * nf + j] * y[j];
        y[i] = s;
      }
      // Back substitution
      for (let i = nf - 1; i >= 0; i--) {
        let s = y[i];
        for (let j = i + 1; j < nf; j++) s -= K[i * nf + j] * y[j];
        y[i] = s / K[i * nf + i];
      }

      const u = new Float64Array(2 * n);
      for (let i = 0; i < nf; i++) u[dofOwner[i]] = y[i];

      // Member axial force: N = (EA/L) * [c s]·(u_b - u_a)
      const forces = new Float64Array(m);
      for (let e = 0; e < m; e++) {
        const a = members[e].a, b = members[e].b;
        const du = u[2 * b] - u[2 * a];
        const dv = u[2 * b + 1] - u[2 * a + 1];
        forces[e] = k[e] * (C[e] * du + S[e] * dv);
      }
      return { displacements: u, forces: forces };
    }

    return { ok: true, solve: solve, lengths: L };
  }

  /** One-shot convenience wrapper. */
  function solveTruss(nodes, members, loads) {
    const s = createSolver(nodes, members);
    if (!s.ok) return s;
    const r = s.solve(loads);
    return { ok: true, displacements: r.displacements, forces: r.forces };
  }

  /**
   * Member design capacity in kN.
   *  - Yield capacity  P_y = fy · A
   *  - Euler load      P_e = π² · E · I / L²   (pin-ended, effective length K = 1)
   *  - Tension:     φ · P_y
   *  - Compression without buckling: φ · P_y
   *  - Compression with buckling, rules.bucklingCurve:
   *      'aisc'  (default) AISC 360 §E3 column curve, which allows for the
   *              initial crookedness and residual stresses of real members:
   *              P_n = 0.658^(P_y/P_e) · P_y   when P_y/P_e ≤ 2.25
   *              P_n = 0.877 · P_e             otherwise
   *      'euler' ideal Euler buckling capped at yield: min(P_y, P_e)
   * @param {{E:number, A:number, I:number, fy:number}} material  kN/m², m², m⁴, kN/m²
   * @param {number} length  metres
   * @param {{buckling?:boolean, phi?:number, bucklingCurve?:string}} [rules]
   */
  function memberCapacity(material, length, rules) {
    const phi = rules && typeof rules.phi === 'number' ? rules.phi : 1;
    const yieldCap = material.fy * material.A;
    const euler = (Math.PI * Math.PI * material.E * material.I) / (length * length);
    let nominal = yieldCap;
    if (rules && rules.buckling) {
      if (rules.bucklingCurve === 'euler') {
        nominal = Math.min(yieldCap, euler);
      } else {
        const ratio = yieldCap / euler;
        nominal = ratio <= 2.25 ? Math.pow(0.658, ratio) * yieldCap : 0.877 * euler;
      }
    }
    return {
      tension: phi * yieldCap,
      compression: phi * nominal,
      yieldCap: yieldCap,
      euler: euler,
      // "buckling" is worth mentioning once it costs more than 5% of yield
      bucklingGoverns: nominal < 0.95 * yieldCap
    };
  }

  /**
   * Combined axial force and bending (AISC 360 §H1-1), as a utilization:
   *   pr ≥ 0.2:  pr + 8/9 · mr      pr < 0.2:  pr / 2 + mr
   * pr = axial force / axial capacity, mr = moment / moment capacity.
   */
  function beamColumnRatio(pr, mr) {
    return pr >= 0.2 ? pr + (8 / 9) * mr : pr / 2 + mr;
  }

  /** |force| / capacity, using the tension or compression capacity as appropriate. */
  function utilization(force, capacity) {
    return force >= 0 ? force / capacity.tension : -force / capacity.compression;
  }

  /**
   * Truck positions for the moving-load check: every `step` metres from x0 to
   * x1, plus every deck joint. Within one deck member the truck load splits
   * linearly between its two joints, so member forces and deflections are
   * linear there and their extremes occur at deck joints. Including the joints
   * makes the envelope exact whatever the step.
   */
  function loadPositions(x0, x1, step, jointXs) {
    const set = new Set();
    const n = Math.max(1, Math.ceil((x1 - x0) / step - 1e-9));
    for (let i = 0; i <= n; i++) set.add(+Math.min(x1, x0 + i * step).toFixed(9));
    for (const x of jointXs || []) if (x >= x0 && x <= x1) set.add(x);
    return Array.from(set).sort((p, q) => p - q);
  }

  /**
   * Full moving-load evaluation of a bridge.
   *
   * @param {object} input
   * @param {Array<{x,y,fixX,fixY}>} input.nodes
   * @param {Array<{a:number,b:number,material:object}>} input.members
   *        material: {E, A, I, fy, weight (kN/m)}
   * @param {Array<{a:number,b:number,member?:number}>} input.deck
   *        road members the truck drives over, in order, left to right (node indices)
   * @param {number} input.truckWeight  kN, applied as a point load
   * @param {object} input.rules
   *        {buckling:boolean, deflection:boolean, phi:number,
   *         bucklingCurve:'aisc'|'euler', loadStep:number (m), deflectionLimit:number (m),
   *         deckMomentCapacity:number (kN·m, road segment M_p = fy·Z),
   *         deadFactor, liveFactor, impactFactor (default 1; strength checks only)}
   * @returns {object} {ok:false, mechanism} for an unstable structure, otherwise
   *   {ok:true, solver, caps, baseLoads, segs, loadsAt(x), positions,
   *    worstT, worstC, ratio (max of axial and bending), axialRatio, governs,
   *    bendingMoment, bendingRatio, momentCapacity (φ applied), maxRatio, critical,
   *    strengthFail:{x, member, util}|null,
   *    bendingFail:{x, member, moment, capacity}|null,
   *    deflFail:{x, node, value}|null,
   *    maxDefl, maxDeflX, maxDeflNode, deflectionLimit, passed}
   *   A position x of null means "no truck on the bridge".
   */
  function evaluateBridge(input) {
    const nodes = input.nodes, members = input.members, rules = input.rules || {};
    const W = input.truckWeight;
    const mems = members.map((m) => ({ a: m.a, b: m.b, EA: m.material.E * m.material.A }));
    const solver = createSolver(nodes, mems);
    if (!solver.ok) return { ok: false, mechanism: solver };

    const n = nodes.length, m = members.length;
    const baseLoads = new Float64Array(2 * n);
    const caps = members.map((mem, i) => {
      const w = mem.material.weight * solver.lengths[i]; // self-weight, half to each end
      baseLoads[2 * mem.a + 1] -= w / 2;
      baseLoads[2 * mem.b + 1] -= w / 2;
      return memberCapacity(mem.material, solver.lengths[i], rules);
    });

    const segs = input.deck.map((d) => ({ a: d.a, b: d.b, member: d.member, x0: nodes[d.a].x, x1: nodes[d.b].x }));
    function segmentAt(x) {
      if (x === null || x === undefined) return null;
      for (let i = 0; i < segs.length; i++) {
        const s = segs[i];
        if (x >= s.x0 && x <= s.x1) return { seg: s, index: i, t: (x - s.x0) / (s.x1 - s.x0) };
      }
      return null;
    }
    // Load factors. Strength checks use factored loads:
    //   deadFactor · self-weight + liveFactor · impactFactor · truck
    // Deflection (a serviceability check) uses unfactored self-weight plus the
    // truck with impact. All factors default to 1.
    const impact = rules.impactFactor > 0 ? rules.impactFactor : 1;
    const gD = rules.deadFactor > 0 ? rules.deadFactor : 1;
    const gL = (rules.liveFactor > 0 ? rules.liveFactor : 1) * impact;
    /** Nodal loads with the truck at x (null = no truck). service = deflection loads. */
    function loadsAt(x, service) {
      const dF = service ? 1 : gD, lF = service ? impact : gL;
      const F = new Float64Array(2 * n);
      for (let k = 0; k < F.length; k++) F[k] = baseLoads[k] * dF;
      const on = segmentAt(x);
      if (on) {
        F[2 * on.seg.a + 1] -= W * lF * (1 - on.t);
        F[2 * on.seg.b + 1] -= W * lF * on.t;
      }
      return F;
    }
    const sameLoads = gD === 1 && gL === impact; // strength and service loads coincide

    const deckNodes = [];
    for (const s of segs) for (const k of [s.a, s.b]) if (!nodes[k].fixY && deckNodes.indexOf(k) < 0) deckNodes.push(k);

    const step = rules.loadStep > 0 ? rules.loadStep : 0.5;
    const xs = segs.length ? loadPositions(segs[0].x0, segs[segs.length - 1].x1, step, segs.map((s) => s.x0)) : [];
    const positions = [null].concat(xs);
    const limit = rules.deflection ? rules.deflectionLimit : Infinity;

    const worstT = new Float64Array(m), worstC = new Float64Array(m);
    let strengthFail = null, deflFail = null;
    let maxDefl = 0, maxDeflX = null, maxDeflNode = -1;

    for (const x of positions) {
      const r = solver.solve(loadsAt(x));
      let worstHere = -1, worstU = 1;
      for (let i = 0; i < m; i++) {
        const f = r.forces[i];
        if (f > worstT[i]) worstT[i] = f;
        if (-f > worstC[i]) worstC[i] = -f;
        const u = utilization(f, caps[i]);
        if (u > worstU) { worstU = u; worstHere = i; }
      }
      if (!strengthFail && worstHere >= 0) strengthFail = { x: x, member: worstHere, util: worstU };
      const rs = sameLoads ? r : solver.solve(loadsAt(x, true));
      for (const k of deckNodes) {
        const d = Math.abs(rs.displacements[2 * k + 1]);
        if (d > maxDefl) { maxDefl = d; maxDeflX = x; maxDeflNode = k; }
        if (!deflFail && d > limit) deflFail = { x: x, node: k, value: d };
      }
    }

    // Deck bending: the truss model is axial-only, so each road segment the
    // truck drives over is also checked as a simply supported beam with the
    // truck at its midpoint, where its moment peaks:
    //   M = P·L/4 (truck) + w·L²/8 (the segment's own weight), factored
    // and combined with the segment's axial force at that moment (AISC H1-1).
    const bendingMoment = new Float64Array(m);   // factored M at midpoint, kN·m
    const deckAxialRatio = new Float64Array(m);  // axial force / capacity, truck at midpoint
    const deckMomentRatio = new Float64Array(m); // M / φM_p
    const bendingRatio = new Float64Array(m);    // combined (H1-1) utilization
    const momentCap = rules.deckMomentCapacity > 0 ? (rules.phi || 1) * rules.deckMomentCapacity : Infinity;
    let bendingFail = null;
    for (const s of segs) {
      if (s.member === undefined) continue;
      const len = solver.lengths[s.member];
      const w = members[s.member].material.weight;
      const xm = (s.x0 + s.x1) / 2;
      const Mmax = (gL * W * len) / 4 + (gD * w * len * len) / 8;
      const N = solver.solve(loadsAt(xm)).forces[s.member];
      const pr = utilization(N, caps[s.member]);
      const mr = Mmax / momentCap;
      const combined = beamColumnRatio(pr, mr);
      bendingMoment[s.member] = Mmax;
      deckAxialRatio[s.member] = pr;
      deckMomentRatio[s.member] = mr;
      bendingRatio[s.member] = combined;
      if (combined > 1 && (!bendingFail || xm < bendingFail.x)) {
        bendingFail = { x: xm, member: s.member, moment: Mmax, capacity: momentCap, axialRatio: pr, momentRatio: mr, util: combined };
      }
    }

    // Utilization per member = worst of axial (tension/compression) and bending
    const axialRatio = new Float64Array(m);
    const ratio = new Float64Array(m);
    const governs = new Array(m);
    let maxRatio = 0, critical = -1;
    for (let i = 0; i < m; i++) {
      const rt = worstT[i] / caps[i].tension;
      const rc = worstC[i] / caps[i].compression;
      axialRatio[i] = Math.max(rt, rc);
      governs[i] = rc > rt ? 'C' : 'T';
      ratio[i] = Math.max(axialRatio[i], bendingRatio[i]);
      if (ratio[i] > maxRatio) { maxRatio = ratio[i]; critical = i; }
    }

    return {
      ok: true, solver: solver, caps: caps, baseLoads: baseLoads, segs: segs,
      segmentAt: segmentAt, loadsAt: loadsAt, positions: positions, deckNodes: deckNodes,
      worstT: worstT, worstC: worstC, ratio: ratio, axialRatio: axialRatio, governs: governs,
      bendingMoment: bendingMoment, bendingRatio: bendingRatio, momentCapacity: momentCap,
      deckAxialRatio: deckAxialRatio, deckMomentRatio: deckMomentRatio,
      deadFactor: gD, liveFactor: gL, impactFactor: impact,
      maxRatio: maxRatio, critical: critical,
      strengthFail: strengthFail, bendingFail: bendingFail, deflFail: deflFail,
      maxDefl: maxDefl, maxDeflX: maxDeflX, maxDeflNode: maxDeflNode,
      deflectionLimit: limit,
      passed: !strengthFail && !bendingFail && !deflFail
    };
  }

  return {
    createSolver: createSolver,
    solveTruss: solveTruss,
    memberCapacity: memberCapacity,
    beamColumnRatio: beamColumnRatio,
    utilization: utilization,
    loadPositions: loadPositions,
    evaluateBridge: evaluateBridge
  };
});
