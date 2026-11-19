'use strict';

/* =====================================================================
 *  FÍSICA — pura, sem DOM (também roda no Node para testes/calibragem)
 *  Mundo em metros: x = lateral (centro do gol = 0), y = altura,
 *  z = distância até a linha do gol (campo em z > 0, rede em z < 0).
 * ===================================================================== */
const Physics = (() => {
  const G = 9.81;
  const R = 0.11;               // raio da bola
  const DRAG = 0.0125;          // arrasto quadrático (≈ Cd 0,25)
  const MAGNUS = 0.0052;        // efeito Magnus: a = k · (ω × v)
  const SPIN_DECAY = 0.25;
  const GOAL_HALF = 3.66, GOAL_H = 2.44, POST_R = 0.06, NET_DEPTH = 2.0;
  const POST_X = GOAL_HALF + POST_R, BAR_Y = GOAL_H + POST_R;
  const KEEPER_Z = 0.35, KEEPER_R = 0.26, KEEPER_LEN = 2.25;
  const WALL_DIST = 9.15, WALL_R = 0.24;
  const DT = 1 / 240;
  const MAX_SPIN = 75;          // rad/s

  const v3 = (x = 0, y = 0, z = 0) => ({ x, y, z });
  const add = (a, b) => v3(a.x + b.x, a.y + b.y, a.z + b.z);
  const sub = (a, b) => v3(a.x - b.x, a.y - b.y, a.z - b.z);
  const mul = (a, s) => v3(a.x * s, a.y * s, a.z * s);
  const dot = (a, b) => a.x * b.x + a.y * b.y + a.z * b.z;
  const len = (a) => Math.hypot(a.x, a.y, a.z);
  const clamp = (v, a, b) => Math.min(b, Math.max(a, v));
  const lerp = (a, b, t) => a + (b - a) * t;
  function gauss(rnd = Math.random) {
    const u = 1 - rnd(), v = rnd();
    return clamp(Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v), -3, 3);
  }

  // Gravidade + arrasto + Magnus
  function accel(v, w) {
    const s = len(v);
    return v3(
      -DRAG * s * v.x + MAGNUS * (w.y * v.z - w.z * v.y),
      -G - DRAG * s * v.y + MAGNUS * (w.z * v.x - w.x * v.z),
      -DRAG * s * v.z + MAGNUS * (w.x * v.y - w.y * v.x),
    );
  }
  function integrate(b, dt) {
    b.v = add(b.v, mul(accel(b.v, b.w), dt));
    b.p = add(b.p, mul(b.v, dt));
    b.w = mul(b.w, 1 - SPIN_DECAY * dt);
  }

  // Simula até a bola cruzar o plano z = zPlane; devolve {x, y, t} ou null.
  function crossPlane(p, v, w, zPlane, bounce, maxT = 3) {
    const b = { p: { ...p }, v: { ...v }, w: { ...w } };
    let t = 0;
    while (t < maxT) {
      const prev = b.p;
      integrate(b, DT); t += DT;
      if (bounce && b.p.y < R) { b.p.y = R; if (b.v.y < 0) b.v.y = -b.v.y * 0.5; b.v.x *= 0.9; b.v.z *= 0.9; }
      if (b.p.z <= zPlane) {
        const k = (prev.z - zPlane) / (prev.z - b.p.z);
        return { x: lerp(prev.x, b.p.x, k), y: lerp(prev.y, b.p.y, k), t: t - DT * (1 - k) };
      }
      if (b.v.z >= 0) return null;
    }
    return null;
  }

  function launchVel(speed, yaw, pitch) {
    return v3(speed * Math.cos(pitch) * Math.sin(yaw), speed * Math.sin(pitch), -speed * Math.cos(pitch) * Math.cos(yaw));
  }

  // Acha direção (yaw/pitch) para que a bola, com essa velocidade e efeito, passe pelo alvo.
  function solveLaunch(B, T, speed, wy) {
    const D = Math.hypot(T.x - B.x, B.z - T.z);
    let yaw = Math.atan2(T.x - B.x, B.z - T.z);
    let pitch = Math.atan2(T.y - B.y, D) + 0.05;
    const w = v3(0, wy, 0);
    for (let i = 0; i < 18; i++) {
      const c = crossPlane(B, launchVel(speed, yaw, pitch), w, T.z, false);
      if (!c) { pitch = Math.min(0.8, pitch + 0.08); continue; }
      const ex = T.x - c.x, ey = T.y - c.y;
      if (Math.abs(ex) < 0.004 && Math.abs(ey) < 0.004) break;
      yaw += ex / D;
      pitch = clamp(pitch + (ey / D) * 1.1, -0.1, 0.8);
    }
    return { yaw, pitch };
  }

  // ---------- Cenário da cobrança ----------
  function makeKick(index, rnd = Math.random) {
    if (index > 0 && rnd() < 0.16) return { type: 'penalty', ball: { x: 0, z: 11 }, wall: 0, dist: 11 };
    const z = 17.6 + rnd() * (7 + index * 1.6);
    const maxX = Math.min(17, 5 + (z - 17) * 0.9);
    const x = (rnd() * 2 - 1) * maxX;
    const dist = Math.hypot(x, z);
    let wall = dist < 21 ? 5 : dist < 25 ? 4 : dist < 30 ? 3 : 0;
    if (rnd() < 0.18) wall = 0;
    return { type: 'free', ball: { x, z }, wall, dist };
  }

  function buildWall(kick, rnd = Math.random) {
    if (!kick.wall) return { players: [], side: 0 };
    const B = kick.ball;
    const hl = Math.hypot(B.x, B.z);
    const h = { x: -B.x / hl, z: -B.z / hl };
    const perp = { x: -h.z, z: h.x };
    const side = Math.abs(B.x) < 1.5 ? (rnd() < 0.5 ? -1 : 1) : Math.sign(B.x);
    const post = { x: side * GOAL_HALF, z: 0 };
    const toPost = { x: post.x - B.x, z: post.z - B.z };
    const k = WALL_DIST / (toPost.x * h.x + toPost.z * h.z);
    const Q = { x: B.x + toPost.x * k, z: B.z + toPost.z * k };       // onde a linha bola→trave cruza a barreira
    const outSign = Math.sign(perp.x * post.x + perp.z * post.z) || 1;
    const out = { x: perp.x * outSign, z: perp.z * outSign };
    const players = [];
    for (let i = 0; i < kick.wall; i++) {
      const off = 0.32 - i * 0.58;
      players.push({ x: Q.x + out.x * off, z: Q.z + out.z * off, j0: 0.1 + rnd() * 0.08, jh: 0.26 + rnd() * 0.1 });
    }
    return { players, side, line: { Q, out } };
  }

  function wallJump(pl, t) {
    const s = (t - pl.j0) / 0.55;
    return s > 0 && s < 1 ? pl.jh * Math.sin(Math.PI * s) : 0;
  }

  // ---------- Goleiro ----------
  function keeperIdle(x0) {
    return { x0, z: KEEPER_Z, react: Infinity, dur: 1, feetEnd: { x: x0, y: 0.05 }, theta: 0, len: 1.95 };
  }

  function keeperPlan(kick, x0, launch, diff, rnd = Math.random) {
    const K = keeperIdle(x0);
    const pred = crossPlane(launch.p, launch.v, launch.w, KEEPER_Z, true);
    if (!pred) return K;
    const speed = len(launch.v);
    const onTarget = Math.abs(pred.x) < GOAL_HALF + 0.5 && pred.y < GOAL_H + 0.4;
    let react = diff.react + (kick.wall ? 0.1 : 0) + rnd() * 0.06;
    let tx, ty;
    if (!onTarget) {
      tx = x0 + clamp(pred.x - x0, -0.7, 0.7); ty = 1.4;
    } else if (pred.t - react < 0.32) {
      // Sem tempo de reagir (pênalti): o goleiro precisa adivinhar o canto.
      react = 0.03 + rnd() * 0.05;
      if (rnd() < diff.guess) { tx = pred.x + gauss(rnd) * 0.4; ty = pred.y + gauss(rnd) * 0.35; }
      else { const r = rnd(); tx = r < 0.2 ? x0 : (r < 0.6 ? -1 : 1) * (1.5 + rnd() * 1.8); ty = 0.3 + rnd() * 1.7; }
    } else {
      const sigma = 0.18 + 0.4 * Math.abs(launch.w.y) / MAX_SPIN + (kick.wall ? 0.12 : 0) + 0.012 * Math.max(0, speed - 20);
      tx = pred.x + gauss(rnd) * sigma;
      ty = pred.y + gauss(rnd) * sigma * 0.6;
    }
    tx = clamp(tx, -4.3, 4.3); ty = clamp(ty, 0.15, 2.7);
    const dx = tx - x0, sgn = Math.sign(dx) || 1;
    const feetEnd = { x: x0 + sgn * Math.min(1.0, Math.abs(dx) * 0.45), y: 0.05 + clamp(ty - 1.6, 0, 0.5) };
    const vx = tx - feetEnd.x, vy = Math.max(0.05, ty - feetEnd.y);
    K.theta = Math.atan2(vx, vy);
    K.len = clamp(Math.hypot(vx, vy), 1.35, KEEPER_LEN);
    K.feetEnd = feetEnd;
    K.react = react;
    K.dur = 0.3 + 0.12 * Math.abs(dx);
    K.target = { x: tx, y: ty };
    return K;
  }

  function keeperPose(K, t) {
    const s = clamp((t - K.react) / K.dur, 0, 1);
    const e = 1 - Math.pow(1 - s, 3);
    const fx = lerp(K.x0, K.feetEnd.x, e);
    const fy = lerp(0.05, K.feetEnd.y, e) + (Math.abs(K.theta) > 0.5 ? 0.35 * Math.sin(Math.PI * e) * (1 - e * 0.5) : 0);
    const theta = K.theta * e;
    const L = lerp(1.95, K.len, e);
    return {
      feet: v3(fx, fy, K.z),
      hands: v3(fx + Math.sin(theta) * L, fy + Math.cos(theta) * L, K.z),
      theta, len: L, armsUp: Math.min(1, e * 1.6), s,
    };
  }

  // ---------- Colisões ----------
  function closestOnSeg(p, a, b) {
    const ab = sub(b, a);
    const t = clamp(dot(sub(p, a), ab) / (dot(ab, ab) || 1), 0, 1);
    return add(a, mul(ab, t));
  }
  function collideCapsule(ball, a, b, rad, e, fr) {
    const c = closestOnSeg(ball.p, a, b);
    const d = sub(ball.p, c);
    const dist = len(d), min = R + rad;
    if (dist >= min || dist === 0) return false;
    const n = mul(d, 1 / dist);
    ball.p = add(c, mul(n, min));
    const vn = dot(ball.v, n);
    if (vn < 0) {
      ball.v = mul(sub(ball.v, mul(n, (1 + e) * vn)), fr);
      ball.w = mul(ball.w, 0.4);
    }
    return true;
  }

  // ---------- Voo da bola com tudo (barreira, goleiro, traves, rede) ----------
  function createShot(kick, wall, keeper, launch) {
    return {
      p: { ...launch.p }, v: { ...launch.v }, w: { ...launch.w }, t: 0,
      startZ: launch.p.z, kick, wall, keeper,
      flags: { wall: false, keeper: false, post: false },
      crossed: false, inGoal: false, caught: false, outcome: null, events: [], maxSpeed: len(launch.v),
    };
  }

  function setOutcome(S, o) { if (!S.outcome) { S.outcome = o; S.events.push({ type: 'outcome', outcome: o }); } }
  function fallback(S) { return S.flags.keeper ? 'save' : S.flags.wall ? 'wall' : S.flags.post ? 'post' : 'short'; }

  function stepShot(S, dt) {
    S.t += dt;
    if (S.caught) { const pose = keeperPose(S.keeper, S.t); S.p = add(pose.hands, v3(0, -0.12, 0.12)); return; }
    integrate(S, dt);

    // chão
    if (S.p.y < R) {
      S.p.y = R;
      if (S.v.y < 0) {
        if (S.v.y < -1.5) S.events.push({ type: 'bounce', p: { ...S.p } });
        S.v.y = -S.v.y * 0.5; S.v.x *= 0.88; S.v.z *= 0.88; S.w = mul(S.w, 0.7);
      }
      if (Math.abs(S.v.y) < 0.4) { const f = 1 - 1.1 * dt; S.v.x *= f; S.v.z *= f; }
    }

    // barreira
    for (const pl of S.wall) {
      const j = wallJump(pl, S.t);
      if (collideCapsule(S, v3(pl.x, j + 0.12, pl.z), v3(pl.x, j + 1.66, pl.z), WALL_R, 0.35, 0.7)) {
        if (!S.flags.wall) S.events.push({ type: 'wall', p: { ...S.p } });
        S.flags.wall = true;
      }
    }

    // goleiro
    const pose = keeperPose(S.keeper, S.t);
    const speedNow = len(S.v);
    let hitKeeper = collideCapsule(S, pose.feet, pose.hands, KEEPER_R, 0.3, 0.75);
    if (!hitKeeper && pose.s < 0.3) {
      hitKeeper = collideCapsule(S, v3(pose.feet.x - 0.55, 1.25, KEEPER_Z), v3(pose.feet.x + 0.55, 1.25, KEEPER_Z), 0.12, 0.3, 0.75);
    }
    if (hitKeeper) {
      if (!S.flags.keeper) {
        S.events.push({ type: 'keeper', p: { ...S.p } });
        const nearHands = len(sub(S.p, pose.hands)) < 0.45;
        if (!S.outcome && nearHands && speedNow < 17.5 && !S.crossed) {
          S.caught = true; S.v = v3(); S.w = v3();
          setOutcome(S, 'catch');
        }
      }
      S.flags.keeper = true;
    }

    // traves e travessão
    const posts = [
      [v3(-POST_X, 0, 0), v3(-POST_X, BAR_Y, 0)],
      [v3(POST_X, 0, 0), v3(POST_X, BAR_Y, 0)],
      [v3(-POST_X, BAR_Y, 0), v3(POST_X, BAR_Y, 0)],
    ];
    for (const [a, b] of posts) {
      if (collideCapsule(S, a, b, POST_R, 0.7, 0.9)) {
        if (!S.flags.post || S.events.at(-1)?.type !== 'post') S.events.push({ type: 'post', p: { ...S.p } });
        S.flags.post = true;
      }
    }

    // linha do gol
    if (!S.crossed && S.p.z < -R) {
      S.crossed = true;
      if (Math.abs(S.p.x) < GOAL_HALF && S.p.y < GOAL_H) { S.inGoal = true; setOutcome(S, 'goal'); }
      else setOutcome(S, S.flags.keeper ? 'save' : S.flags.wall ? 'wall' : S.flags.post ? 'post' : (S.p.y >= GOAL_H ? 'over' : 'wide'));
    }

    // rede (só se entrou)
    if (S.inGoal) {
      if (S.p.z < -NET_DEPTH + R) {
        if (S.v.z < 0) S.events.push({ type: 'net', p: { ...S.p }, force: -S.v.z });
        S.p.z = -NET_DEPTH + R; S.v.z = Math.abs(S.v.z) * 0.08; S.v.x *= 0.4; S.v.y *= 0.4; S.w = v3();
      }
      if (Math.abs(S.p.x) > GOAL_HALF - R) { S.p.x = Math.sign(S.p.x) * (GOAL_HALF - R); S.v.x *= -0.15; S.v.z *= 0.6; }
      if (S.p.y > GOAL_H - R) { S.p.y = GOAL_H - R; S.v.y = -Math.abs(S.v.y) * 0.15; }
    }

    // sem desfecho ainda: parou, voltou ou demorou demais
    if (!S.outcome) {
      const stopped = len(S.v) < 0.6 && S.p.y <= R + 0.01;
      if (S.t > 4 || stopped || S.p.z > S.startZ + 3 || (S.flags.keeper && S.v.z > 0 && S.t > 0.4)) setOutcome(S, fallback(S));
    }
  }

  return {
    R, GOAL_HALF, GOAL_H, POST_R, POST_X, BAR_Y, NET_DEPTH, KEEPER_Z, KEEPER_LEN, WALL_DIST, MAX_SPIN, DT,
    v3, add, sub, mul, dot, len, clamp, lerp, gauss,
    crossPlane, launchVel, solveLaunch, makeKick, buildWall, wallJump,
    keeperIdle, keeperPlan, keeperPose, createShot, stepShot,
  };
})();

if (typeof module !== 'undefined' && module.exports) module.exports = Physics;

/* =====================================================================
 *  JOGO — renderização em canvas, entrada por deslize, áudio e HUD
 * ===================================================================== */
if (typeof window !== 'undefined') (() => {
  const P = Physics;
  const { v3, clamp, lerp } = P;

  const KICKS = 5;
  const TEAMS = {
    Brasil:    { kit: '#f2cf3a', trim: '#1a8f4e', sock: '#f4f4f4', flag: 'radial-gradient(circle,#1e3f96 0 19%,transparent 20%),radial-gradient(ellipse 46% 40%,#fedf00 0 97%,transparent 100%),#009c3b' },
    Argentina: { kit: '#8fd0ea', trim: '#1b1f3a', sock: '#f4f4f4', flag: 'radial-gradient(circle,#f6b40e 0 11%,transparent 12%),linear-gradient(#74acdf 0 33%,#fff 33% 67%,#74acdf 67%)' },
    'França':  { kit: '#2546a0', trim: '#f4f0e6', sock: '#c8283a', flag: 'linear-gradient(90deg,#0055a4 0 33%,#fff 33% 67%,#ef4135 67%)' },
    Portugal:  { kit: '#c8343a', trim: '#1f7a46', sock: '#c8343a', flag: 'radial-gradient(circle at 40% 50%,#ffd200 0 16%,transparent 17%),linear-gradient(90deg,#046a38 0 40%,#da291c 40%)' },
    'Japão':   { kit: '#1f3b8f', trim: '#f3f0e8', sock: '#1f3b8f', flag: 'radial-gradient(circle,#bc002d 0 27%,#fff 28%)' },
    Alemanha:  { kit: '#f3f1ea', trim: '#1b1b1b', sock: '#f3f1ea', flag: 'linear-gradient(#000 0 33%,#dd0000 33% 67%,#ffce00 67%)' },
  };
  const KEEPER_KIT = { kit: '#7d3cff', trim: '#1a1033', sock: '#7d3cff', glove: '#d6ff3d' };

  // ---------- DOM ----------
  const $ = (s) => document.querySelector(s);
  const canvas = $('#game');
  const ctx = canvas.getContext('2d');
  const ui = {
    intro: $('#intro'), summary: $('#summary'), hud: $('#hud'),
    flag: $('#hudFlag'), team: $('#hudTeam'), kickNo: $('#kickNo'), kickDesc: $('#kickDesc'),
    goals: $('#hudGoals'), points: $('#hudPoints'), dots: $('#dots'),
    banner: $('#banner'), bannerTitle: $('#bannerTitle'), bannerSub: $('#bannerSub'),
    hint: $('#hint'), stats: $('#shotStats'), mute: $('#muteBtn'),
  };

  const store = {
    get(k, d) { try { const v = localStorage.getItem(k); return v === null ? d : v; } catch { return d; } },
    set(k, v) { try { localStorage.setItem(k, v); } catch { /* ignora */ } },
  };

  // ---------- Estado ----------
  const S = {
    phase: 'intro',            // intro | aim | flight | result | summary
    team: 'Brasil', rival: 'Argentina',
    index: 0, results: [], points: 0, best: Number(store.get('cng.best', 0)) || 0,
    kick: null, wall: { players: [] }, keeper: null, shot: null,
    swipe: null, lastSwipe: null,
    time: 0, phaseTime: 0, timeScale: 1, shake: 0, flash: 0,
    trail: [], particles: [], netHit: null, ballSpin: 0,
    hintShown: true, muted: store.get('cng.muted', '0') === '1',
  };

  // ---------- Câmera / projeção ----------
  const cam = { pos: v3(), look: v3(), f: 1, cx: 0, cy: 0, r: v3(), u: v3(), fw: v3(), base: null };
  let W = 0, H = 0, DPR = 1;
  const NEAR = 0.3;

  function setCamera(pos, look) {
    cam.pos = pos; cam.look = look;
    const d = P.sub(look, pos), l = P.len(d);
    cam.fw = P.mul(d, 1 / l);
    const r = v3(-cam.fw.z, 0, cam.fw.x);              // fw × up
    const rl = Math.hypot(r.x, r.z) || 1;
    cam.r = v3(r.x / rl, 0, r.z / rl);
    cam.u = v3(cam.r.y * cam.fw.z - cam.r.z * cam.fw.y, cam.r.z * cam.fw.x - cam.r.x * cam.fw.z, cam.r.x * cam.fw.y - cam.r.y * cam.fw.x);
  }
  function toCam(p) {
    const d = P.sub(p, cam.pos);
    return { x: P.dot(d, cam.r), y: P.dot(d, cam.u), z: P.dot(d, cam.fw) };
  }
  function projC(c) { return { x: cam.cx + cam.f * c.x / c.z, y: cam.cy - cam.f * c.y / c.z, s: cam.f / c.z, z: c.z }; }
  function proj(p) { const c = toCam(p); return c.z < NEAR ? null : projC(c); }
  function unproject(sx, sy, zPlane) {
    const dir = P.add(P.add(P.mul(cam.fw, cam.f), P.mul(cam.r, sx - cam.cx)), P.mul(cam.u, -(sy - cam.cy)));
    if (dir.z >= -1e-6) return null;
    const t = (zPlane - cam.pos.z) / dir.z;
    return P.add(cam.pos, P.mul(dir, t));
  }

  function cameraFor(kick) {
    const B = kick.ball, hl = Math.hypot(B.x, B.z);
    const h = { x: -B.x / hl, z: -B.z / hl };
    const back = kick.type === 'penalty' ? 4.6 : 5.2;
    return {
      pos: v3(B.x - h.x * back, 1.75, B.z - h.z * back),
      look: v3(0, 1.0, 0),
      intro: v3(B.x - h.x * (back + 4), 3.2, B.z - h.z * (back + 4)),
    };
  }

  function layoutCamera() {
    cam.f = Math.min(H * 1.7, W * 1.9);
    cam.cx = W / 2;
    if (!S.kick) { cam.cy = H * 0.35; return; }
    const base = cameraFor(S.kick);
    setCamera(base.pos, base.look);
    cam.cy = 0;
    const c = toCam(v3(S.kick.ball.x, P.R, S.kick.ball.z));
    cam.cy = H * 0.8 + cam.f * c.y / c.z;          // bola sempre a ~80% da altura da tela
    cam.base = base;
  }

  function resize() {
    DPR = Math.min(window.devicePixelRatio || 1, 2);
    const w = canvas.clientWidth, h = canvas.clientHeight;
    W = Math.round(w * DPR); H = Math.round(h * DPR);
    canvas.width = W; canvas.height = H;
    layoutCamera();
  }

  // ---------- Texturas geradas ----------
  function makeCrowd() {
    const c = document.createElement('canvas'); c.width = 1600; c.height = 360;
    const g = c.getContext('2d');
    const grd = g.createLinearGradient(0, 0, 0, c.height);
    grd.addColorStop(0, '#0b0f1a'); grd.addColorStop(1, '#1b2232');
    g.fillStyle = grd; g.fillRect(0, 0, c.width, c.height);
    const shirts = ['#e8c547', '#2a9d5c', '#d9dde3', '#3a6fd8', '#c0392b', '#f39c12', '#8e44ad', '#ecf0f1', '#16a085'];
    for (let row = 0; row < 26; row++) {
      const y = 12 + row * 13.5;
      g.fillStyle = row % 2 ? '#141a27' : '#181f2e';
      g.fillRect(0, y + 8, c.width, 5);
      for (let x = (row % 2) * 4; x < c.width; x += 8 + Math.random() * 3) {
        if (Math.random() < 0.08) continue;
        const light = 0.55 + row / 26 * 0.45;
        g.globalAlpha = light;
        g.fillStyle = shirts[(Math.random() * shirts.length) | 0];
        g.fillRect(x, y + 3, 6, 7);
        g.fillStyle = ['#e0b48f', '#8d5a3b', '#c68c66', '#5b3a29'][(Math.random() * 4) | 0];
        g.beginPath(); g.arc(x + 3, y + 1.5, 2.6, 0, Math.PI * 2); g.fill();
      }
    }
    g.globalAlpha = 1;
    return c;
  }
  function makeBoards() {
    const c = document.createElement('canvas'); c.width = 2400; c.height = 60;
    const g = c.getContext('2d');
    const ads = [['CHUTE NO GOL', '#0d1b3d', '#5ee0ff'], ['FUTEBOL ARCADE', '#1b0d33', '#d6ff3d'], ['GOLAÇO', '#3d0d16', '#ffd166'], ['NO ÂNGULO', '#0d3324', '#7dffb2']];
    const seg = c.width / 8;
    for (let i = 0; i < 8; i++) {
      const [txt, bg, fg] = ads[i % ads.length];
      g.fillStyle = bg; g.fillRect(i * seg, 0, seg, c.height);
      g.fillStyle = fg; g.font = '900 italic 34px "Segoe UI", system-ui, sans-serif';
      g.textAlign = 'center'; g.textBaseline = 'middle';
      g.fillText(txt, i * seg + seg / 2, c.height / 2 + 2);
      g.fillStyle = '#ffffff22'; g.fillRect(i * seg, 0, 2, c.height);
    }
    return c;
  }
  const TEX = { crowd: null, boards: null };

  // ---------- Desenho: primitivas 3D ----------
  function polyScreen(pts) {
    const cs = pts.map(toCam), out = [];
    for (let i = 0; i < cs.length; i++) {
      const a = cs[i], b = cs[(i + 1) % cs.length];
      const ain = a.z >= NEAR, bin = b.z >= NEAR;
      if (ain) out.push(a);
      if (ain !== bin) {
        const t = (NEAR - a.z) / (b.z - a.z);
        out.push({ x: lerp(a.x, b.x, t), y: lerp(a.y, b.y, t), z: NEAR });
      }
    }
    return out.map(projC);
  }
  function fillPoly(pts, style) {
    const s = polyScreen(pts);
    if (s.length < 3) return;
    ctx.beginPath(); ctx.moveTo(s[0].x, s[0].y);
    for (let i = 1; i < s.length; i++) ctx.lineTo(s[i].x, s[i].y);
    ctx.closePath(); ctx.fillStyle = style; ctx.fill();
  }
  function groundLine(x1, z1, x2, z2, w = 0.12, style = 'rgba(255,255,255,.82)') {
    const dx = x2 - x1, dz = z2 - z1, l = Math.hypot(dx, dz) || 1;
    const nx = -dz / l * w / 2, nz = dx / l * w / 2;
    fillPoly([v3(x1 + nx, 0, z1 + nz), v3(x2 + nx, 0, z2 + nz), v3(x2 - nx, 0, z2 - nz), v3(x1 - nx, 0, z1 - nz)], style);
  }
  function groundArc(cx, cz, r, a0, a1, w = 0.12) {
    const n = 24;
    for (let i = 0; i < n; i++) {
      const t0 = lerp(a0, a1, i / n), t1 = lerp(a0, a1, (i + 1) / n);
      groundLine(cx + Math.sin(t0) * r, cz + Math.cos(t0) * r, cx + Math.sin(t1) * r, cz + Math.cos(t1) * r, w);
    }
  }
  function line3(a, b, width, style) {
    const ca = toCam(a), cb = toCam(b);
    if (ca.z < NEAR && cb.z < NEAR) return;
    let A = ca, B = cb;
    if (A.z < NEAR) { const t = (NEAR - A.z) / (B.z - A.z); A = { x: lerp(A.x, B.x, t), y: lerp(A.y, B.y, t), z: NEAR }; }
    if (B.z < NEAR) { const t = (NEAR - B.z) / (A.z - B.z); B = { x: lerp(B.x, A.x, t), y: lerp(B.y, A.y, t), z: NEAR }; }
    const pa = projC(A), pb = projC(B);
    ctx.beginPath(); ctx.moveTo(pa.x, pa.y); ctx.lineTo(pb.x, pb.y);
    ctx.lineWidth = typeof width === 'function' ? width((pa.s + pb.s) / 2) : width;
    ctx.strokeStyle = style; ctx.stroke();
  }
  // Mapeia uma textura sobre um quadrilátero vertical, em faixas (aproximação afim).
  function texturedWall(img, x0, x1, bottom, top, strips, sx0 = 0, sx1 = img.width) {
    for (let i = 0; i < strips; i++) {
      const ta = i / strips, tb = (i + 1) / strips;
      const bl = proj(v3(lerp(x0, x1, ta), bottom.y, bottom.z));
      const br = proj(v3(lerp(x0, x1, tb), bottom.y, bottom.z));
      const tl = proj(v3(lerp(x0, x1, ta), top.y, top.z));
      if (!bl || !br || !tl) continue;
      const srcX = lerp(sx0, sx1, ta), srcW = (sx1 - sx0) / strips;
      ctx.save();
      ctx.setTransform((br.x - bl.x) / srcW, (br.y - bl.y) / srcW, (tl.x - bl.x) / -img.height, (tl.y - bl.y) / -img.height, bl.x, bl.y);
      ctx.drawImage(img, srcX, 0, srcW + 0.6, img.height, 0, -img.height, srcW + 0.6, img.height);
      ctx.restore();
    }
  }

  // ---------- Cena ----------
  function drawSky() {
    const g = ctx.createLinearGradient(0, 0, 0, H);
    g.addColorStop(0, '#04060c'); g.addColorStop(0.45, '#0e1626'); g.addColorStop(1, '#13202a');
    ctx.fillStyle = g; ctx.fillRect(0, 0, W, H);
  }
  function drawStadium() {
    // arquibancada atrás do gol (inclinada para trás)
    texturedWall(TEX.crowd, -95, 95, v3(0, 0.9, -9), v3(0, 19, -34), 40);
    // marquise
    fillPoly([v3(-95, 19, -34), v3(95, 19, -34), v3(95, 21.5, -30), v3(-95, 21.5, -30)], '#0a0d14');
    // refletores
    for (const x of [-48, -16, 16, 48]) {
      const p = proj(v3(x, 23, -33));
      if (!p) continue;
      const r = Math.max(12, p.s * 7);
      const g = ctx.createRadialGradient(p.x, p.y, 0, p.x, p.y, r * 3);
      g.addColorStop(0, 'rgba(255,250,225,.95)'); g.addColorStop(0.15, 'rgba(255,245,210,.45)'); g.addColorStop(1, 'rgba(255,240,200,0)');
      ctx.fillStyle = g; ctx.fillRect(p.x - r * 3, p.y - r * 3, r * 6, r * 6);
    }
    // flashes de câmera na torcida
    for (let i = 0; i < 4; i++) {
      if (Math.random() > 0.35) continue;
      const p = proj(v3((Math.random() * 2 - 1) * 60, 2 + Math.random() * 14, -10 - Math.random() * 20));
      if (!p) continue;
      ctx.fillStyle = 'rgba(255,255,255,.9)';
      ctx.beginPath(); ctx.arc(p.x, p.y, Math.max(1, p.s * 0.15), 0, Math.PI * 2); ctx.fill();
    }
  }
  function drawPitch() {
    // gramado com faixas de corte
    fillPoly([v3(-120, 0, -9), v3(120, 0, -9), v3(120, 0, 90), v3(-120, 0, 90)], '#1f5a33');
    for (let i = 0, z = -9; z < 90; z += 4.5, i++) {
      if (i % 2) fillPoly([v3(-120, 0, z), v3(120, 0, z), v3(120, 0, z + 4.5), v3(-120, 0, z + 4.5)], '#256a3c');
    }
    // linhas
    groundLine(-34, 0, 34, 0);
    groundLine(-34, 0, -34, 60); groundLine(34, 0, 34, 60);
    groundLine(-9.16, 0, -9.16, 5.5); groundLine(9.16, 0, 9.16, 5.5); groundLine(-9.16, 5.5, 9.16, 5.5);
    groundLine(-20.16, 0, -20.16, 16.5); groundLine(20.16, 0, 20.16, 16.5); groundLine(-20.16, 16.5, 20.16, 16.5);
    groundArc(0, 11, 9.15, -0.925, 0.925);
    fillPoly(circlePts(0, 11, 0.12), 'rgba(255,255,255,.85)');
    // spray: marca da bola e linha da barreira
    if (S.kick) {
      const B = S.kick.ball;
      for (let i = 0; i < 14; i++) { const a = i / 14 * Math.PI * 2; fillPoly(circlePts(B.x + Math.cos(a) * 0.32, B.z + Math.sin(a) * 0.32, 0.035), 'rgba(255,255,255,.75)'); }
      const line = S.wall.line;
      if (line && S.wall.players.length) {
        const n = S.wall.players.length;
        for (let i = -3; i < n * 2 + 3; i++) {
          const off = 0.6 - i * 0.29;
          const x = line.Q.x + line.out.x * off, z = line.Q.z + line.out.z * off;
          const hl = Math.hypot(B.x - x, B.z - z);
          fillPoly(circlePts(x + (B.x - x) / hl * 0.35, z + (B.z - z) / hl * 0.35, 0.05), 'rgba(255,255,255,.7)');
        }
      }
    }
    // placas de publicidade
    texturedWall(TEX.boards, -40, 40, v3(0, 0, -5), v3(0, 0.95, -5), 32);
  }
  function circlePts(x, z, r, n = 10) {
    const pts = [];
    for (let i = 0; i < n; i++) { const a = i / n * Math.PI * 2; pts.push(v3(x + Math.cos(a) * r, 0, z + Math.sin(a) * r)); }
    return pts;
  }

  // ----- Gol e rede -----
  function netOffset(x, y) {
    const n = S.netHit;
    if (!n) return 0;
    const age = S.time - n.t;
    if (age > 1.6) return 0;
    const d2 = (x - n.x) ** 2 + (y - n.y) ** 2;
    return n.force * 0.045 * Math.exp(-d2 / 0.9) * Math.exp(-age * 2.6) * Math.cos(age * 14);
  }
  function drawNet() {
    const X = P.POST_X, Hh = P.BAR_Y, D = P.NET_DEPTH, back = -D, topBack = 2.1;
    ctx.lineCap = 'round';
    const style = 'rgba(235,240,245,.32)';
    const lw = (s) => Math.max(0.6 * DPR, s * 0.018);
    const backY = (y) => y * topBack / Hh;
    const step = 0.3;
    // fundo
    for (let x = -X; x <= X + 1e-6; x += step) {
      let prev = null;
      for (let y = 0; y <= Hh + 1e-6; y += step) {
        const p = v3(x, backY(y), back - netOffset(x, y));
        if (prev) line3(prev, p, lw, style);
        prev = p;
      }
    }
    for (let y = 0; y <= Hh + 1e-6; y += step) {
      let prev = null;
      for (let x = -X; x <= X + 1e-6; x += step) {
        const p = v3(x, backY(y), back - netOffset(x, y));
        if (prev) line3(prev, p, lw, style);
        prev = p;
      }
    }
    // laterais e teto
    for (let z = 0; z >= back - 1e-6; z -= step) {
      const t = -z / D, yTop = lerp(Hh, topBack, t);
      line3(v3(-X, 0, z), v3(-X, yTop, z), lw, style);
      line3(v3(X, 0, z), v3(X, yTop, z), lw, style);
      line3(v3(-X, yTop, z), v3(X, yTop, z), lw, style);
    }
    for (let y = 0; y <= Hh; y += step) {
      line3(v3(-X, y, 0), v3(-X, backY(y), back), lw, style);
      line3(v3(X, y, 0), v3(X, backY(y), back), lw, style);
    }
    for (let x = -X; x <= X + 1e-6; x += step) line3(v3(x, Hh, 0), v3(x, topBack, back), lw, style);
    // ferros de sustentação
    const bar = 'rgba(200,205,210,.7)';
    line3(v3(-X, 0, back), v3(-X, topBack, back), (s) => Math.max(1, s * 0.05), bar);
    line3(v3(X, 0, back), v3(X, topBack, back), (s) => Math.max(1, s * 0.05), bar);
  }
  function drawPosts() {
    const X = P.POST_X, Hh = P.BAR_Y;
    const w = (s) => Math.max(2 * DPR, s * 0.12);
    ctx.lineCap = 'round';
    const shake = S.postShake ? Math.sin(S.time * 70) * S.postShake * 0.02 : 0;
    const pts = [v3(-X, 0, 0), v3(-X + shake, Hh, 0), v3(X + shake, Hh, 0), v3(X, 0, 0)];
    // sombra/volume
    for (let i = 0; i < 3; i++) line3(pts[i], pts[i + 1], (s) => w(s) * 1.15, 'rgba(120,130,140,.9)');
    for (let i = 0; i < 3; i++) line3(pts[i], pts[i + 1], w, '#f7f8fa');
    // sombras das traves no chão
    ctx.globalAlpha = 0.25;
    groundLine(-X, 0, -X - 0.4, -1.4, 0.12, '#000'); groundLine(X, 0, X - 0.4, -1.4, 0.12, '#000');
    ctx.globalAlpha = 1;
  }

  // ----- Jogadores -----
  function drawShadow(x, z, w, alpha = 0.35) {
    const p = proj(v3(x, 0, z));
    if (!p) return;
    ctx.fillStyle = `rgba(0,0,0,${alpha})`;
    ctx.beginPath(); ctx.ellipse(p.x, p.y, w * p.s, w * p.s * 0.28, 0, 0, Math.PI * 2); ctx.fill();
  }
  function rr(x, y, w, h, r) { ctx.beginPath(); ctx.roundRect ? ctx.roundRect(x, y, w, h, r) : ctx.rect(x, y, w, h); }
  function shade(hex, amt) {
    const n = parseInt(hex.slice(1), 16);
    const f = (c) => clamp(Math.round(c + amt * 255), 0, 255);
    return `rgb(${f(n >> 16)},${f((n >> 8) & 255)},${f(n & 255)})`;
  }

  // Figura de frente; unidades em metros, origem nos pés, y para cima negativo.
  function drawFigure(kit, pose) {
    const skin = pose.skin || '#c68c66';
    // pernas
    ctx.fillStyle = kit.sock; rr(-0.17, -0.48, 0.13, 0.46, 0.05); ctx.fill(); rr(0.04, -0.48, 0.13, 0.46, 0.05); ctx.fill();
    ctx.fillStyle = skin; rr(-0.165, -0.85, 0.12, 0.4, 0.05); ctx.fill(); rr(0.045, -0.85, 0.12, 0.4, 0.05); ctx.fill();
    ctx.fillStyle = '#111'; rr(-0.2, -0.06, 0.18, 0.07, 0.03); ctx.fill(); rr(0.02, -0.06, 0.18, 0.07, 0.03); ctx.fill();
    // calção
    ctx.fillStyle = kit.trim; rr(-0.22, -1.06, 0.44, 0.26, 0.05); ctx.fill();
    // braços (atrás do tronco quando abertos)
    const armsUp = pose.armsUp || 0;
    const drawArm = (side) => {
      ctx.save();
      ctx.translate(side * 0.22, -1.44);
      const ang = pose.armsCross ? side * 0.35 : side * lerp(0.5, 2.9, armsUp);
      ctx.rotate(-ang);
      ctx.fillStyle = kit.kit; rr(-0.06, 0, 0.12, 0.22, 0.05); ctx.fill();
      ctx.fillStyle = skin; rr(-0.05, 0.18, 0.1, 0.3, 0.05); ctx.fill();
      if (pose.glove) { ctx.fillStyle = pose.glove; rr(-0.08, 0.42, 0.16, 0.17, 0.05); ctx.fill(); }
      ctx.restore();
    };
    if (!pose.armsCross) { drawArm(-1); drawArm(1); }
    // tronco
    const g = ctx.createLinearGradient(-0.25, 0, 0.25, 0);
    g.addColorStop(0, shade(kit.kit, -0.12)); g.addColorStop(0.5, kit.kit); g.addColorStop(1, shade(kit.kit, -0.2));
    ctx.fillStyle = g; rr(-0.25, -1.52, 0.5, 0.52, 0.09); ctx.fill();
    ctx.fillStyle = kit.trim; rr(-0.25, -1.52, 0.5, 0.05, 0.02); ctx.fill();
    if (pose.number) {
      ctx.fillStyle = kit.trim; ctx.font = '900 0.2px sans-serif'; ctx.textAlign = 'center'; ctx.textBaseline = 'middle';
      ctx.fillText(pose.number, 0, -1.27);
    }
    if (pose.armsCross) {
      ctx.fillStyle = skin;
      ctx.save(); ctx.translate(-0.2, -1.42); ctx.rotate(-0.25); rr(-0.06, 0, 0.12, 0.5, 0.05); ctx.fill(); ctx.restore();
      ctx.save(); ctx.translate(0.2, -1.42); ctx.rotate(0.25); rr(-0.06, 0, 0.12, 0.5, 0.05); ctx.fill(); ctx.restore();
      ctx.fillStyle = kit.kit; rr(-0.26, -1.48, 0.12, 0.18, 0.05); ctx.fill(); rr(0.14, -1.48, 0.12, 0.18, 0.05); ctx.fill();
    }
    // cabeça
    ctx.fillStyle = skin; ctx.beginPath(); ctx.arc(0, -1.66, 0.12, 0, Math.PI * 2); ctx.fill();
    ctx.fillStyle = pose.hair || '#2b1d16';
    ctx.beginPath(); ctx.arc(0, -1.69, 0.12, Math.PI * 1.05, Math.PI * 1.95); ctx.fill();
  }

  function drawWallPlayer(pl, i) {
    const t = S.shot ? S.shot.t : 0;
    const j = P.wallJump(pl, t);
    const p = proj(v3(pl.x, j, pl.z));
    if (!p) return;
    ctx.save(); ctx.translate(p.x, p.y); ctx.scale(p.s, p.s);
    drawFigure(TEAMS[S.rival], { armsCross: true, skin: ['#c68c66', '#8d5a3b', '#e0b48f', '#6b4430', '#d9a27c'][i % 5], hair: i % 3 ? '#2b1d16' : '#d8b26a' });
    ctx.restore();
  }

  function drawKeeper() {
    const pose = P.keeperPose(S.keeper, S.shot ? S.shot.t : -1);
    const sway = S.phase === 'aim' ? Math.sin(S.time * 3) * 0.06 : 0;
    const feet = P.add(pose.feet, v3(sway, 0, 0));
    const pf = proj(feet), ph = proj(P.add(feet, v3(Math.sin(pose.theta) * pose.len, Math.cos(pose.theta) * pose.len, 0)));
    if (!pf || !ph) return;
    const ang = Math.atan2(ph.x - pf.x, pf.y - ph.y);
    const scale = Math.hypot(ph.x - pf.x, ph.y - pf.y) / pose.len;
    ctx.save(); ctx.translate(pf.x, pf.y); ctx.rotate(ang); ctx.scale(scale, scale);
    drawFigure(KEEPER_KIT, { armsUp: pose.armsUp, glove: KEEPER_KIT.glove, number: '1', skin: '#b97a56' });
    ctx.restore();
  }

  // ----- Bola -----
  function drawBall(p3) {
    const p = proj(p3);
    if (!p) return;
    const r = Math.max(2, P.R * p.s);
    ctx.save();
    ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.clip();
    const g = ctx.createRadialGradient(p.x - r * 0.35, p.y - r * 0.4, r * 0.1, p.x, p.y, r);
    g.addColorStop(0, '#ffffff'); g.addColorStop(0.6, '#e4e7ea'); g.addColorStop(1, '#9aa3ab');
    ctx.fillStyle = g; ctx.fillRect(p.x - r, p.y - r, r * 2, r * 2);
    // gomos girando
    ctx.fillStyle = '#20262c';
    const rot = S.ballSpin;
    for (let i = 0; i < 6; i++) {
      const a = rot + i * Math.PI / 3, k = i === 0 ? 0 : 0.62;
      const cx = p.x + Math.cos(a) * r * k * Math.cos(rot * 0.7), cy = p.y + Math.sin(a) * r * k;
      pentagon(cx, cy, r * (i === 0 ? 0.3 : 0.22), a);
    }
    const hl = ctx.createRadialGradient(p.x - r * 0.4, p.y - r * 0.45, 0, p.x - r * 0.4, p.y - r * 0.45, r * 0.7);
    hl.addColorStop(0, 'rgba(255,255,255,.55)'); hl.addColorStop(1, 'rgba(255,255,255,0)');
    ctx.fillStyle = hl; ctx.fillRect(p.x - r, p.y - r, r * 2, r * 2);
    ctx.restore();
  }
  function pentagon(x, y, r, a) {
    ctx.beginPath();
    for (let i = 0; i < 5; i++) { const t = a + i * Math.PI * 2 / 5; ctx[i ? 'lineTo' : 'moveTo'](x + Math.cos(t) * r, y + Math.sin(t) * r); }
    ctx.closePath(); ctx.fill();
  }
  function ballPos() {
    if (S.shot) return S.shot.p;
    return v3(S.kick.ball.x, P.R, S.kick.ball.z);
  }
  function drawBallShadow(p3) {
    const alpha = clamp(0.45 - p3.y * 0.06, 0.08, 0.45);
    drawShadow(p3.x + p3.y * 0.15, p3.z - p3.y * 0.1, P.R * 1.2, alpha);
  }
  function drawTrail() {
    if (S.trail.length < 2) return;
    ctx.lineCap = 'round';
    for (let i = 1; i < S.trail.length; i++) {
      const a = proj(S.trail[i - 1]), b = proj(S.trail[i]);
      if (!a || !b) continue;
      const k = i / S.trail.length;
      ctx.strokeStyle = S.curveTrail ? `rgba(255,214,102,${k * 0.55})` : `rgba(255,255,255,${k * 0.45})`;
      ctx.lineWidth = Math.max(1, P.R * b.s * 1.4 * k);
      ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
    }
  }

  // ----- Mira (gesto) -----
  function drawSwipe() {
    const sw = S.swipe || S.lastSwipe;
    if (!sw || sw.points.length < 2) return;
    const fade = S.swipe ? 1 : clamp(1 - (S.time - sw.endTime) / 0.45, 0, 1);
    if (fade <= 0) return;
    const pts = sw.points;
    ctx.save();
    ctx.lineCap = 'round'; ctx.lineJoin = 'round';
    ctx.shadowColor = 'rgba(255,214,102,.9)'; ctx.shadowBlur = 18 * DPR;
    ctx.strokeStyle = `rgba(255,236,170,${0.9 * fade})`; ctx.lineWidth = 5 * DPR;
    ctx.beginPath(); ctx.moveTo(pts[0].x * DPR, pts[0].y * DPR);
    for (const p of pts) ctx.lineTo(p.x * DPR, p.y * DPR);
    ctx.stroke();
    ctx.restore();
    if (S.swipe) {
      const aim = readSwipe(S.swipe, false);
      if (aim && aim.target) {
        const p = proj(aim.target);
        if (p) {
          const ok = Math.abs(aim.target.x) < P.GOAL_HALF && aim.target.y < P.GOAL_H;
          ctx.strokeStyle = ok ? 'rgba(125,255,178,.95)' : 'rgba(255,120,110,.95)';
          ctx.lineWidth = 2 * DPR;
          const r = Math.max(8 * DPR, p.s * 0.28);
          ctx.beginPath(); ctx.arc(p.x, p.y, r, 0, Math.PI * 2); ctx.stroke();
          ctx.beginPath(); ctx.moveTo(p.x - r * 1.5, p.y); ctx.lineTo(p.x - r * 0.5, p.y); ctx.moveTo(p.x + r * 0.5, p.y); ctx.lineTo(p.x + r * 1.5, p.y);
          ctx.moveTo(p.x, p.y - r * 1.5); ctx.lineTo(p.x, p.y - r * 0.5); ctx.moveTo(p.x, p.y + r * 0.5); ctx.lineTo(p.x, p.y + r * 1.5); ctx.stroke();
        }
      }
    }
  }
  function drawBallHalo() {
    if (S.phase !== 'aim' || S.swipe) return;
    const p = proj(ballPos());
    if (!p) return;
    const r = P.R * p.s;
    const k = (Math.sin(S.time * 4) + 1) / 2;
    ctx.strokeStyle = `rgba(255,236,170,${0.35 + k * 0.4})`;
    ctx.lineWidth = 2 * DPR;
    ctx.beginPath(); ctx.arc(p.x, p.y, r * (1.5 + k * 0.4), 0, Math.PI * 2); ctx.stroke();
  }

  // ----- Partículas e pós-efeitos -----
  function burst(x, y, colors, n = 90) {
    for (let i = 0; i < n; i++) {
      const a = Math.random() * Math.PI * 2, sp = (2 + Math.random() * 9) * DPR;
      S.particles.push({ x, y, vx: Math.cos(a) * sp, vy: Math.sin(a) * sp - 6 * DPR, life: 1.6 + Math.random(), age: 0, c: colors[i % colors.length], r: (3 + Math.random() * 4) * DPR, rot: Math.random() * 6 });
    }
  }
  function drawParticles(dt) {
    for (const q of S.particles) {
      q.age += dt; q.vy += 18 * DPR * dt; q.vx *= 0.99;
      q.x += q.vx * dt * 60; q.y += q.vy * dt * 60 * 0.5; q.rot += dt * 8;
      ctx.save(); ctx.globalAlpha = clamp(1 - q.age / q.life, 0, 1);
      ctx.translate(q.x, q.y); ctx.rotate(q.rot); ctx.fillStyle = q.c;
      ctx.fillRect(-q.r, -q.r * 0.5, q.r * 2, q.r); ctx.restore();
    }
    S.particles = S.particles.filter((q) => q.age < q.life);
  }
  function drawPost() {
    const v = ctx.createRadialGradient(W / 2, H * 0.55, Math.min(W, H) * 0.35, W / 2, H * 0.55, Math.max(W, H) * 0.8);
    v.addColorStop(0, 'rgba(0,0,0,0)'); v.addColorStop(1, 'rgba(0,0,0,.55)');
    ctx.fillStyle = v; ctx.fillRect(0, 0, W, H);
    if (S.flash > 0) { ctx.fillStyle = `rgba(255,255,255,${S.flash * 0.35})`; ctx.fillRect(0, 0, W, H); }
    if (S.fade > 0) { ctx.fillStyle = `rgba(4,6,12,${S.fade})`; ctx.fillRect(0, 0, W, H); }
  }

  // ---------- Render ----------
  function render(dt) {
    ctx.setTransform(1, 0, 0, 1, 0, 0);
    const sh = S.shake * DPR * 6;
    if (sh > 0.1) ctx.translate((Math.random() - 0.5) * sh, (Math.random() - 0.5) * sh);
    drawSky();
    if (!S.kick) { drawPost(); return; }
    drawStadium();
    drawPitch();

    const bp = ballPos();
    // sombras no chão
    for (const pl of S.wall.players) drawShadow(pl.x, pl.z, 0.32);
    const kp = P.keeperPose(S.keeper, S.shot ? S.shot.t : -1);
    drawShadow((kp.feet.x + kp.hands.x) / 2, P.KEEPER_Z, 0.5 + Math.abs(kp.hands.x - kp.feet.x) * 0.4, 0.3);
    drawBallShadow(bp);

    // objetos ordenados por profundidade (pintor)
    const items = [
      { d: toCam(v3(0, 1, -1.2)).z, fn: drawNet },
      { d: toCam(v3(0, 1, 0)).z, fn: drawPosts },
      { d: toCam(v3(kp.feet.x, 1, P.KEEPER_Z)).z, fn: drawKeeper },
      { d: toCam(bp).z, fn: () => { drawTrail(); drawBall(bp); } },
    ];
    S.wall.players.forEach((pl, i) => items.push({ d: toCam(v3(pl.x, 1, pl.z)).z, fn: () => drawWallPlayer(pl, i) }));
    items.sort((a, b) => b.d - a.d).forEach((it) => it.fn());

    drawBallHalo();
    drawSwipe();
    drawParticles(dt);
    drawPost();
  }

  // ---------- Áudio (sintetizado) ----------
  let AC = null, crowdGain = null;
  function audio() {
    if (S.muted) return null;
    if (!AC) {
      try { AC = new (window.AudioContext || window.webkitAudioContext)(); } catch { return null; }
      // torcida ambiente: ruído filtrado em loop
      const buf = AC.createBuffer(1, AC.sampleRate * 2, AC.sampleRate);
      const d = buf.getChannelData(0);
      for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
      const src = AC.createBufferSource(); src.buffer = buf; src.loop = true;
      const bp = AC.createBiquadFilter(); bp.type = 'bandpass'; bp.frequency.value = 700; bp.Q.value = 0.6;
      crowdGain = AC.createGain(); crowdGain.gain.value = 0.035;
      src.connect(bp).connect(crowdGain).connect(AC.destination); src.start();
    }
    if (AC.state === 'suspended') AC.resume();
    return AC;
  }
  function noise(dur) {
    const b = AC.createBuffer(1, Math.ceil(AC.sampleRate * dur), AC.sampleRate);
    const d = b.getChannelData(0);
    for (let i = 0; i < d.length; i++) d[i] = Math.random() * 2 - 1;
    const s = AC.createBufferSource(); s.buffer = b; return s;
  }
  function env(g, t, peak, a, d) { g.gain.setValueAtTime(0.0001, t); g.gain.exponentialRampToValueAtTime(peak, t + a); g.gain.exponentialRampToValueAtTime(0.0001, t + a + d); }
  const sfx = {
    kick() {
      const ac = audio(); if (!ac) return; const t = ac.currentTime;
      const o = ac.createOscillator(), g = ac.createGain();
      o.frequency.setValueAtTime(160, t); o.frequency.exponentialRampToValueAtTime(45, t + 0.15);
      env(g, t, 0.9, 0.005, 0.18); o.connect(g).connect(ac.destination); o.start(t); o.stop(t + 0.25);
      const n = noise(0.05), ng = ac.createGain(); env(ng, t, 0.3, 0.002, 0.04); n.connect(ng).connect(ac.destination); n.start(t);
    },
    post() {
      const ac = audio(); if (!ac) return; const t = ac.currentTime;
      for (const [f, v] of [[880, 0.25], [1320, 0.15], [2310, 0.08]]) {
        const o = ac.createOscillator(), g = ac.createGain(); o.type = 'triangle'; o.frequency.value = f;
        env(g, t, v, 0.002, 0.9); o.connect(g).connect(ac.destination); o.start(t); o.stop(t + 1);
      }
    },
    thud() {
      const ac = audio(); if (!ac) return; const t = ac.currentTime;
      const o = ac.createOscillator(), g = ac.createGain(); o.frequency.setValueAtTime(110, t); o.frequency.exponentialRampToValueAtTime(50, t + 0.12);
      env(g, t, 0.6, 0.003, 0.14); o.connect(g).connect(ac.destination); o.start(t); o.stop(t + 0.2);
    },
    roar(big) {
      const ac = audio(); if (!ac) return; const t = ac.currentTime;
      const n = noise(3.5), f = ac.createBiquadFilter(), g = ac.createGain();
      f.type = 'bandpass'; f.frequency.value = big ? 900 : 500; f.Q.value = 0.5;
      env(g, t, big ? 0.55 : 0.2, big ? 0.25 : 0.15, big ? 3 : 1.2);
      n.connect(f).connect(g).connect(ac.destination); n.start(t);
    },
    whistle() {
      const ac = audio(); if (!ac) return; const t = ac.currentTime;
      const o = ac.createOscillator(), lfo = ac.createOscillator(), lg = ac.createGain(), g = ac.createGain();
      o.frequency.value = 2900; lfo.frequency.value = 28; lg.gain.value = 120;
      lfo.connect(lg).connect(o.frequency); env(g, t, 0.08, 0.01, 0.3);
      o.connect(g).connect(ac.destination); o.start(t); lfo.start(t); o.stop(t + 0.35); lfo.stop(t + 0.35);
    },
  };

  // ---------- Entrada (deslize) ----------
  function cssPos(e) { const r = canvas.getBoundingClientRect(); return { x: e.clientX - r.left, y: e.clientY - r.top, t: performance.now() }; }

  // Interpreta o gesto: direção → alvo no gol, velocidade do gesto → força, curvatura → efeito.
  function readSwipe(sw, final) {
    const pts = sw.points;
    const a = pts[0], b = pts[pts.length - 1];
    const dx = b.x - a.x, dy = b.y - a.y, chord = Math.hypot(dx, dy);
    const Hc = canvas.clientHeight;
    if (chord < Hc * 0.06 || dy > -Hc * 0.04) return null;
    const ballS = proj(ballPos());
    const target = unproject(ballS.x + dx * DPR * 1.05, ballS.y + dy * DPR * 1.05, 0);
    if (!target) return null;
    target.y = Math.max(P.R, target.y);
    // curvatura: maior desvio perpendicular em relação à corda
    const ux = dx / chord, uy = dy / chord;
    let dev = 0;
    for (const p of pts) {
      const c = ux * (p.y - a.y) - uy * (p.x - a.x);
      if (Math.abs(c) > Math.abs(dev)) dev = c;
    }
    const curv = dev / chord;
    const spin = Math.abs(curv) < 0.035 ? 0 : clamp(curv * 300, -P.MAX_SPIN, P.MAX_SPIN);
    // força: velocidade do gesto (alturas de tela por segundo)
    let speed = null;
    if (final) {
      const dur = Math.max(70, b.t - a.t);
      const hps = (chord / Hc) / (dur / 1000);
      speed = clamp(15 + hps * 7.5, 16, 34);
    }
    return { target, spin, speed, curv };
  }

  canvas.addEventListener('pointerdown', (e) => {
    if (S.phase !== 'aim' || S.swipe) return;
    const p = cssPos(e);
    const b = proj(ballPos());
    const r = Math.max(70, P.R * b.s / DPR * 3);
    if (Math.hypot(p.x - b.x / DPR, p.y - b.y / DPR) > r) return;
    e.preventDefault();
    canvas.setPointerCapture(e.pointerId);
    S.swipe = { id: e.pointerId, points: [p] };
    audio();
  });
  canvas.addEventListener('pointermove', (e) => {
    if (!S.swipe || e.pointerId !== S.swipe.id) return;
    S.swipe.points.push(cssPos(e));
    if (S.swipe.points.length > 120) S.swipe.points.splice(1, 1);
  });
  function endSwipe(e, cancel) {
    if (!S.swipe || e.pointerId !== S.swipe.id) return;
    const sw = S.swipe;
    sw.points.push(cssPos(e));
    sw.endTime = S.time;
    S.swipe = null; S.lastSwipe = sw;
    if (cancel) return;
    // usa só os últimos ~350 ms para a velocidade (o "chute"), mas o gesto todo para curva/direção
    const aim = readSwipe(sw, true);
    if (!aim) { flashHint('Deslize <b>para cima</b>, em direção ao gol'); return; }
    shoot(aim);
  }
  canvas.addEventListener('pointerup', (e) => endSwipe(e, false));
  canvas.addEventListener('pointercancel', (e) => endSwipe(e, true));

  // ---------- Fluxo do jogo ----------
  function difficulty(i) { return { react: 0.24 - i * 0.015, guess: 0.33 + i * 0.03 }; }

  function setupKick() {
    S.kick = P.makeKick(S.index);
    S.wall = P.buildWall(S.kick);
    const x0 = S.wall.players.length ? -S.wall.side * 0.55 : clamp(S.kick.ball.x * 0.06, -0.8, 0.8);
    S.keeper = P.keeperIdle(x0);
    S.shot = null; S.trail = []; S.netHit = null; S.postShake = 0; S.lastSwipe = null;
    S.phase = 'aim'; S.phaseTime = 0; S.introCam = 1;
    layoutCamera();
    const k = S.kick;
    ui.kickNo.textContent = `Cobrança ${S.index + 1}/${KICKS}`;
    ui.kickDesc.textContent = k.type === 'penalty'
      ? 'Pênalti · 11 m'
      : `Falta · ${Math.round(k.dist)} m · ${k.wall ? `barreira de ${k.wall}` : 'sem barreira'}`;
    ui.banner.className = 'banner';
    ui.stats.className = 'shot-stats';
    renderDots();
    sfx.whistle();
  }

  function shoot(aim) {
    const k = S.kick;
    const B = v3(k.ball.x, P.R, k.ball.z);
    const sol = P.solveLaunch(B, aim.target, aim.speed, aim.spin);
    // imprecisão humana: maior com chute forte e muito efeito
    const err = 0.005 + 0.0007 * (aim.speed - 16) + 0.006 * Math.abs(aim.spin) / P.MAX_SPIN;
    const yaw = sol.yaw + P.gauss() * err, pitch = sol.pitch + P.gauss() * err * 0.7;
    const launch = { p: B, v: P.launchVel(aim.speed, yaw, pitch), w: v3(0, aim.spin * (1 + P.gauss() * 0.05), 0) };
    S.keeper = P.keeperPlan(k, S.keeper.x0, launch, difficulty(S.index));
    S.shot = P.createShot(k, S.wall.players, S.keeper, launch);
    S.curveTrail = Math.abs(aim.spin) > 20;
    S.phase = 'flight'; S.phaseTime = 0;
    S.aim = aim;
    hideHint();
    sfx.kick();
    const kmh = Math.round(aim.speed * 3.6);
    const curve = aim.spin === 0 ? 'sem efeito' : `efeito ${aim.spin > 0 ? '↶ para a esquerda' : '↷ para a direita'}`;
    ui.stats.innerHTML = `<b>${kmh}</b> km/h · ${curve}`;
    ui.stats.className = 'shot-stats show';
  }

  const OUTCOMES = {
    goal:  { title: 'GOOOL!', good: true },
    save:  { title: 'DEFENDEU!' },
    catch: { title: 'AGARROU!' },
    wall:  { title: 'NA BARREIRA!' },
    post:  { title: 'NA TRAVE!' },
    over:  { title: 'POR CIMA!' },
    wide:  { title: 'PRA FORA!' },
    short: { title: 'FALTOU FORÇA' },
  };

  function resolve(outcome) {
    const sh = S.shot, k = S.kick;
    const scored = outcome === 'goal';
    let sub = '', pts = 0;
    if (scored) {
      const p = sh.p, bonus = [];
      pts = 100;
      if (Math.abs(p.x) > 2.7 && p.y > 1.5) { pts += 100; bonus.push('no ângulo +100'); }
      if (Math.abs(S.aim.spin) > 30) { pts += 50; bonus.push('com efeito +50'); }
      if (k.wall && sh.flags.wall === false && Math.abs(S.aim.spin) > 30) { pts += 25; bonus.push('contornou a barreira +25'); }
      if (k.dist > 25) { pts += 50; bonus.push('de longe +50'); }
      if (sh.flags.post) { pts += 25; bonus.push('bateu na trave e entrou +25'); }
      if (sh.flags.keeper) bonus.push('o goleiro ainda tocou!');
      sub = `+${pts} pontos${bonus.length ? ' · ' + bonus.join(' · ') : ''}`;
      S.points += pts;
      burst(W / 2, H * 0.35, [TEAMS[S.team].kit, TEAMS[S.team].trim, '#ffffff', '#ffd166']);
      S.flash = 1;
      sfx.roar(true);
    } else {
      const tips = {
        save: 'O goleiro chegou. Mire mais no canto ou bata mais forte.',
        catch: 'Chute fraco e no meio: fácil para o goleiro.',
        wall: 'Bateu na barreira. Use efeito (gesto em curva) para contornar.',
        post: 'Por centímetros!',
        over: 'Força demais ou mirou alto demais.',
        wide: 'Mira aberta. Lembre que o efeito leva a bola de volta.',
        short: 'Deslize mais rápido para chutar com mais força.',
      };
      sub = tips[outcome] || '';
      sfx.roar(false);
    }
    S.results.push(scored);
    ui.bannerTitle.textContent = OUTCOMES[outcome].title;
    ui.bannerSub.textContent = sub;
    ui.banner.className = `banner show ${scored ? 'is-goal' : 'is-miss'}`;
    ui.goals.textContent = S.results.filter(Boolean).length;
    ui.points.textContent = S.points;
    bump(scored ? ui.goals : null); bump(ui.points);
    renderDots();
    S.phase = 'result'; S.phaseTime = 0;
  }

  function bump(el) { if (!el) return; el.classList.remove('bump'); void el.offsetWidth; el.classList.add('bump'); }

  function renderDots() {
    ui.dots.innerHTML = '';
    for (let i = 0; i < KICKS; i++) {
      const d = document.createElement('span');
      const r = S.results[i];
      d.className = 'dot' + (r === true ? ' is-goal' : r === false ? ' is-miss' : i === S.results.length ? ' current' : '');
      ui.dots.appendChild(d);
    }
  }

  function nextKick() {
    S.index++;
    if (S.index >= KICKS) { showSummary(); return; }
    setupKick();
  }

  function showSummary() {
    S.phase = 'summary';
    const goals = S.results.filter(Boolean).length;
    const record = S.points > S.best;
    if (record) { S.best = S.points; store.set('cng.best', S.best); }
    $('#sumTitle').textContent = goals === 5 ? 'Perfeito! 🏆' : goals >= 3 ? 'Craque!' : goals >= 1 ? 'Dá pra melhorar' : 'O goleiro venceu';
    $('#sumGoals').textContent = goals;
    $('#sumPoints').textContent = S.points;
    $('#sumBest').textContent = record ? 'Novo recorde!' : `Recorde: ${S.best} pontos`;
    const dots = $('#sumDots'); dots.innerHTML = ui.dots.innerHTML;
    ui.summary.hidden = false;
    $('#again').focus();
  }

  function newGame() {
    S.index = 0; S.results = []; S.points = 0;
    ui.goals.textContent = '0'; ui.points.textContent = '0';
    ui.summary.hidden = true; ui.intro.hidden = true; ui.hud.hidden = false;
    S.fade = 1;
    setupKick();
  }

  // ---------- Dicas ----------
  let hintTimer = 0;
  function hideHint() { if (S.hintShown) { S.hintShown = false; ui.hint.classList.add('hide'); } }
  function flashHint(html) {
    ui.hint.innerHTML = html; ui.hint.classList.remove('hide'); S.hintShown = true;
    clearTimeout(hintTimer); hintTimer = setTimeout(hideHint, 2200);
  }

  // ---------- Loop ----------
  let last = performance.now();
  function frame(now) {
    const realDt = Math.min(0.05, (now - last) / 1000); last = now;
    S.time += realDt; S.phaseTime += realDt;
    S.shake = Math.max(0, S.shake - realDt * 3);
    S.flash = Math.max(0, S.flash - realDt * 2.5);
    S.postShake = Math.max(0, (S.postShake || 0) - realDt * 2);
    S.fade = Math.max(0, (S.fade || 0) - realDt * 2.2);

    if (S.kick) updateCamera(realDt);

    if ((S.phase === 'flight' || S.phase === 'result') && S.shot) {
      const sh = S.shot;
      // câmera lenta perto do gol
      const near = S.phase === 'flight' && sh.p.z < 7 && sh.p.z > -0.5 && sh.v.z < 0;
      S.timeScale = lerp(S.timeScale, near ? 0.38 : 1, 1 - Math.exp(-realDt * 10));
      let simDt = realDt * S.timeScale;
      while (simDt > 1e-6) {
        const dt = Math.min(P.DT, simDt); simDt -= dt;
        P.stepShot(sh, dt);
      }
      S.ballSpin += (P.len(sh.v) * 0.12 + sh.w.y * 0.02) * realDt * S.timeScale * 6;
      S.trail.push({ ...sh.p }); if (S.trail.length > 26) S.trail.shift();
      for (const ev of sh.events) handleEvent(ev);
      sh.events.length = 0;
      if (S.phase === 'result' && S.phaseTime > 2.4) { S.fade = 0; nextKick(); }
    } else {
      S.timeScale = 1;
    }

    render(realDt);
    requestAnimationFrame(frame);
  }

  function handleEvent(ev) {
    switch (ev.type) {
      case 'post': sfx.post(); S.shake = 1; S.postShake = 1; break;
      case 'wall': sfx.thud(); S.shake = 0.4; break;
      case 'keeper': sfx.thud(); S.shake = 0.5; break;
      case 'net': S.netHit = { x: ev.p.x, y: ev.p.y, t: S.time, force: Math.min(30, ev.force) }; break;
      case 'outcome': if (S.phase === 'flight') resolve(ev.outcome); break;
      default: break;
    }
  }

  function updateCamera(dt) {
    const base = cam.base || cameraFor(S.kick);
    S.introCam = Math.max(0, (S.introCam || 0) - dt * 1.4);
    const e = S.introCam * S.introCam * (3 - 2 * S.introCam);
    let pos = P.add(P.mul(base.pos, 1 - e), P.mul(base.intro, e));
    let look = base.look;
    if (S.shot && (S.phase === 'flight' || S.phase === 'result')) {
      // leve acompanhamento da bola
      const b = S.shot.p;
      const k = clamp(S.shot.t / 1.2, 0, 1) * 0.18;
      look = v3(lerp(look.x, b.x, k), lerp(look.y, b.y, k * 0.5), look.z);
      pos = P.add(pos, P.mul(P.sub(v3(0, 1.6, 0), pos), k * 0.25));
    }
    setCamera(pos, look);
  }

  // ---------- Intro / controles ----------
  let selected = document.querySelector('.country.selected');
  document.querySelectorAll('.country').forEach((btn) => btn.addEventListener('click', () => {
    selected.classList.remove('selected'); selected.setAttribute('aria-pressed', 'false');
    btn.classList.add('selected'); btn.setAttribute('aria-pressed', 'true');
    selected = btn;
  }));
  const lastTeam = store.get('cng.team', null);
  const lastBtn = lastTeam && document.querySelector(`.country[data-team="${lastTeam}"]`);
  if (lastBtn) lastBtn.click();

  $('#play').addEventListener('click', () => {
    S.team = selected.dataset.team;
    const others = Object.keys(TEAMS).filter((t) => t !== S.team);
    S.rival = others[(Math.random() * others.length) | 0];
    store.set('cng.team', S.team);
    ui.flag.style.background = TEAMS[S.team].flag;
    ui.team.textContent = `${S.team} × ${S.rival}`;
    audio();
    newGame();
  });
  $('#again').addEventListener('click', newGame);
  $('#changeTeam').addEventListener('click', () => { ui.summary.hidden = true; ui.intro.hidden = false; S.phase = 'intro'; });
  $('#restart').addEventListener('click', newGame);
  function syncMute() { ui.mute.textContent = S.muted ? '🔇' : '🔊'; ui.mute.setAttribute('aria-label', S.muted ? 'Ativar som' : 'Desativar som'); }
  ui.mute.addEventListener('click', () => {
    S.muted = !S.muted; store.set('cng.muted', S.muted ? '1' : '0'); syncMute();
    if (AC) S.muted ? AC.suspend() : AC.resume();
  });
  syncMute();

  // Cena de fundo atrás da tela inicial
  TEX.crowd = makeCrowd(); TEX.boards = makeBoards();
  window.addEventListener('resize', resize);
  S.kick = P.makeKick(0); S.wall = P.buildWall(S.kick); S.keeper = P.keeperIdle(0);
  resize();
  renderDots();
  requestAnimationFrame(frame);

  // Gancho de teste: window.__penalty expõe estado para automação
  window.__cng = { S, shoot, proj, ballPos, newGame, readSwipe };
})();
