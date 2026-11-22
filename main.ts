// Sokoban solver: push-optimal per room.
//
// A level is a hallway with independent rooms off it (boxes never leave their
// room), so the level's push-optimal count is the sum of its rooms' optima.
// The solver splits the level into rooms, solves every room push-optimally
// with A* over box configurations (player reachability by flood fill,
// deadlock pruning, min-assignment heuristic), then joins the rooms with the
// player's walks through the hallway.
//
// No clock, no randomness, no network: the same input always gives the same
// output.
import { mkdirSync, readdirSync, readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const I = process.env.MAKEFUN_INPUT_DIR ?? "/input";
const O = process.env.MAKEFUN_OUTPUT_DIR ?? "/output";

// Directions: 0 up, 1 down, 2 left, 3 right.
const DX = [0, 0, -1, 1];
const DY = [-1, 1, 0, 0];
const CH = ["u", "d", "l", "r"];

// ---------------------------------------------------------------- parsing

function parseLevel(text) {
  const raw = text.split("\n");
  if (raw.length > 0 && raw[raw.length - 1] === "") raw.pop();
  const h = raw.length;
  const w = Math.max(0, ...raw.map((l) => l.length));
  const wall = new Uint8Array(w * h).fill(1);
  const goal = new Uint8Array(w * h);
  const boxes = [];
  let player = -1;
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      const c = raw[y][x] ?? "#";
      const i = y * w + x;
      if (c === "#") continue;
      wall[i] = 0;
      if (c === ".") goal[i] = 1;
      else if (c === "$") boxes.push(i);
      else if (c === "*") {
        goal[i] = 1;
        boxes.push(i);
      } else if (c === "@") player = i;
      else if (c === "+") {
        goal[i] = 1;
        player = i;
      }
    }
  }
  boxes.sort((a, b) => a - b);
  return { w, h, wall, goal, boxes, player };
}

// ------------------------------------------------------- walking (BFS path)

function walkPath(w, h, wall, blocked, from, to) {
  if (from === to) return "";
  const prev = new Int32Array(w * h).fill(-1);
  const prevDir = new Int8Array(w * h).fill(-1);
  const q = [from];
  prev[from] = from;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked[n] || prev[n] !== -1) continue;
      prev[n] = c;
      prevDir[n] = d;
      if (n === to) {
        const dirs = [];
        let cur = n;
        while (cur !== from) {
          dirs.push(prevDir[cur]);
          cur = prev[cur];
        }
        dirs.reverse();
        return dirs.map((d) => CH[d]).join("");
      }
      q.push(n);
    }
  }
  return null;
}

// ------------------------------------------------- flood fill (reachability)

function floodRegion(w, h, wall, blocked, from) {
  const seen = new Uint8Array(w * h);
  const q = [from];
  seen[from] = 1;
  for (let qi = 0; qi < q.length; qi++) {
    const c = q[qi];
    const cx = c % w;
    const cy = (c / w) | 0;
    for (let d = 0; d < 4; d++) {
      const nx = cx + DX[d];
      const ny = cy + DY[d];
      if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
      const n = ny * w + nx;
      if (wall[n] || blocked[n] || seen[n]) continue;
      seen[n] = 1;
      q.push(n);
    }
  }
  return seen;
}

// ------------------------------------------------------------ room finding

function findRooms(level) {
  const { w, h, wall, goal, boxes, player } = level;
  const hallRow = (player / w) | 0;
  const boxSet = new Set(boxes);
  const seen = new Uint8Array(w * h);
  const rooms = [];
  for (let i = 0; i < w * h; i++) {
    if (wall[i] || seen[i] || ((i / w) | 0) === hallRow) continue;
    const cells = [];
    const stack = [i];
    seen[i] = 1;
    while (stack.length > 0) {
      const c = stack.pop();
      cells.push(c);
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d];
        const ny = cy + DY[d];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const n = ny * w + nx;
        if (wall[n] || seen[n] || ny === hallRow) continue;
        seen[n] = 1;
        stack.push(n);
      }
    }
    const rboxes = cells.filter((c) => boxSet.has(c)).sort((a, b) => a - b);
    if (rboxes.length === 0) continue;
    const rgoals = cells.filter((c) => goal[c]).sort((a, b) => a - b);
    const doors = [];
    let entry = -1;
    for (const c of cells) {
      const cx = c % w;
      const cy = (c / w) | 0;
      for (let d = 0; d < 4; d++) {
        const nx = cx + DX[d];
        const ny = cy + DY[d];
        if (ny === hallRow && nx >= 0 && nx < w && !wall[ny * w + nx]) {
          doors.push(c);
          if (entry === -1) entry = ny * w + nx;
          break;
        }
      }
    }
    rooms.push({ cells: new Set(cells), boxes: rboxes, goals: rgoals, doors, entry });
  }
  return { hallRow, rooms };
}

// ------------------------------------------- push distance to one goal (BFS)

// dist[c] = fewest pushes for a box on c to reach goal g, ignoring other
// boxes. A box on c can be pushed to x when x is in the domain and the player
// can stand on the cell beyond c (2c - x), which must be floor.
function pushDistToGoal(w, h, wall, domain, g) {
  const dist = new Int32Array(w * h).fill(-1);
  dist[g] = 0;
  const q = [g];
  for (let qi = 0; qi < q.length; qi++) {
    const x = q[qi];
    const xx = x % w;
    const xy = (x / w) | 0;
    for (let d = 0; d < 4; d++) {
      const cx = xx - DX[d];
      const cy = xy - DY[d];
      const sx = xx - 2 * DX[d];
      const sy = xy - 2 * DY[d];
      if (cx < 0 || cy < 0 || cx >= w || cy >= h) continue;
      if (sx < 0 || sy < 0 || sx >= w || sy >= h) continue;
      const c = cy * w + cx;
      const s = sy * w + sx;
      if (!domain.has(c) || wall[s] || dist[c] !== -1) continue;
      dist[c] = dist[x] + 1;
      q.push(c);
    }
  }
  return dist;
}

// ------------------------------------------------------------- deadlocking

function isDeadBox(w, h, wall, goal, boxSet, b) {
  if (goal[b]) return false;
  const bx = b % w;
  const by = (b / w) | 0;
  const up = by === 0 || wall[b - w];
  const dn = by === h - 1 || wall[b + w];
  const lf = bx === 0 || wall[b - 1];
  const rt = bx === w - 1 || wall[b + 1];
  if ((up && lf) || (up && rt) || (dn && lf) || (dn && rt)) return true;
  for (let oy = -1; oy <= 0; oy++) {
    for (let ox = -1; ox <= 0; ox++) {
      const x0 = bx + ox;
      const y0 = by + oy;
      if (x0 < 0 || y0 < 0 || x0 + 1 >= w || y0 + 1 >= h) continue;
      let allBlocked = true;
      let hasGoal = false;
      for (let dy = 0; dy < 2; dy++) {
        for (let dx = 0; dx < 2; dx++) {
          const c = (y0 + dy) * w + (x0 + dx);
          if (goal[c]) hasGoal = true;
          if (!wall[c] && !boxSet.has(c)) allBlocked = false;
        }
      }
      if (allBlocked && !hasGoal) return true;
    }
  }
  return false;
}

// ------------------------------------------------------------------- heap

class Heap {
  constructor() {
    this.a = [];
  }
  get size() {
    return this.a.length;
  }
  push(e) {
    const a = this.a;
    a.push(e);
    let i = a.length - 1;
    while (i > 0) {
      const p = (i - 1) >> 1;
      if (cmp(a[i], a[p]) >= 0) break;
      [a[i], a[p]] = [a[p], a[i]];
      i = p;
    }
  }
  pop() {
    const a = this.a;
    const top = a[0];
    const last = a.pop();
    if (a.length > 0) {
      a[0] = last;
      let i = 0;
      for (;;) {
        const l = 2 * i + 1;
        const r = l + 1;
        let m = i;
        if (l < a.length && cmp(a[l], a[m]) < 0) m = l;
        if (r < a.length && cmp(a[r], a[m]) < 0) m = r;
        if (m === i) break;
        [a[i], a[m]] = [a[m], a[i]];
        i = m;
      }
    }
    return top;
  }
}
// entries [f, h, counter, key]: lower f, then lower h, then earlier entry.
function cmp(a, b) {
  if (a[0] !== b[0]) return a[0] - b[0];
  if (a[1] !== b[1]) return a[1] - b[1];
  return a[2] - b[2];
}

function permutations(n) {
  const out = [];
  const cur = [];
  const used = new Array(n).fill(false);
  (function gen() {
    if (cur.length === n) {
      out.push(cur.slice());
      return;
    }
    for (let i = 0; i < n; i++) {
      if (used[i]) continue;
      used[i] = true;
      cur.push(i);
      gen();
      cur.pop();
      used[i] = false;
    }
  })();
  return out;
}

// ------------------------------------------------------------ room solving

const MAX_NODES = 1000000;

// Solve one room push-optimally (weight 1) or greedily (weight > 1).
// level: parsed level; room: from findRooms; otherBlocked: occupancy of the
// boxes that are not this room's (static during this search); startPlayer:
// where the player stands (the hallway cell outside the door).
// Returns { moves, pushes, finalBoxes, finalPlayer } or null.
function solveRoom(level, room, otherBlocked, startPlayer, weight) {
  const { w, h, wall, goal } = level;
  const domain = room.cells;
  const goals = room.goals;
  const n = room.boxes.length;
  if (n === 0) return { moves: "", pushes: 0, finalBoxes: [], finalPlayer: startPlayer };
  if (goals.length !== n) return null;
  const dmaps = goals.map((g) => pushDistToGoal(w, h, wall, domain, g));
  const perms = permutations(n);
  const heuristic = (boxes) => {
    let best = Infinity;
    for (const p of perms) {
      let s = 0;
      for (let i = 0; i < n; i++) {
        const d = dmaps[p[i]][boxes[i]];
        if (d < 0) {
          s = Infinity;
          break;
        }
        s += d;
      }
      if (s < best) best = s;
    }
    return best;
  };

  const startBoxes = room.boxes.slice().sort((a, b) => a - b);
  const keyOf = (boxes, player) => boxes.join(",") + "|" + player;
  const startKey = keyOf(startBoxes, startPlayer);
  const states = new Map();
  states.set(startKey, { boxes: startBoxes, player: startPlayer, parent: null, action: null, g: 0 });
  const h0 = heuristic(startBoxes);
  if (h0 === Infinity) return null;
  const heap = new Heap();
  let counter = 0;
  heap.push([h0, h0, counter++, startKey]);
  let expansions = 0;
  let goalKey = null;

  while (heap.size > 0) {
    const [, , , key] = heap.pop();
    const st = states.get(key);
    if (!st) continue;
    if (st.boxes.every((b) => goal[b])) {
      goalKey = key;
      break;
    }
    if (++expansions > MAX_NODES) return null;
    const blocked = otherBlocked.slice();
    for (const b of st.boxes) blocked[b] = 1;
    const reach = floodRegion(w, h, wall, blocked, st.player);
    for (let bi = 0; bi < n; bi++) {
      const b = st.boxes[bi];
      const bx = b % w;
      const by = (b / w) | 0;
      for (let d = 0; d < 4; d++) {
        const px = bx - DX[d];
        const py = by - DY[d];
        const nx = bx + DX[d];
        const ny = by + DY[d];
        if (px < 0 || py < 0 || px >= w || py >= h) continue;
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
        const behind = py * w + px;
        const dest = ny * w + nx;
        if (!reach[behind]) continue;
        if (!domain.has(dest) || blocked[dest]) continue;
        const nb = st.boxes.slice();
        nb[bi] = dest;
        nb.sort((a, b2) => a - b2);
        const nbSet = new Set(nb);
        let dead = false;
        for (const bb of nb) {
          if (goal[bb]) continue;
          let ok = false;
          for (let gi = 0; gi < n; gi++) {
            if (dmaps[gi][bb] >= 0) {
              ok = true;
              break;
            }
          }
          if (!ok || isDeadBox(w, h, wall, goal, nbSet, bb)) {
            dead = true;
            break;
          }
        }
        if (dead) continue;
        const nkey = keyOf(nb, b);
        const ng = st.g + 1;
        const prev = states.get(nkey);
        if (prev && prev.g <= ng) continue;
        const nh = heuristic(nb);
        if (nh === Infinity) continue;
        states.set(nkey, { boxes: nb, player: b, parent: key, action: { from: b, to: dest, dir: d }, g: ng });
        heap.push([ng + weight * nh, nh, counter++, nkey]);
      }
    }
  }
  if (goalKey === null) return null;

  // Rebuild the move string: for each push, the player's walk to the push
  // position (in the parent state's configuration) plus the push itself.
  const actions = [];
  let cur = goalKey;
  while (states.get(cur).parent !== null) {
    const s = states.get(cur);
    actions.push({ st: s, parent: states.get(s.parent) });
    cur = s.parent;
  }
  actions.reverse();
  let moves = "";
  for (const { st, parent } of actions) {
    const a = st.action;
    const blocked = otherBlocked.slice();
    for (const b of parent.boxes) blocked[b] = 1;
    const behind = a.from - DY[a.dir] * w - DX[a.dir];
    const walk = walkPath(w, h, wall, blocked, parent.player, behind);
    if (walk === null) return null;
    moves += walk + CH[a.dir];
  }
  const goalState = states.get(goalKey);
  return { moves, pushes: goalState.g, finalBoxes: goalState.boxes, finalPlayer: goalState.player };
}

// --------------------------------------------------------------- level solve

function solve(levelText) {
  const level = parseLevel(levelText);
  const { w, h, wall, goal, boxes, player } = level;
  const { rooms } = findRooms(level);
  let todo = rooms;
  if (todo.length === 0) {
    // No hallway split (player inside the only room): the whole level is one room.
    const cells = new Set();
    for (let i = 0; i < w * h; i++) if (!wall[i]) cells.add(i);
    const goals = [];
    for (let i = 0; i < w * h; i++) if (goal[i]) goals.push(i);
    todo = [{ cells, boxes: boxes.slice(), goals, doors: [], entry: player }];
  }
  // Deterministic room order: by entry cell (door column along the hallway).
  todo.sort((a, b) => a.entry - b.entry);
  const curBoxes = new Set(boxes);
  let curPlayer = player;
  let moves = "";
  for (const room of todo) {
    if (room.entry === -1) return "";
    const blocked = new Uint8Array(w * h);
    for (const b of curBoxes) blocked[b] = 1;
    const walk = walkPath(w, h, wall, blocked, curPlayer, room.entry);
    if (walk === null) return "";
    moves += walk;
    curPlayer = room.entry;
    const otherBlocked = new Uint8Array(w * h);
    for (const b of curBoxes) if (!room.boxes.includes(b)) otherBlocked[b] = 1;
    let res = solveRoom(level, room, otherBlocked, curPlayer, 1);
    if (res === null) res = solveRoom(level, room, otherBlocked, curPlayer, 4);
    if (res === null) return "";
    moves += res.moves;
    for (const b of room.boxes) curBoxes.delete(b);
    for (const b of res.finalBoxes) curBoxes.add(b);
    curPlayer = res.finalPlayer;
  }
  return moves;
}

// -------------------------------------------------------------------- main

for (const id of readdirSync(I).sort()) {
  const moves = solve(readFileSync(join(I, id, "level.txt"), "utf8"));
  mkdirSync(join(O, id), { recursive: true });
  writeFileSync(join(O, id, "solution.txt"), `${moves}\n`);
}
