/*
  بسم الله الرحمن الرحيم
*/
// Fast Input
const fs = require("fs");
const input = fs.readFileSync(0, "utf8").trim().split(/\s+/);
let idx = 0;

// Input readers
const readInt = () => Number(input[idx++]);
const readBigInt = () => BigInt(input[idx++]);
const readStr = () => input[idx++];
const readIntArr = (n) => Array.from({ length: n }, () => readInt());

// Output buffer
let output = [];

// Modulo (common in Codeforces)
const MOD = 1_000_000_007n;

// ---------- Math Utilities ----------
function gcd(a, b) {
  return b === 0 ? a : gcd(b, a % b);
}
function lcm(a, b) {
  return (a / gcd(a, b)) * b;
}

// Modular arithmetic
function modAdd(a, b, m = MOD) {
  return ((a % m) + (b % m)) % m;
}
function modSub(a, b, m = MOD) {
  return ((a % m) - (b % m) + m) % m;
}
function modMul(a, b, m = MOD) {
  return ((a % m) * (b % m)) % m;
}
function modPow(base, exp, m = MOD) {
  base %= m;
  let res = 1n;
  while (exp > 0n) {
    if (exp & 1n) res = (res * base) % m;
    base = (base * base) % m;
    exp >>= 1n;
  }
  return res;
}
function modInv(a, m = MOD) {
  return modPow(a, m - 2n, m);
}
function modDiv(a, b, m = MOD) {
  return modMul(a, modInv(b, m), m);
}

// nCr with factorial precomputation
const fact = [1n];
const invFact = [1n];
function precomputeFactorials(n, m = MOD) {
  for (let i = 1n; i <= n; i++) {
    fact[i] = (fact[i - 1n] * i) % m;
  }
  invFact[n] = modInv(fact[n], m);
  for (let i = n - 1n; i >= 0n; i--) {
    invFact[i] = (invFact[i + 1n] * (i + 1n)) % m;
  }
}
function nCr(n, r, m = MOD) {
  if (r < 0n || r > n) return 0n;
  return (((fact[n] * invFact[r]) % m) * invFact[n - r]) % m;
}

// ---------- Graph Utilities ----------

// Build adjacency list
function buildGraph(n, edges, directed = false) {
  const graph = Array.from({ length: n }, () => []);
  for (const [u, v, w = 1] of edges) {
    graph[u].push([v, w]);
    if (!directed) graph[v].push([u, w]);
  }
  return graph;
}

// BFS (unweighted shortest paths)
function bfs(start, n, graph) {
  const dist = Array(n).fill(-1);
  const queue = [start];
  dist[start] = 0;
  for (let i = 0; i < queue.length; i++) {
    const u = queue[i];
    for (const [v] of graph[u]) {
      if (dist[v] === -1) {
        dist[v] = dist[u] + 1;
        queue.push(v);
      }
    }
  }
  return dist;
}

// DFS (recursive)
function dfs(u, graph, visited, action = () => {}) {
  visited[u] = true;
  action(u);
  for (const [v] of graph[u]) {
    if (!visited[v]) dfs(v, graph, visited, action);
  }
}

// Dijkstra (weighted shortest paths)
function dijkstra(start, n, graph) {
  const dist = Array(n).fill(Infinity);
  const pq = [[0, start]]; // [dist, node]
  dist[start] = 0;

  while (pq.length) {
    pq.sort((a, b) => a[0] - b[0]); // simple priority queue
    const [d, u] = pq.shift();
    if (d !== dist[u]) continue;

    for (const [v, w] of graph[u]) {
      if (dist[v] > d + w) {
        dist[v] = d + w;
        pq.push([dist[v], v]);
      }
    }
  }
  return dist;
}

// ---------- Main solver ----------
function solve() {
  const t = readInt(); // number of test cases
  for (let _ = 0; _ < t; _++) {
    const n = readInt(); // number of nodes
    const m = readInt(); // number of edges

    const edges = [];
    for (let i = 0; i < m; i++) {
      const u = readInt() - 1;
      const v = readInt() - 1;
      const w = readInt(); // weight (if unweighted, ignore)
      edges.push([u, v, w]);
    }

    const graph = buildGraph(n, edges, false);

    // Example: shortest path from node 0
    const dist = dijkstra(0, n, graph);

    output.push(dist.join(" "));
  }
}

// Run
solve();
console.log(output.join("\n"));
