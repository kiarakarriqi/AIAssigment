#!/usr/bin/env python3
"""
adaptive_puzzle_solver.py

Battery-constrained Multi-Goal Grid with Recharge Stations.
Implements:
 - Uniform-Cost Search (UCS) -- uninformed
 - A* search with admissible heuristic: dist-to-nearest + MST(remaining targets)

Requirements:
    - Python 3.8+
    - matplotlib
    - pandas
"""

import heapq
import time
import random
import csv
from collections import namedtuple
import matplotlib.pyplot as plt
import pandas as pd
import math

# -------------------------
# Basic types and helpers
# -------------------------
Point = tuple  # (r, c)
State = namedtuple("State", ["pos", "mask", "battery"])  # mask: bitmask of visited targets

def manhattan(a: Point, b: Point) -> int:
    return abs(a[0]-b[0]) + abs(a[1]-b[1])

def neighbors(pos, rows, cols):
    r,c = pos
    for nr,nc in ((r+1,c),(r-1,c),(r,c+1),(r,c-1)):
        if 0 <= nr < rows and 0 <= nc < cols:
            yield (nr,nc)

# -------------------------
# Instance generator
# -------------------------
def generate_instance(rows=10, cols=10, n_targets=3, obstacle_prob=0.12, n_recharge=2,
                      max_battery=10, recharge_time=5, seed=None):
    rnd = random.Random(seed)
    obstacles = set()
    for r in range(rows):
        for c in range(cols):
            if rnd.random() < obstacle_prob:
                obstacles.add((r,c))
    start = (0,0)
    obstacles.discard(start)
    free_cells = [(r,c) for r in range(rows) for c in range(cols) if (r,c) not in obstacles and (r,c)!=start]
    rnd.shuffle(free_cells)
    if len(free_cells) < n_targets + n_recharge:
        obstacles = set()
        free_cells = [(r,c) for r in range(rows) for c in range(cols) if (r,c)!=start]
        rnd.shuffle(free_cells)
    targets = free_cells[:n_targets]
    recharge_stations = set(free_cells[n_targets:n_targets+n_recharge])
    return {
        "rows": rows, "cols": cols,
        "obstacles": obstacles,
        "start": start,
        "targets": targets,
        "recharge_stations": recharge_stations,
        "max_battery": max_battery,
        "recharge_time": recharge_time,
        "seed": seed
    }

# -------------------------
# Heuristic helpers: MST + caching
# -------------------------
def mst_lower_bound(points):
    """Prim-like MST lower bound using Manhattan distances."""
    if len(points) <= 1:
        return 0
    pts = list(points)
    n = len(pts)
    in_mst = [False]*n
    mincost = [float('inf')]*n
    mincost[0] = 0
    total = 0
    for _ in range(n):
        u = -1
        for i in range(n):
            if not in_mst[i] and (u == -1 or mincost[i] < mincost[u]):
                u = i
        in_mst[u] = True
        total += mincost[u]
        for v in range(n):
            if not in_mst[v]:
                d = manhattan(pts[u], pts[v])
                if d < mincost[v]:
                    mincost[v] = d
    return total

_mst_cache = {}
def mst_cached(points):
    """Cache MST lower bound. Key uses sorted points row-major for determinism."""
    if not points:
        return 0
    # ensure deterministic ordering for cache key
    key = tuple(sorted(points, key=lambda p: (p[0], p[1])))
    if key in _mst_cache:
        return _mst_cache[key]
    val = mst_lower_bound(points)
    _mst_cache[key] = val
    return val

def heuristic(state: State, problem):
    targets = problem["targets"]
    rem = []
    for i,t in enumerate(targets):
        if not ((state.mask >> i) & 1):
            rem.append(t)
    if not rem:
        return 0
    d_to_nearest = min(manhattan(state.pos, t) for t in rem)
    mst = mst_cached(rem)
    return d_to_nearest + mst

# -------------------------
# UCS implementation
# -------------------------
def uniform_cost_search(problem, node_limit=200000):
    rows = problem["rows"]; cols = problem["cols"]
    obstacles = problem["obstacles"]
    start = problem["start"]
    targets = problem["targets"]
    recharge = problem["recharge_stations"]
    max_battery = problem["max_battery"]
    recharge_time = problem["recharge_time"]
    n_targets = len(targets)
    goal_mask = (1<<n_targets) - 1

    start_state = State(start, 0, max_battery)
    pq = []
    counter = 0
    heapq.heappush(pq, (0, counter, start_state, None, None))  # cost, counter, state, parent_idx, action
    counter += 1
    best_cost = {}   # exact key -> best cost
    best_by_posmask = {}  # (pos,mask) -> (best_cost, best_battery) for dominance pruning
    parents = []
    nodes_expanded = 0
    max_fringe_size = max(1, len(pq))

    while pq:
        if len(pq) > max_fringe_size:
            max_fringe_size = len(pq)
        cost, _, state, parent_idx, action = heapq.heappop(pq)
        key = (state.pos, state.mask, state.battery)

        # skip if we already have equal or better cost for same full key
        if key in best_cost and best_cost[key] <= cost:
            continue

        # dominance pruning: if same (pos,mask) was seen with cost <= cost and battery >= battery, skip
        pm = (state.pos, state.mask)
        if pm in best_by_posmask:
            prev_cost, prev_batt = best_by_posmask[pm]
            if prev_cost <= cost and prev_batt >= state.battery:
                continue

        best_cost[key] = cost
        # update best_by_posmask
        if pm not in best_by_posmask or cost < best_by_posmask[pm][0] or state.battery > best_by_posmask[pm][1]:
            best_by_posmask[pm] = (cost, state.battery)

        parents.append((state, parent_idx, action, cost))
        current_idx = len(parents)-1
        nodes_expanded += 1

        if state.mask == goal_mask:
            path = []
            idx = current_idx
            while idx is not None:
                s,p,a,c = parents[idx]
                path.append((s,a,c))
                idx = p
            path.reverse()
            return {"success": True, "cost": cost, "nodes": nodes_expanded, "path": path,
                    "explored_states": len(best_cost), "max_fringe_size": max_fringe_size}

        if nodes_expanded > node_limit:
            return {"success": False, "reason": "node_limit", "nodes": nodes_expanded,
                    "cost": None, "explored_states": len(best_cost), "max_fringe_size": max_fringe_size}

        for nb in neighbors(state.pos, rows, cols):
            if nb in obstacles or state.battery <= 0:
                continue
            new_battery = state.battery - 1
            new_mask = state.mask
            for i,t in enumerate(targets):
                if nb == t:
                    new_mask |= (1<<i)
            new_state = State(nb, new_mask, new_battery)
            heapq.heappush(pq, (cost + 1, counter, new_state, current_idx, ("move", nb)))
            counter += 1

        if state.pos in recharge and state.battery < max_battery:
            new_state = State(state.pos, state.mask, max_battery)
            heapq.heappush(pq, (cost + recharge_time, counter, new_state, current_idx, ("recharge", state.pos)))
            counter += 1

    return {"success": False, "reason": "empty", "nodes": nodes_expanded,
            "cost": None, "explored_states": len(best_cost), "max_fringe_size": max_fringe_size}

# -------------------------
# A* implementation
# -------------------------
def a_star_search(problem, node_limit=200000):
    rows = problem["rows"]; cols = problem["cols"]
    obstacles = problem["obstacles"]
    start = problem["start"]
    targets = problem["targets"]
    recharge = problem["recharge_stations"]
    max_battery = problem["max_battery"]
    recharge_time = problem["recharge_time"]
    n_targets = len(targets)
    goal_mask = (1<<n_targets) - 1

    start_state = State(start, 0, max_battery)
    g0 = 0
    h0 = heuristic(start_state, problem)
    pq = []
    counter = 0
    heapq.heappush(pq, (g0 + h0, g0, counter, start_state, None, None))
    counter += 1
    best_g = {}
    best_by_posmask = {}
    parents = []
    nodes_expanded = 0
    max_fringe_size = max(1, len(pq))

    while pq:
        if len(pq) > max_fringe_size:
            max_fringe_size = len(pq)
        f, g, _, state, parent_idx, action = heapq.heappop(pq)
        key = (state.pos, state.mask, state.battery)

        # skip if we already have equal or better g for same exact key
        if key in best_g and best_g[key] <= g:
            continue

        # dominance pruning on (pos,mask)
        pm = (state.pos, state.mask)
        if pm in best_by_posmask:
            prev_cost, prev_batt = best_by_posmask[pm]
            if prev_cost <= g and prev_batt >= state.battery:
                continue

        best_g[key] = g
        if pm not in best_by_posmask or g < best_by_posmask[pm][0] or state.battery > best_by_posmask[pm][1]:
            best_by_posmask[pm] = (g, state.battery)

        parents.append((state, parent_idx, action, g))
        current_idx = len(parents)-1
        nodes_expanded += 1

        if state.mask == goal_mask:
            path = []
            idx = current_idx
            while idx is not None:
                s,p,a,c = parents[idx]
                path.append((s,a,c))
                idx = p
            path.reverse()
            return {"success": True, "cost": g, "nodes": nodes_expanded, "path": path,
                    "explored_states": len(best_g), "max_fringe_size": max_fringe_size}

        if nodes_expanded > node_limit:
            return {"success": False, "reason": "node_limit", "nodes": nodes_expanded,
                    "cost": None, "explored_states": len(best_g), "max_fringe_size": max_fringe_size}

        for nb in neighbors(state.pos, rows, cols):
            if nb in obstacles or state.battery <= 0:
                continue
            new_battery = state.battery - 1
            new_mask = state.mask
            for i,t in enumerate(targets):
                if nb == t:
                    new_mask |= (1<<i)
            new_state = State(nb, new_mask, new_battery)
            new_g = g + 1
            h = heuristic(new_state, problem)
            heapq.heappush(pq, (new_g + h, new_g, counter, new_state, current_idx, ("move", nb)))
            counter += 1

        if state.pos in recharge and state.battery < max_battery:
            new_state = State(state.pos, state.mask, max_battery)
            new_g = g + recharge_time
            h = heuristic(new_state, problem)
            heapq.heappush(pq, (new_g + h, new_g, counter, new_state, current_idx, ("recharge", state.pos)))
            counter += 1

    return {"success": False, "reason": "empty", "nodes": nodes_expanded,
            "cost": None, "explored_states": len(best_g), "max_fringe_size": max_fringe_size}

# -------------------------
# Utilities
# -------------------------
def path_to_positions(path):
    return [step[0].pos for step in path]

def evaluate_instance(problem):
    t0 = time.perf_counter(); res_ucs = uniform_cost_search(problem); t1 = time.perf_counter(); res_ucs["time"] = t1 - t0
    t0 = time.perf_counter(); res_astar = a_star_search(problem); t1 = time.perf_counter(); res_astar["time"] = t1 - t0
    return {"ucs": res_ucs, "astar": res_astar}

def run_experiments(n_instances=6, seed_base=42, **gen_kwargs):
    experiments = []
    for i in range(n_instances):
        inst = generate_instance(seed=seed_base+i, **gen_kwargs)
        res = evaluate_instance(inst)
        experiments.append((inst,res))
    return experiments

def summarize_and_save(experiments, filename="results.csv"):
    rows = []
    for idx,(inst,res) in enumerate(experiments):
        row = {
            "instance": idx,
            "seed": inst.get("seed"),
            "n_targets": len(inst["targets"]),
            "targets": ";".join(f"{t[0]}-{t[1]}" for t in inst["targets"]),
            "recharge_stations": ";".join(f"{r[0]}-{r[1]}" for r in inst["recharge_stations"]),
            "obstacles_count": len(inst["obstacles"]),
            "max_battery": inst.get("max_battery"),
            "recharge_time": inst.get("recharge_time")
        }
        # UCS fields (safe .get)
        ucs = res["ucs"]
        ast = res["astar"]
        row.update({
            "ucs_success": ucs.get("success", False),
            "ucs_cost": ucs.get("cost"),
            "ucs_nodes": ucs.get("nodes"),
            "ucs_time": ucs.get("time"),
            "ucs_explored_states": ucs.get("explored_states"),
            "ucs_max_fringe": ucs.get("max_fringe_size"),
            "ucs_reason": ucs.get("reason"),
            "astar_success": ast.get("success", False),
            "astar_cost": ast.get("cost"),
            "astar_nodes": ast.get("nodes"),
            "astar_time": ast.get("time"),
            "astar_explored_states": ast.get("explored_states"),
            "astar_max_fringe": ast.get("max_fringe_size"),
            "astar_reason": ast.get("reason")
        })
        rows.append(row)
    df = pd.DataFrame(rows)
    # For plotting, replace None with NaN / -1 (choose -1 so plots show)
    plot_df = df.copy()
    plot_df['ucs_cost'] = plot_df['ucs_cost'].astype('float').fillna(-1)
    plot_df['astar_cost'] = plot_df['astar_cost'].astype('float').fillna(-1)
    plot_df['ucs_nodes'] = plot_df['ucs_nodes'].astype('float').fillna(-1)
    plot_df['astar_nodes'] = plot_df['astar_nodes'].astype('float').fillna(-1)
    plot_df['ucs_time'] = plot_df['ucs_time'].astype('float').fillna(-1)
    plot_df['astar_time'] = plot_df['astar_time'].astype('float').fillna(-1)

    df.to_csv(filename, index=False)
    print(df)
    print(f"\nSaved results to {filename}")
    return df, plot_df

# -------------------------
# Main
# -------------------------
def main():
    print("Running experiments...")
    exps = run_experiments(n_instances=6, seed_base=42, rows=10, cols=10, n_targets=3, obstacle_prob=0.12,
                           n_recharge=2, max_battery=10, recharge_time=5)
    # demo first instance
    inst,res = exps[0]
    print("\nFirst instance summary:")
    print(f" Targets: {inst['targets']}")
    print(f" Recharges: {inst['recharge_stations']}")
    print(f" Obstacles count: {len(inst['obstacles'])}")
    ucs_cost = res['ucs'].get('cost')
    ucs_nodes = res['ucs'].get('nodes')
    ast_cost = res['astar'].get('cost')
    ast_nodes = res['astar'].get('nodes')
    print(f"UCS path cost: {ucs_cost}, nodes: {ucs_nodes}, success: {res['ucs'].get('success')}, reason: {res['ucs'].get('reason')}")
    print(f"A* path cost: {ast_cost}, nodes: {ast_nodes}, success: {res['astar'].get('success')}, reason: {res['astar'].get('reason')}")

    df, plot_df = summarize_and_save(exps)

    # --- Plotting ---
    # Nodes expanded
    plt.figure(figsize=(8,4))
    plt.plot(plot_df['instance'], plot_df['ucs_nodes'], marker='o', label='UCS')
    plt.plot(plot_df['instance'], plot_df['astar_nodes'], marker='s', label='A*')
    plt.xlabel('Instance'); plt.ylabel('Nodes Expanded'); plt.title('Nodes Expanded per Instance')
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.savefig("nodes_expanded.png"); plt.show()

    # Runtime
    plt.figure(figsize=(8,4))
    plt.plot(plot_df['instance'], plot_df['ucs_time'], marker='o', label='UCS')
    plt.plot(plot_df['instance'], plot_df['astar_time'], marker='s', label='A*')
    plt.xlabel('Instance'); plt.ylabel('Time (s)'); plt.title('Runtime per Instance')
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.savefig("runtime.png"); plt.show()

    # Solution cost
    plt.figure(figsize=(8,4))
    plt.plot(plot_df['instance'], plot_df['ucs_cost'], marker='o', label='UCS')
    plt.plot(plot_df['instance'], plot_df['astar_cost'], marker='s', label='A*')
    plt.xlabel('Instance'); plt.ylabel('Solution Cost'); plt.title('Solution Cost per Instance')
    plt.legend(); plt.grid(True); plt.tight_layout(); plt.savefig("solution_cost.png"); plt.show()

    print("\nGraphs saved: nodes_expanded.png, runtime.png, solution_cost.png")

if __name__ == "__main__":
    main()