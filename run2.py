import sys
from collections import deque
import heapq


def min_steps_to_collect_all_keys(grid):
    H = len(grid)
    W = len(grid[0]) if H else 0

    robots = []
    keys = {}
    for i in range(H):
        for j in range(W):
            c = grid[i][j]
            if c == '@':
                robots.append((i, j))
            elif 'a' <= c <= 'z':
                keys[c] = (i, j)

    Rn = len(robots)
    Kn = len(keys)
    if Kn == 0:
        return 0

    key_list = sorted(keys)
    k2i = {k: idx for idx, k in enumerate(key_list)}
    ki2pos = {k2i[k]: pos for k, pos in keys.items()}

    total_nodes = Kn + Rn
    robot_offsets = {Kn + r: robots[r] for r in range(Rn)}

    dist_free = {n: {} for n in range(total_nodes)}
    for node in range(total_nodes):
        sx, sy = ki2pos[node] if node < Kn else robot_offsets[node]
        seen = [[False] * W for _ in range(H)]
        dq = deque([(sx, sy, 0)])
        seen[sx][sy] = True
        while dq:
            x, y, d = dq.popleft()
            ch = grid[x][y]
            if 'a' <= ch <= 'z':
                dist_free[node][k2i[ch]] = d
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if 0 <= nx < H and 0 <= ny < W and not seen[nx][ny] and grid[nx][ny] != '#':
                    seen[nx][ny] = True
                    dq.append((nx, ny, d + 1))

    graph = {n: {} for n in range(total_nodes)}
    for node in range(total_nodes):
        sx, sy = ki2pos[node] if node < Kn else robot_offsets[node]
        visited = {(sx, sy): [0]}
        dq = deque([(sx, sy, 0, 0)])
        while dq:
            x, y, d, mask = dq.popleft()
            ch = grid[x][y]
            if 'a' <= ch <= 'z':
                ki = k2i[ch]
                if ki != node:
                    graph[node].setdefault(ki, []).append((mask, d))
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < H and 0 <= ny < W): continue
                if grid[nx][ny] == '#': continue
                m2 = mask
                cc = grid[nx][ny]
                if 'A' <= cc <= 'Z':
                    m2 |= 1 << k2i[cc.lower()]
                p = (nx, ny)
                skip = False
                if p in visited:
                    for old in visited[p]:
                        if old | m2 == m2:
                            skip = True
                            break
                if skip:
                    continue
                if p in visited:
                    visited[p] = [o for o in visited[p] if not (m2 | o == o)]
                    visited[p].append(m2)
                else:
                    visited[p] = [m2]
                dq.append((nx, ny, d + 1, m2))
        for ki, lst in list(graph[node].items()):
            lst.sort(key=lambda x: x[1])
            keep = []
            for m, d in lst:
                dominated = False
                for pm, pd in keep:
                    if (pm | m) == m and pd <= d:
                        dominated = True
                        break
                if dominated:
                    continue
                keep = [(pm, pd) for pm, pd in keep if not ((m | pm) == pm and d <= pd)]
                keep.append((m, d))
            graph[node][ki] = keep

    ALL = (1 << Kn) - 1
    start = tuple(sorted(Kn + r for r in range(Rn)))
    pq = [(0, 0, start, 0)]
    best = {(start, 0): 0}

    def mst(pos_tuple, kmask):
        rem = [i for i in range(Kn) if not (kmask & (1 << i))]
        if not rem:
            return 0
        conn = set(pos_tuple)
        md = {i: float('inf') for i in rem}
        for i in rem:
            for v in conn:
                if v in dist_free and i in dist_free[v]:
                    d = dist_free[v][i]
                    if d < md[i]:
                        md[i] = d
        cost = 0
        while rem:
            nxt = min(rem, key=lambda x: md[x])
            if md[nxt] == float('inf'):
                return float('inf')
            cost += md[nxt]
            conn.add(nxt)
            rem.remove(nxt)
            for j in rem:
                if nxt in dist_free and j in dist_free[nxt]:
                    d = dist_free[nxt][j]
                    if d < md[j]:
                        md[j] = d
        return cost

    while pq:
        f, g, pos, km = heapq.heappop(pq)
        if best.get((pos, km), 1e18) != g:
            continue
        if km == ALL:
            return g
        for ridx, nid in enumerate(pos):
            for tgt, opts in graph[nid].items():
                bit = 1 << tgt
                if km & bit:
                    continue
                for req, d in opts:
                    if req & ~km:
                        continue
                    nk = km | bit
                    new_pos = list(pos)
                    new_pos[ridx] = tgt
                    new_pos = tuple(sorted(new_pos))
                    ng = g + d
                    st = (new_pos, nk)
                    if ng < best.get(st, 1e18):
                        best[st] = ng
                        h = mst(new_pos, nk)
                        heapq.heappush(pq, (ng + h, ng, new_pos, nk))


def main():
    grid = [list(line.rstrip('\n')) for line in sys.stdin]
    print(min_steps_to_collect_all_keys(grid))


if __name__ == '__main__':
    main()
