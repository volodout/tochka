import sys
from collections import deque, defaultdict
import heapq


def min_steps_to_collect_all_keys(grid):
    n, m = len(grid), len(grid[0])
    robots = []
    key_positions = {}
    for i in range(n):
        for j in range(m):
            c = grid[i][j]
            if c == '@':
                robots.append((i, j))
            elif 'a' <= c <= 'z':
                key_positions[c] = (i, j)
    K = len(key_positions)
    if K == 0:
        return 0

    keys = sorted(key_positions.keys())
    key_index = {k: idx for idx, k in enumerate(keys)}

    sources = robots + [key_positions[k] for k in keys]
    S = len(sources)

    dist = [dict() for _ in range(S)]
    req = [dict() for _ in range(S)]
    door_bit = {k.upper(): 1 << key_index[k] for k in keys}

    for s, (si, sj) in enumerate(sources):
        q = deque([(si, sj, 0, 0)])
        seen = set([(si, sj)])
        while q:
            i, j, d, doors_mask = q.popleft()
            c = grid[i][j]
            if 'a' <= c <= 'z':
                ki = key_index[c]
                dist[s][ki] = d
                req[s][ki] = doors_mask
            for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ni, nj = i + di, j + dj
                if not (0 <= ni < n and 0 <= nj < m): continue
                if (ni, nj) in seen: continue
                ch = grid[ni][nj]
                if ch == '#': continue
                new_mask = doors_mask
                if 'A' <= ch <= 'Z' and ch in door_bit:
                    new_mask |= door_bit[ch]
                seen.add((ni, nj))
                q.append((ni, nj, d + 1, new_mask))

    ALL_KEYS = (1 << K) - 1

    init_pos = tuple(range(4))
    min_dist = {}
    heap = [(0, init_pos, 0)]
    min_dist[(init_pos, 0)] = 0

    while heap:
        steps, positions, mask = heapq.heappop(heap)
        if min_dist.get((positions, mask), 1 << 60) < steps:
            continue
        if mask == ALL_KEYS:
            return steps

        for r in range(4):
            src = positions[r]
            for ki, d in dist[src].items():
                bit = 1 << ki
                if mask & bit:
                    continue
                if req[src][ki] & ~mask:
                    continue
                new_pos = list(positions)
                new_pos[r] = 4 + ki
                new_pos = tuple(new_pos)
                new_mask = mask | bit
                new_steps = steps + d
                state = (new_pos, new_mask)
                if new_steps < min_dist.get(state, 1 << 60):
                    min_dist[state] = new_steps
                    heapq.heappush(heap, (new_steps, new_pos, new_mask))

    return -1


def main():
    grid = [list(line.rstrip('\n')) for line in sys.stdin]
    print(min_steps_to_collect_all_keys(grid))


if __name__ == '__main__':
    main()
