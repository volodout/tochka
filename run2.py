import sys
from collections import deque
import heapq


def min_steps_to_collect_all_keys(grid):
    height = len(grid)
    width = len(grid[0]) if height > 0 else 0

    start_positions = []
    key_positions = {}
    for i in range(height):
        for j in range(width):
            cell = grid[i][j]
            if cell == '@':
                start_positions.append((i, j))
            elif cell.islower():
                key_positions[cell] = (i, j)
    num_keys = len(key_positions)
    if num_keys == 0:
        return 0

    keys_sorted = sorted(key_positions.keys())
    letter_to_bit = {letter: idx for idx, letter in enumerate(keys_sorted)}
    bit_to_letter = {idx: letter for letter, idx in letter_to_bit.items()}

    points = []
    for pos in start_positions:
        points.append(pos)
    for letter in keys_sorted:
        points.append(key_positions[letter])
    point_index = {pos: idx for idx, pos in enumerate(points)}

    def bfs(start_idx):
        sx, sy = points[start_idx]
        visited = {(sx, sy): [0]}
        queue = deque([(sx, sy, 0, 0)])
        results = {}

        while queue:
            x, y, req_mask, dist = queue.popleft()
            cell = grid[x][y]
            if cell.islower():
                key_bit = letter_to_bit[cell]
                if point_index[(x, y)] != start_idx:
                    results.setdefault(key_bit, []).append((req_mask, dist))
            for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
                nx, ny = x + dx, y + dy
                if not (0 <= nx < height and 0 <= ny < width):
                    continue
                cell = grid[nx][ny]
                if cell == '#':
                    continue
                new_mask = req_mask
                if cell.isupper():
                    door_bit = letter_to_bit[cell.lower()]
                    new_mask |= (1 << door_bit)
                if (nx, ny) not in visited:
                    visited[(nx, ny)] = [new_mask]
                    queue.append((nx, ny, new_mask, dist + 1))
                else:
                    skip = False
                    for mask_old in visited[(nx, ny)]:
                        if mask_old | new_mask == new_mask:
                            skip = True
                            break
                    if skip:
                        continue
                    visited[(nx, ny)].append(new_mask)
                    queue.append((nx, ny, new_mask, dist + 1))
        return results

    reachable = [None] * len(points)
    for idx in range(len(points)):
        reachable[idx] = bfs(idx)

    start_state_positions = tuple(
        sorted(point_index[pos] for pos in start_positions))
    start_keys_mask = 0
    target_mask = (1 << num_keys) - 1
    pq = [(0, start_state_positions, start_keys_mask)]
    visited_states = {(start_state_positions, start_keys_mask): 0}

    while pq:
        dist, positions, keys_mask = heapq.heappop(pq)
        if visited_states[(positions, keys_mask)] < dist:
            continue
        if keys_mask == target_mask:
            return dist
        for robot_idx, pos_idx in enumerate(positions):
            for key_bit, paths in reachable[pos_idx].items():
                if keys_mask & (1 << key_bit):
                    continue
                for req_mask, step_dist in paths:
                    if req_mask & keys_mask == req_mask:
                        new_keys_mask = keys_mask | (1 << key_bit)
                        target_pos_idx = point_index[key_positions[bit_to_letter[key_bit]]]
                        new_positions = list(positions)
                        new_positions[robot_idx] = target_pos_idx
                        new_positions = tuple(sorted(new_positions))
                        new_dist = dist + step_dist
                        state = (new_positions, new_keys_mask)
                        if state not in visited_states or visited_states[state] > new_dist:
                            visited_states[state] = new_dist
                            heapq.heappush(pq, (new_dist, new_positions, new_keys_mask))


def main():
    grid = [list(line.rstrip('\n')) for line in sys.stdin]
    print(min_steps_to_collect_all_keys(grid))


if __name__ == '__main__':
    main()
