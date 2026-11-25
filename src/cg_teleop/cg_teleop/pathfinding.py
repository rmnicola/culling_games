"""Find a route through a Culling Games maze with breadth-first search."""

from collections import deque


MOVES = (
    ('up', -1, 0),
    ('down', 1, 0),
    ('left', 0, -1),
    ('right', 0, 1),
)


def shortest_path(flattened_map, shape):
    """Return (direction, destination) steps from r to t, or None if blocked."""
    rows, cols = shape
    if rows < 1 or cols < 1 or len(flattened_map) != rows * cols:
        raise ValueError('Invalid map dimensions')

    grid = [flattened_map[row * cols:(row + 1) * cols]
            for row in range(rows)]
    starts = [(row, col) for row in range(rows) for col in range(cols)
              if grid[row][col] == 'r']
    targets = [(row, col) for row in range(rows) for col in range(cols)
               if grid[row][col] == 't']
    if len(starts) != 1 or len(targets) != 1:
        raise ValueError('Map must have exactly one robot and one target')

    start, target = starts[0], targets[0]
    queue = deque([start])
    previous = {start: None}

    while queue:
        current = queue.popleft()
        if current == target:
            break
        for direction, row_change, col_change in MOVES:
            neighbor = (current[0] + row_change, current[1] + col_change)
            row, col = neighbor
            if (0 <= row < rows and 0 <= col < cols and
                    grid[row][col] in ('f', 't') and
                    neighbor not in previous):
                previous[neighbor] = (current, direction)
                queue.append(neighbor)

    if target not in previous:
        return None

    path = []
    position = target
    while position != start:
        parent, direction = previous[position]
        path.append((direction, position))
        position = parent
    path.reverse()
    return path
