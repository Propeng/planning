import numpy as np

# conv_matrix = np.array([
#     [0, 0.1, 0],
#     [0.0, 0.1, 0.0],
#     [0.0, 0.2, 0.0],
#     [0.1, 0.3, 0.1],
#     [0.2, 0.5, 0.2],
#     [0.2, 1, 0.2],
#     [0.1, 0.5, 0.1],
# ])
conv_matrix = np.array([
    [0, 0.1, 0],
    [0.0, 0.2, 0.0],
    [0.0, 0.3, 0.0],
    [0.1, 0.5, 0.1],
    [0.2, 0.8, 0.2],
    [0.2, 0.8, 0.2],
    [0.2, 0.8, 0.2],
    [0.2, 1, 0.2],
    [0.1, 0.5, 0.1],
])

def conv_function(x, y):
    #Offset function by (1,5) to align with matrix center
    x += 1
    y += 7

    if y < 0 or x < 0 or y >= len(conv_matrix) or x >= len(conv_matrix[y]):
        return 0
    return conv_matrix[y][x]

def apply_conv(waypoint_matrix):
    height = len(waypoint_matrix)
    width = max(len(row) for row in waypoint_matrix)

    cost_matrix = np.zeros((height, width))
    for y, row in enumerate(waypoint_matrix):
        for x, waypoint in enumerate(row):
            cost_matrix[y][x] = waypoint.cost
    
    new_cost_matrix = np.copy(cost_matrix)

    for y1 in range(height):
        for x1 in range(width):
            cost = cost_matrix[y1][x1]
            for y2 in range(height):
                for x2 in range(width):
                    conv_val = cost * conv_function(x2-x1, y2-y1)
                    new_cost_matrix[y2][x2] += conv_val
                    if new_cost_matrix[y2][x2] > 1.0:
                        new_cost_matrix[y2][x2] = 1.0

    for y in range(height):
        for x in range(width):
            if x < len(waypoint_matrix[y]):
                cost = new_cost_matrix[y][x]
                waypoint_matrix[y][x].cost = cost
