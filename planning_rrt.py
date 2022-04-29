
import numpy as np
import networkx as nx

from queue import PriorityQueue
SQRT2 = np.sqrt(2)


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)

def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def a_star_graph(graph, h, start, goal):
    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:
            current_cost = branch[current_node][0]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                # get the tuple representation
                cost = graph.edges[current_node, next_node]['weight']

                branch_cost = current_cost + cost
                queue_cost = branch_cost + h(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    branch[next_node] = (branch_cost, current_node)
                    queue.put((queue_cost, next_node))

    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            node = branch[n][1]
            path.append((int(node[0]), int(node[1])))
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')
    return path[::-1], path_cost


class RRT:
    def __init__(self, x_init):
        # A tree is a special case of a graph with
        # directed edges and only one path to any vertex.
        self.tree = nx.DiGraph()
        self.tree.add_node(x_init)

    def add_vertex(self, x_new):
        self.tree.add_node(tuple(x_new))

    def add_edge(self, x_near, x_new, w):
        self.tree.add_edge(tuple(x_near), tuple(x_new), weight=w)

    @property
    def vertices(self):
        return self.tree.nodes()

    @property
    def edges(self):
        return self.tree.edges()

    @property
    def graph(self):
        return self.tree

def sample_state(grid):
    x = np.random.uniform(0, grid.shape[0])
    y = np.random.uniform(0, grid.shape[1])
    return (x, y)


def nearest_neighbor(x_rand, rrt):
    closest_dist = 100000
    closest_vertex = None
    x_rand = np.array(x_rand)

    for v in rrt.vertices:
        d = np.linalg.norm(x_rand - np.array(v[:2]))
        if d < closest_dist:
            closest_dist = d
            closest_vertex = v
    return closest_vertex

def select_input(x_rand, x_near):
    return np.arctan2(x_rand[1] - x_near[1], x_rand[0] - x_near[0])

def new_state(x_near, u, dt):
    nx = x_near[0] + np.cos(u)*dt
    ny = x_near[1] + np.sin(u)*dt
    return [nx, ny]


def generate_RRT(grid, x_init, num_vertices, dt):
    rrt = RRT(x_init)
    for _ in range(num_vertices):

        x_rand = sample_state(grid)
        # sample states until a free state is found
        while grid[int(x_rand[0]), int(x_rand[1])] == 1:
            x_rand = sample_state(grid)

        x_near = nearest_neighbor(x_rand, rrt)
        u = select_input(x_rand, x_near)
        x_new = new_state(x_near, u, dt)
        xn0 = int(x_new[0])
        if (xn0 >= grid.shape[0]):
            xn0 = grid.shape[0] - 1
        xn1 = int(x_new[1])
        if (xn1 >= grid.shape[1]):
            xn1 = grid.shape[1] - 1

        if grid[xn0, xn1] == 0:
            # the orientation `u` will be added as metadata to
            # the edge
            dist = np.linalg.norm(np.array(x_near) - np.array(x_new))
            rrt.add_edge(x_near, x_new, dist)
        # else :
        # print("sample, near, new",x_rand, x_near, x_new)
        # print("SKIPPED")

    return rrt