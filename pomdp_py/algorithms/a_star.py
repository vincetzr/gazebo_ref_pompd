"""Implementation of the A* algorithm for the gridworld environment.
See https://leetcode.com/problems/shortest-path-in-binary-matrix/solutions/313347/A*-search-in-Python/."""

from heapq import heappush, heappop
import math

class PriorityQueue:
    
    def __init__(self, iterable=[]):
        self.heap = []
        for value in iterable:
            heappush(self.heap, (0, value))
    
    def add(self, value, priority=0):
        heappush(self.heap, (priority, value))
    
    def pop(self):
        priority, value = heappop(self.heap)
        return value
    
    def __len__(self):
        return len(self.heap)
    
class AStar():
    
    def __init__(self, gridworld):
        self.all_actions = gridworld.agent.all_actions
        self.grid_map = gridworld.grid_map
        self.successor_function = self.get_successor_function()
        self.heuristic = self.get_heuristic()
        self.goal_function = self.get_goal_function()
        # self.init_node = Node(gridworld.init_state.position)
        # self.goal_node = Node(gridworld.goal_state.position)
            
    def a_star_search(self, start):
        
        # open set
        frontier = PriorityQueue()
        frontier.add(start)
        
        # closed set
        visited = set()
        
        # came_from[n] is the node immediately preceding n 
        # on the cheapest known path from start to n
        came_from = dict()
        
        # i.e. g[n] is the cost of cheapest known path from start to n
        distance = {start: 0}

        while frontier:
            node = frontier.pop()
            if node in visited:
                continue
            if self.goal_function(node):
                return self.reconstruct_path(came_from, start, node)
            visited.add(node)
            for successor in self.successor_function(node):
                frontier.add(
                    successor,
                    priority = distance[node] + 1 + self.heuristic(successor)
                )
                if (successor not in distance
                    or distance[node] + 1 < distance[successor]):
                    distance[successor] = distance[node] + 1
                    came_from[successor] = node
        return None

    def reconstruct_path(self, came_from, start, end):
        """
        >>> came_from = {'b': 'a', 'c': 'a', 'd': 'c', 'e': 'd', 'f': 'd'}
        >>> reconstruct_path(came_from, 'a', 'e')
        ['a', 'c', 'd', 'e']
        """
        reverse_path = [end]
        while end != start:
            end = came_from[end]
            reverse_path.append(end)
        return list(reversed(reverse_path))
    
    # Fully offline policy generator
    
    def a_star_policy(self, agent):
        """Generates an offline policy for every starting state."""
        actions = agent.all_actions
        states = agent.all_states
        
        policy = dict()
        for state in states:
            path = self.a_star_search(state.position)       
            if path is None:
                # i.e. cannot achieve the goal from current state.
                policy[state] = next(iter(agent.all_actions)) # retrieve an arbitrary item
            elif self.grid_map.at_goal(state.position):
                # policy[state] = 'G'
                policy[state] = next(iter(agent.all_actions)) # retrieve an arbitrary item
            else:
                motion = (path[1][0] - path[0][0], path[1][1] - path[0][1])
                policy[state] = [a for a in actions if a.motion == motion][0]
        
        return policy
    
    # Function constructors
    
    def get_goal_function(self):
        """
        >>> f = get_goal_function([[0, 0], [0, 0]])
        >>> f((0, 0))
        False
        >>> f((0, 1))
        False
        >>> f((1, 1))
        True
        """
        # m, n = self.grid_map.m, self.grid_map.n
        # def is_bottom_right(cell):
        #     return cell == (M-1, N-1)
        # return is_bottom_right
        
        return self.grid_map.at_goal
    
    def get_successor_function(self):
        """
        >>> f = get_successor_function([[0, 0, 0], [0, 1, 0], [1, 0, 0]])
        >>> sorted(f((1, 2)))
        [(0, 1), (0, 2), (2, 1), (2, 2)]
        >>> sorted(f((2, 1)))
        [(1, 0), (1, 2), (2, 2)]
        """
        def get_clear_adjacent_cells(cell):
            i, j = cell
            return (
                (i + a, j + b)
                for (a, b) in [(0,-1), (0,1), (1,0), (-1,0)]
                if (i + a, j + b) in (set(self.grid_map.state_positions) - set(self.grid_map.danger_zones))
            )
        return get_clear_adjacent_cells
    
    def get_heuristic(self):
        """
        >>> f = get_heuristic([[0, 0], [0, 0]])
        >>> f((0, 0))
        1
        >>> f((0, 1))
        1
        >>> f((1, 1))
        0
        """
        def get_clear_path_distance_from_goal(cell):
            distance_to_nearest_goal = math.inf
            for (a, b) in self.grid_map.goals:
                (i, j) = cell
                distance_to_nearest_goal = min(max(abs(a - i), abs(b - j)), distance_to_nearest_goal)
            return distance_to_nearest_goal
        return get_clear_path_distance_from_goal