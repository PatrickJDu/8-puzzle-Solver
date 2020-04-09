import time
import heapq
from collections import defaultdict
import sys
from math import floor


class State:
    """
    This State class represents the state of a node
    For example
    [1, 2, 3, 5, 4, 0, 6, 7, 8] is what state represents
    the hash is used to  achieve O(1) operation
    Depth is 0 if it's root otherwise keep adding one as we transition to child
    Everything else should be self explainatory
    """

    def __init__(self, problem, parent):
        self.hash = ""
        self.problem = problem
        self.parent = parent
        if parent is None:
            self.depth = 0
        else:
            self.depth = parent.depth + 1
        self.cost = self.manhattan_dist() + self.depth

    def __lt__(self, other):
        return other.cost > self.cost

    def hash_(self):
        """
        A string hash that is used for O(1) operation
        :return:
        """
        self.hash = "".join(self.problem)
        return self.hash

    """
    These are the move functions that dictate how the blank (or 0) can move
    """

    def move_up(self, state):
        pos = state.problem.index("0")
        if pos in (0, 1, 2):
            return None
        else:
            child = list(state.problem)
            temp = child[pos - 3]
            child[pos - 3] = "0"
            child[pos] = temp
            return child

    def move_right(self, state):
        pos = state.problem.index("0")
        if pos in (2, 5, 8):
            return None
        else:
            child = list(state.problem)
            temp = child[pos + 1]
            child[pos + 1] = "0"
            child[pos] = temp
            return child

    def move_down(self, state):
        pos = state.problem.index("0")
        if pos in (6, 7, 8):
            return None
        else:
            child = list(state.problem)
            temp = child[pos + 3]
            child[pos + 3] = "0"
            child[pos] = temp
            return child

    def move_left(self, state):
        pos = state.problem.index("0")
        if pos in (0, 3, 6):
            return None
        else:
            child = list(state.problem)
            temp = child[pos - 1]
            child[pos - 1] = "0"
            child[pos] = temp
            return child

    def get_successors(self, search_alg):
        """
        This method dictates how each search will search.
        :param search_alg:
        :return:
        """
        successors = []
        up = self.move_up(self)
        right = self.move_right(self)
        down = self.move_down(self)
        left = self.move_left(self)
        # This is used by bfs
        if search_alg == "bfs":
            if up is not None:
                successors.append(State(up, self))
            if down is not None:
                successors.append(State(down, self))
            if left is not None:
                successors.append(State(left, self))
            if right is not None:
                successors.append(State(right, self))
            return successors
        # This path is used by dfs
        elif search_alg == "dfs":
            if right is not None:
                successors.append(State(right, self))
            if left is not None:
                successors.append(State(left, self))
            if down is not None:
                successors.append(State(down, self))
            if up is not None:
                successors.append(State(up, self))
            return successors
        # This path is used byt ast
        elif search_alg == "ast":
            if up is not None:
                successors.append(State(up, self))
            if down is not None:
                successors.append(State(down, self))
            if left is not None:
                successors.append(State(left, self))
            if right is not None:
                successors.append(State(right, self))
            return successors

    def manhattan_dist(self):
        """
        Attempt at gaining h2(n) or the manhattan distance
        :return:
        """
        h_two_sum = 0
        positions = defaultdict(int)
        for pos in range(len(self.problem)):
            if self.problem[pos] != "0":
                positions[int(self.problem[pos])] = pos
        for pos, idx in positions.items():
            h_two_sum += abs((idx % 3) - pos % 3)
            h_two_sum += abs((idx // 3) - pos // 3)
        return h_two_sum

    @staticmethod
    def trace_back(last_node):
        """
        This is used to get the path to the goal, by traversing backwards
        """
        back_track = last_node.parent
        back_track_after = last_node
        path = []
        while back_track:
            if back_track.move_up(back_track) == back_track_after.problem:
                path.append("UP")
            elif back_track.move_right(back_track) == back_track_after.problem:
                path.append("RIGHT")
            elif back_track.move_down(back_track) == back_track_after.problem:
                path.append("DOWN")
            elif back_track.move_left(back_track) == back_track_after.problem:
                path.append("LEFT")
            back_track = back_track.parent
            back_track_after = back_track_after.parent
        return path[::-1]

    @staticmethod
    def find_parent_pos(child_index):
        return int(floor((child_index - 1) / 2))


class Driver:
    """
    This class contains the search alogorithms along with some helper functions
    """

    def goal(self, current_pos):
        return current_pos == ["0", "1", "2", "3", "4", "5", "6", "7", "8"]

    def bfs(self, state, search_alg):
        """
        BFS implementation
        :param state:
        :param search_alg:
        :return:
        """
        # queue
        frontier = []
        # Used to store the hash value of our state to maintain O(1) operation in traversal of successors
        frontier_set = set()
        # Has state been seen?
        explored = set()
        local_max_depth = 0
        local_nodes_expanded = 0
        frontier.append(state)
        frontier_set.add(state.hash_())
        while frontier:
            current_state = frontier.pop(0)
            frontier_set.discard(current_state.hash_())
            explored.add(current_state.hash_())
            if self.goal(current_state.problem):
                return current_state, local_max_depth, local_nodes_expanded
            local_nodes_expanded += 1
            for succ in current_state.get_successors(search_alg):
                if not (succ.hash_() in frontier_set or succ.hash_() in explored):
                    frontier.append(succ)
                    frontier_set.add(succ.hash_())
                    local_max_depth = max(local_max_depth, succ.depth)
        return "Failed"

    def dfs(self, state, search_alg):
        """
        DFS implementation
        :param state:
        :param search_alg:
        :return:
        """
        # queue
        frontier = []
        # Used to store the hash value of our state to maintain O(1) operation in traversal of successors
        frontier_set = set()
        # Has state been seen
        explored = set()
        local_max_depth = 0
        local_nodes_expanded = 0
        frontier.append(state)
        frontier_set.add(state.hash_())
        while frontier:
            current_state = frontier.pop()
            frontier_set.discard(current_state.hash_())
            explored.add(current_state.hash_())
            if self.goal(current_state.problem):
                return current_state, local_max_depth, local_nodes_expanded
            local_nodes_expanded += 1
            for succ in current_state.get_successors(search_alg):
                if not (succ.hash_() in frontier_set or succ.hash_() in explored):
                    frontier.append(succ)
                    frontier_set.add(succ.hash_())
                    local_max_depth = max(local_max_depth, succ.depth)
        return "Failed"

    def ast(self, state, search_alg):
        """
        A* implementation
        :param state:
        :param search_alg:
        :return:
        """
        # Queue turned heap
        frontier = []
        # Used to store the hash value of our state to maintain O(1) operation in traversal of successors
        frontier_set = set()
        # Have we seen the state?
        explored = set()
        local_max_depth = 0
        local_nodes_expanded = 0
        frontier.append(state)
        frontier_set.add(state.hash_())
        while frontier:
            current_state = heapq.heappop(frontier)
            frontier_set.discard(current_state.hash_)
            explored.add(current_state.hash_())
            if self.goal(current_state.problem):
                print("Success")
                return current_state, local_max_depth, local_nodes_expanded
            local_nodes_expanded += 1
            for succ in current_state.get_successors(search_alg):
                if not (succ.hash_() in frontier_set or succ.hash_() in explored):
                    heapq.heappush(frontier, succ)
                    frontier_set.add(succ.hash_())
                    local_max_depth = max(local_max_depth, succ.depth)
                # decrease_key function
                elif succ.hash_() in frontier_set:
                    for pos in frontier:
                        # if we found a duplicate set the value to the successor under the assumption it is smaller
                        if pos.hash_() == succ.hash_():
                            idx = frontier.index(pos)
                            # swap parent and child
                            while idx != 0 and frontier[State.find_parent_pos(idx)] > frontier[idx]:
                                frontier[idx] = succ
                                frontier[idx], frontier[State.find_parent_pos(idx)] = \
                                    (frontier[State.find_parent_pos(idx)], frontier[idx])
        print("Failed")


search_type = sys.argv[1]
start = sys.argv[2].split(",")
start_time = time.time()
s = State(start, None)
d = Driver()
if search_type == "dfs":
    final_node, max_depth, nodes_expanded = d.dfs(s, search_type)
elif search_type == "bfs":
    final_node, max_depth, nodes_expanded = d.bfs(s, search_type)
else:
    final_node, max_depth, nodes_expanded = d.ast(s, search_type)

path = State.trace_back(final_node)
with open("output.txt", "w") as write_file:
    write_file.write("path_to goal: {}\n".format(path))
    write_file.write("cost_of_path: {}\n".format(len(path)))
    write_file.write("nodes_expanded: {}\n".format(nodes_expanded))
    write_file.write("search_depth: {}\n".format(final_node.depth))
    write_file.write("max_search_depth: {}\n".format(max_depth))
    write_file.write("running_time: {:.8f}\n".format(time.time() - start_time))
