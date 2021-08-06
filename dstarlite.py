from heapq import heappop, heappush, heapify
from collections import defaultdict
import math

class DStarLite(object):
    lane_change_cost = 0.2

    def __init__(self, n_lanes, goal_waypoint):
        self.U = []
        self.k_m = 0
        self.rhs = defaultdict(lambda: float('inf'))
        self.g = defaultdict(lambda: float('inf'))
        self.s_start = self.s_last = (0, 0)
        self.costs = defaultdict(lambda: 0)

        self.n_lanes = n_lanes
        self.goals = [(goal_waypoint, lane) for lane in range(n_lanes)]
        for goal in self.goals:
            self.rhs[goal] = 0
            heappush(self.U, (self.calculate_key(goal), goal))
        
        self.costs_changed = False

        self.compute_shortest_path()
    
    def set_cost(self, s, cost):
        self.costs[s] = cost
        for u in self.pred(s):
            self.update_vertex(u)
        self.costs_changed = True
    
    def h(self, s1, s2):
        (wp1, _) = s1
        (wp2, _) = s2
        return abs(wp2 - wp1)# + abs(l2 - l1)
    
    def c(self, s1, s2):
        (wp1, l1) = s1
        (wp2, l2) = s2
        if abs(l2-l1) > 1:
            return float('inf')
        if wp2 <= wp1:
            return float('inf')
        c = self.costs[(wp2, l2)]
        c += abs(l2-l1) * self.lane_change_cost
        return c
    
    def calculate_key(self, s):
        return (min(self.g[s], self.rhs[s]) + self.h(self.goals[0], s) + self.k_m, min(self.rhs[s], self.g[s]))
    
    def succ(self, s):
        (waypoint, lane) = s
        if s in self.goals:
            return []
        succ = [(waypoint+1, lane)]
        if lane > 0:
            succ.append((waypoint+1, lane-1))
        if lane < self.n_lanes-1:
            succ.append((waypoint+1, lane+1))
        return succ
    
    def pred(self, s):
        (waypoint, lane) = s
        if waypoint == 0:
            return []
        pred = [(waypoint-1, lane)]
        if lane > 0:
            pred.append((waypoint-1, lane-1))
        if lane < self.n_lanes-1:
            pred.append((waypoint-1, lane+1))
        return pred
    
    def update_vertex(self, u):
        if not u in self.goals:
            self.rhs[u] = min(self.c(u, s) + self.g[s] for s in self.succ(u))
        for i, (_, s) in enumerate(self.U):
            if s == u:
                self.U[i] = self.U[-1]
                self.U.pop(i)
                heapify(self.U)
        if self.g[u] != self.rhs[u]:
            heappush(self.U, (self.calculate_key(u), u))
    
    def compute_shortest_path(self):
        #print('hi')
        #print(self.U)
        if len(self.U) == 0:
            return
        while len(self.U) > 0 and (self.U[0][0] < self.calculate_key(self.s_start) or self.rhs[self.s_start] != self.g[self.s_start]):
            k_old = self.U[0][0]
            (_, u) = heappop(self.U)
            if k_old < self.calculate_key(u):
                heappush(self.U, (self.calculate_key(u), u))
            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u] = float('inf')
                for s in self.pred(u):
                    self.update_vertex(s)
                self.update_vertex(u)
                #print("%s -> %s" % (u, self.pred(u)))
    
    def step(self, look_ahead=20):
        if math.isinf(self.g[self.s_start]):
            return []
        
        # s = self.s_start
        # for ss in self.succ(s):
        #     print(s, ss, self.g[ss])
        #     pass

        path = [self.s_start]
        for _ in range(look_ahead):
            s_last = path[-1]
            try:
                s_next = min((self.c(s_last, s) + self.g[s], s) for s in self.succ(s_last))
                if math.isinf(s_next[0]):
                    break
                path.append(s_next[1])
            except ValueError:
                #no more successors
                break
        
        if self.costs_changed:
            self.k_m += self.h(self.s_last, self.s_start)
            self.s_last = self.s_start
            self.compute_shortest_path()
            self.costs_changed = False
        
        return path
