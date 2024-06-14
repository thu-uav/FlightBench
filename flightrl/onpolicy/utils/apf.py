import numpy as np
from queue import deque
import pyastar2d
from onpolicy.envs.habitat.utils.frontier import add_clear_disk, l2distance

# class of APF(Artificial Potential Field)
class APF(object):
    def __init__(self, args):
        self.args = args

        self.cluster_radius = args.ft_cluster_radius
        self.k_attract = args.apf_k_attract
        self.k_agents = args.apf_k_agents
        self.AGENT_INFERENCE_RADIUS = args.apf_AGENT_INFERENCE_RADIUS
        self.num_iters = args.apf_num_iters
        self.repeat_penalty = args.apf_repeat_penalty
        self.dis_type = args.apf_dis_type
        self.use_random = args.apf_use_random
        self.clear_radius = args.ft_clear_radius

        self.num_agents = args.num_agents

        self.num_clusters = args.apf_num_clusters

    def distance(self, a, b):
        a = np.array(a)
        b = np.array(b)
        if self.dis_type == "l2":
            return np.sqrt(((a-b)**2).sum())
        elif self.dis_type == "l1":
            return abs(a-b).sum()

    def schedule(self, map, unexplored, locations, steps, agent_id, penalty = None, full_path = True, clear_disk = False, random_goal = False):
        '''
        APF to schedule path for agent agent_id
        map: H x W
            - 0 for explored & available cell
            - 1 for obstacle
            - 2 for target (frontier)
        locations: num_agents x 2
        steps: available actions
        penalty: repeat penalty
        full_path: default True, False for single step (i.e., next cell)
        '''
        H, W = map.shape
        
        # find available targets
        vis = np.zeros((H,W), dtype = np.uint8)
        que = deque([])
        x, y = locations[agent_id]
        locx, locy = x, y
        vis[x,y] = 1
        que.append((x,y))
        sdis = np.zeros((H, W), dtype = np.uint8)
        random_targets = []
        while len(que)>0:
            x, y = que.popleft()
            for dx, dy in steps:
                x1 = x + dx
                y1 = y + dy
                if vis[x1,y1] == 0 and map[x1,y1] in [0,2]:
                    vis[x1,y1] = 1
                    sdis[x1, y1] = sdis[x,y] + 1
                    que.append((x1, y1))
                    if sdis[x1,y1] > self.clear_radius*2 and sdis[x1,y1] <self.clear_radius *4:
                        random_targets.append((x1,y1))
        
        targets = []
        if clear_disk:
            map, unexplored = add_clear_disk(map, unexplored, self.clear_radius, (locx, locy))
            near = []
            max_dist = -1
            for i in range(H):
                for j in range(W):
                    if map[i,j] == 2 and vis[i,j] == 1:
                        if  l2distance((i,j), (locx,locy))>self.clear_radius:
                            targets.append((i,j))
                        else:
                            near.append((i,j))
                            if self.distance((i,j), (locx,locy)) > max_dist:
                                max_dist = self.distance((i,j), (locx,locy))
        else:
            for i in range(H):
                for j in range(W):
                    if map[i,j] == 2 and vis[i,j] == 1:
                        targets.append((i,j))
        # print("Number of targets", len(targets))
        # clustering
        clusters = []
        num_targets = len(targets)
        valid = [True for _ in range(num_targets)]
        t_targets = []
        for i in range(num_targets):
            if valid[i]:
                # not clustered
                chosen_targets = []
                for j in range(num_targets):
                    if valid[j] and self.distance(targets[i], targets[j]) <= self.cluster_radius:
                        valid[j] = False
                        chosen_targets.append(targets[j])
                min_r = 1e9
                center = None
                for a in chosen_targets:
                    max_d = max([self.distance(a,b) for b in chosen_targets])
                    if max_d < min_r:
                        min_r = max_d
                        center = a
                if len(chosen_targets) >= 3:
                    clusters.append({"center": center, "weight": len(chosen_targets)})
                    for t in chosen_targets:
                        t_targets.append(t)
        # targets = t_targets
        num_targets = len(targets)
        
        # potential
        num_clusters = len(clusters)
        if num_clusters == 0:
            for t in targets:
                clusters.append({'center':t, 'weight':1})
            num_clusters = len(clusters)
        potential = np.zeros((H, W))
        potential[map == 1] = 1e9

        for i in range(num_clusters):
            for j in range(num_clusters):
                if i<j and clusters[i]['weight']/(sdis[clusters[i]['center'][0]][clusters[i]['center'][1]]+1) < clusters[j]['weight']/(sdis[clusters[j]['center'][0]][clusters[j]['center'][1]]+1):
                    tmp = clusters[i]
                    clusters[i] = clusters[j]
                    clusters[j] = tmp
        if num_clusters > self.num_clusters:
            num_clusters = self.num_clusters
            clusters = clusters[:num_clusters]
        # print("num clusters:", num_clusters)

        # potential of targets & obstacles (wave-front dist)
        for cluster in clusters:
            sx, sy = cluster["center"]
            w = cluster["weight"]
            dis = np.ones((H, W), dtype=np.int64) * 1e9
            dis[sx, sy] = 0
            que = deque([(sx, sy)])
            while len(que) > 0:
                (x, y) = que.popleft()
                for dx, dy in steps:
                    x1 = x + dx
                    y1 = y + dy
                    if dis[x1, y1] == 1e9 and map[x1, y1] in [0,2]:
                        dis[x1, y1] = dis[x, y]+1
                        que.append((x1,y1))
            dis[sx, sy] = 1e9
            dis = 1 / dis
            dis[sx, sy] = 0
            potential[map != 1] -= dis[map != 1] * self.k_attract * w 

        # potential of agents
        for x in range(H):
            for y in range(W):
                for agent_loc in locations:
                    d = self.distance(agent_loc, (x,y))
                    if d <= self.AGENT_INFERENCE_RADIUS:
                        potential[x,y] += self.k_agents * (self.AGENT_INFERENCE_RADIUS - d) 
        
        # manual penalty (repeat penalty, etc.)
        if type(penalty) != type(None):
            potential += penalty

        # print("    %d targets"%num_targets)

        # schedule path
        it = 1
        current_loc = locations[agent_id]
        current_potential = 1e4
        minDis2Target = 1e9
        path = [(current_loc[0], current_loc[1])]
        while it <= self.num_iters and minDis2Target > 1:
            it = it + 1
            potential[current_loc[0], current_loc[1]] += self.repeat_penalty
            # print("    it : {}, loc : ({},{}), potential: {}".format(it, current_loc[0], current_loc[1], potential[current_loc[0], current_loc[1]]))
            best_neigh = None
            min_potential = 1e9
            for dx, dy in steps:
                neighbor_loc = (current_loc[0] + dx, current_loc[1] + dy)
                if map[neighbor_loc[0], neighbor_loc[1]] == 1:
                    continue
                if min_potential > potential[neighbor_loc[0], neighbor_loc[1]]:
                    min_potential = potential[neighbor_loc[0], neighbor_loc[1]]
                    best_neigh = neighbor_loc
            if current_potential > min_potential:
                current_potential = min_potential
                current_loc = best_neigh
                path.append(best_neigh)
            for tar in targets:
                l = self.distance(current_loc, tar)
                if l == 0:
                    continue
                minDis2Target = min(minDis2Target, l)
                if l<=1:
                    path.append((tar[0], tar[1]))
                    break
            if not full_path and len(path)>1:
                return path[1] # next grid
        # print("    Iters %d, Goal (%d, %d)"%(it, path[-1][0], path[-1][1]))
        random_plan = False
        if minDis2Target > 1:
            # random_plan = True
            random_plan = (l2distance(locations[agent_id], path[-1]) <= self.clear_radius)
        for i in range(agent_id):
            if locations[i][0]==locations[agent_id][0] and locations[i][1]==locations[agent_id][1]:
                random_plan = True # two agents are at the same location, replan
        random_plan = random_plan and self.use_random
        if random_plan and random_goal:
            # if not reaching a frontier, randomly pick a traget as goal
            if len(random_targets) == 0:
                random_targets.append((np.random.randint(0,H), np.random.randint(0,W)))
            w = np.random.randint(0, len(random_targets))
            path = (locations[agent_id], random_targets[w])
            # print("    random plan, pick goal (%d, %d)"%(random_targets[w][0], random_targets[w][1]))
        return path