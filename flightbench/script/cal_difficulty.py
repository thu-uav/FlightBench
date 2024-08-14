import open3d as o3d
import numpy as np
import os
import sys
import time
import copy
import matplotlib.pyplot as plt

class difficultyCalculator():
    def __init__(self, scene_path):
        # read pcd
        pcd_origin = o3d.io.read_point_cloud(os.path.join(scene_path, "env.ply"))
        self.pcd = pcd_origin.voxel_down_sample(voxel_size=0.15)
        self.pcd_tree = o3d.geometry.KDTreeFlann(self.pcd)
        self.pcd_np = np.asarray(self.pcd.points)

        # read path
        guide_path_file_name = os.listdir(scene_path)
        csv_num = 0
        for name in guide_path_file_name:
            if name[-4:] == ".csv":
                csv_num+=1
        self.guide_path = []
        self.total_len = 0.0
        for i in range(csv_num):
            f_name = os.path.join(scene_path, "roadmap_shortened_unique_path"+str(i)+"_0.csv")
            if os.path.exists(f_name):
                points = np.loadtxt(f_name, delimiter=',').reshape(-1, 6)
                self.guide_path.append(points)
                for d in range(points.shape[0]):
                    self.total_len+=np.linalg.norm(points[d, :3]-points[d, 3:])
            else:
                print("no topo path to gate ", i)
                exit(-1)
        
        # read direct path
        # f_name = os.path.join(scene_path, "direct.csv")
        # self.direct_len = 0.0
        # self.direct_path = []
        # if os.path.exists(f_name):
        #     points = np.loadtxt(f_name, delimiter=',').reshape(-1, 6)
        #     self.direct_path.append(points)
        #     for d in range(points.shape[0]):
        #         self.direct_len+=np.linalg.norm(points[d, :3]-points[d, 3:])
        # else:
        #     print("no direct path!")
        #     exit(-1)

        # calculate
        self.feasibility = 0.0
        self.visibility = 0.0
        self.MW = 0.0
        self.drone_radius = 0.15

    def batchSample(self, p_last, r_last, ph, K):
        # init sphere
        [k_, idx_, _] = self.pcd_tree.search_knn_vector_3d(ph, 1)
        r_tmp = min(np.linalg.norm(self.pcd_np[idx_]-ph) - self.drone_radius, 4.0)
        r_min = min(r_last, r_tmp)
        r_max = max(r_last, r_tmp)
        d = np.linalg.norm(ph - p_last)
        if r_tmp <= 0.0 or d < r_max - r_min or d > r_min+r_max:
            best_p = None
            best_r = None
            best_score = -np.inf
        else:
            h1 = r_min * (1-(r_min * r_min + d * d - r_max * r_max) / (2 * r_min * d))
            h2 = r_max * (1-(r_max * r_max + d * d - r_min * r_min) / (2 * r_max * d))
            V_inter = np.pi / 3 * (3*r_min-h1)*h1*h1 + np.pi / 3 * (3*r_max-h2)*h2*h2
            score = 0.7 * 4/3 * np.pi * np.power(r_tmp, 3) + 0.1 * V_inter - 0.4 * 1.0
            best_score = copy.deepcopy(score)
            best_p = ph.copy()
            best_r = copy.deepcopy(r_tmp)
        sigma_x = 1 / 4 * np.linalg.norm(p_last - ph)
        sigma_y = sigma_z = 2 * sigma_x
        sigma = np.array([sigma_x, sigma_y, sigma_z])
        ori_unit_vector = (ph-p_last) / np.linalg.norm(ph-p_last)
        k = np.cross(np.array([1, 0, 0]), ori_unit_vector)
        k = k / np.linalg.norm(k)
        theta = np.arccos(np.dot(np.array([1, 0, 0]), ori_unit_vector))
        for i in range(K):
            sample_body = np.random.randn(3) * sigma
            sample = sample_body * np.cos(theta) + np.cross(k, sample_body) * np.sin(theta) + k * np.dot(k, sample_body) *(1-np.cos(theta))
            p_cand = ph + sample
            [k_, idx_, _] = self.pcd_tree.search_knn_vector_3d(p_cand, 1)
            r_cand = min(np.linalg.norm(self.pcd_np[idx_]-p_cand) - self.drone_radius, 4.0)

            # if bad radius
            if r_cand <= 0.0:
                continue
            # if contain
            r_min = min(r_last, r_cand)
            r_max = max(r_last, r_cand)
            d = np.linalg.norm(p_cand - p_last)
            if d < r_max - r_min:
                continue
            # if no attach
            if d > r_min+r_max:
                continue
            h1 = r_min * (1-(r_min * r_min + d * d - r_max * r_max) / (2 * r_min * d))
            h2 = r_max * (1-(r_max * r_max + d * d - r_min * r_min) / (2 * r_max * d))
            V_inter = np.pi / 3 * (3*r_min-h1)*h1*h1 + np.pi / 3 * (3*r_max-h2)*h2*h2
            # print("p_cand:", p_cand)
            # print("ph:", ph)
            # print("norm:", np.linalg.norm(p_cand-ph))
            p_score = 0.35 * np.dot(ori_unit_vector, p_cand-ph) + 0.1 * np.linalg.norm(p_cand-ph-np.dot(ori_unit_vector, p_cand-ph) * ori_unit_vector)
            score = 0.7 * 4/3 * np.pi * np.power(r_cand, 3) + 0.1 * V_inter - p_score
            if score > best_score:
                best_score = copy.deepcopy(score)
                best_p = copy.deepcopy(p_cand)
                best_r = copy.deepcopy(r_cand)
        return best_p, best_r

    def generateCorridorAlongPath(self, path_to_cal):
        center_list = []
        r_list = []
        ph_list = []
        start_point = path_to_cal[0][0, :3]
        end_point = path_to_cal[-1][-1, 3:]
        p0 = start_point.copy()
        [k_, idx_, _] = self.pcd_tree.search_knn_vector_3d(p0, 1)
        r0 = min(max(0.0, np.linalg.norm(self.pcd_np[idx_]-p0) - self.drone_radius), 4.0)

        p_curr = p0
        r_curr = r0
        progress = 0.0
        center_list.append(p_curr.copy())
        r_list.append(copy.deepcopy(r_curr))

        # check if finish
        finish = np.linalg.norm(end_point-p_curr) < r_curr
        while not finish:
            # cal ph
            ph = p_curr.copy()
            gap = max(0.3, np.linalg.norm(ph - p_curr) / 2)
            while np.linalg.norm(ph - p_curr) < r_curr:
                progress += gap
                gap = max(0.3, 0.5 * gap)
                progress_remain = copy.deepcopy(progress)
                break_flag = False
                for path_id in range(len(path_to_cal)):
                    for piece_id in range(path_to_cal[path_id].shape[0]):
                        if progress_remain < np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:]):
                            break_flag = True
                            break
                        else:
                            progress_remain -= np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:])
                    if break_flag:
                        break

                ph = path_to_cal[path_id][piece_id, :3] + progress_remain / np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:]) * (path_to_cal[path_id][piece_id, 3:] - path_to_cal[path_id][piece_id, :3])

            # batch sample and append
            ph_list.append(ph)
            p_last = copy.deepcopy(p_curr)
            r_last = copy.deepcopy(r_curr)
            p_curr, r_curr = self.batchSample(p_last=p_last, r_last=r_last, ph = ph, K = 40)
            while type(p_curr) == type(None):
                p_curr, r_curr = self.batchSample(p_last=p_last, r_last=r_last, ph = ph, K = 40)
            center_list.append(p_curr.copy())
            r_list.append(copy.deepcopy(r_curr))
            finish = np.linalg.norm(end_point-p_curr) < r_curr

        # print("corridor num: ", len(center_list))
        fig, ax = plt.subplots()
        # ax.set_xlim(-10, 6)
        # ax.set_ylim(-7, 8)
        for g in range(len(path_to_cal)):
            for p in range(path_to_cal[g].shape[0]):
                plt.plot([path_to_cal[g][p, 0], path_to_cal[g][p, 3]], [path_to_cal[g][p, 1], path_to_cal[g][p, 4]], c = 'r')
        for i in range(len(center_list)):
            circle = plt.Circle(center_list[i][:2], r_list[i], color='blue', fill = False)
            plt.text(center_list[i][0], center_list[i][1], str(i))
            ax.add_patch(circle)
        for i in range(len(ph_list)):
            plt.scatter(ph_list[i][0], ph_list[i][1], color = 'green')

        plt.savefig("corridor.png")
            
        # fig = plt.figure(dpi = 150)
        # ax = fig.add_subplot(111, projection='3d')

        # # ax.set_xlim(-12, 8)
        # # ax.set_ylim(0, 4.5)
        # ax.set_zlim(0, 4.5)

        # for g in range(len(path_to_cal)):
        #     for p in range(path_to_cal[g].shape[0]):
        #         if g == 0 and p == 0:
        #             ax.plot([path_to_cal[g][p, 0], path_to_cal[g][p, 3]], [path_to_cal[g][p, 1], path_to_cal[g][p, 4]], [path_to_cal[g][p, 2], path_to_cal[g][p, 5]], c = 'r', label = "guide")
        #         else:
        #             ax.plot([path_to_cal[g][p, 0], path_to_cal[g][p, 3]], [path_to_cal[g][p, 1], path_to_cal[g][p, 4]], [path_to_cal[g][p, 2], path_to_cal[g][p, 5]], c = 'r')
        # for i in range(len(center_list)):
        #     ax.scatter(center_list[i][0], center_list[i][1], center_list[i][2], s = r_list[i]*1200, c = 'blue', alpha = 0.6)

        # plt.savefig("corridor.png")

        return center_list, r_list

    def calFeasibility(self):
        fea_list = []
        sensing_range = 4.0
        for i in range(8):
            centers, rs = self.generateCorridorAlongPath(self.guide_path)
            N = len(centers) // 2
            rs = sorted(rs)[:N]
            feasibility = 0.0
            for i in range(N):
                feasibility += 1/rs[i]
            feasibility = feasibility * sensing_range / (2 * N)
            fea_list.append(copy.deepcopy(feasibility))
        feasibility = min(fea_list)
        print("TO:", feasibility)
        return feasibility

    def calVisibility(self, path_to_cal):
        progress = 0.0
        vis_list = []
        sensing_range = 4.0
        half_fov = np.pi / 5
        interval = [half_fov, np.pi / 6, np.pi / 8, np.pi / 12, np.pi / 20, 0]
        while progress < self.total_len:
            progress +=  sensing_range / 10

            # get point with progress
            progress_remain = copy.deepcopy(progress)
            break_flag = False
            for path_id in range(len(path_to_cal)):
                for piece_id in range(path_to_cal[path_id].shape[0]):
                    if progress_remain < np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:]):
                        break_flag = True
                        break
                    else:
                        progress_remain -= np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:])
                if break_flag:
                    break

            pn = path_to_cal[path_id][piece_id, :3] + progress_remain / np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:]) * (path_to_cal[path_id][piece_id, 3:] - path_to_cal[path_id][piece_id, :3])
            ori = (path_to_cal[path_id][piece_id, 3:] - path_to_cal[path_id][piece_id, :3]) / np.linalg.norm(path_to_cal[path_id][piece_id, :3] - path_to_cal[path_id][piece_id, 3:])
            
            cos_thre = np.cos(np.array(interval))
            [k_, idx_, _] = self.pcd_tree.search_radius_vector_3d(pn, sensing_range) # sensing range

            nearest_dist = [np.inf for i in range(len(interval)-1)]
            for candidate in idx_:
                cos_value = np.dot(self.pcd_np[candidate] - pn, ori) / np.linalg.norm(self.pcd_np[candidate] - pn)
                dist = np.linalg.norm(self.pcd_np[candidate] - pn)
                if cos_value >= cos_thre[0] and cos_value < cos_thre[1]:
                    if dist < nearest_dist[0]:
                        nearest_dist[0] = copy.deepcopy(dist)
                elif cos_value >= cos_thre[1] and cos_value < cos_thre[2]:
                    if dist < nearest_dist[1]:
                        nearest_dist[1] = copy.deepcopy(dist)
                elif cos_value >= cos_thre[2] and cos_value < cos_thre[3]:
                    if dist < nearest_dist[2]:
                        nearest_dist[2] = copy.deepcopy(dist)
                elif cos_value >= cos_thre[3]:
                    if dist < nearest_dist[3]:
                        nearest_dist[3] = copy.deepcopy(dist)
                else:
                    # not in fov
                    continue
            obs_in_sight = 0.05 * sensing_range / nearest_dist[0] + 0.1 * sensing_range / nearest_dist[1] + 0.3 * sensing_range / nearest_dist[2] + 0.55 * sensing_range / nearest_dist[3]
            vis_list.append(copy.deepcopy(obs_in_sight))

        print("VO:", sum(vis_list) / len(vis_list))
        return sum(vis_list) / len(vis_list)

    def calMW(self, path_to_cal):
        theta_diff_list = []
        for path_id in range(len(path_to_cal)):
            if path_to_cal[path_id].shape[0] > 1:
                for piece_id in range(path_to_cal[path_id].shape[0]-1):
                    vec1 = (path_to_cal[path_id][piece_id, 3:] - path_to_cal[path_id][piece_id, :3]) / np.linalg.norm(path_to_cal[path_id][piece_id, 3:] - path_to_cal[path_id][piece_id, :3])
                    vec2 = (path_to_cal[path_id][piece_id + 1, 3:] - path_to_cal[path_id][piece_id + 1, :3]) / np.linalg.norm(path_to_cal[path_id][piece_id + 1, 3:] - path_to_cal[path_id][piece_id + 1, :3])
                    theta = np.arccos(np.dot(vec1, vec2))
                    # print(theta * 180 / np.pi)
                    theta_diff_list.append(np.exp(2 * theta / np.pi)-1)
            if path_id != len(path_to_cal)-1:
                vec1 = (path_to_cal[path_id][-1, 3:] - path_to_cal[path_id][-1, :3]) / np.linalg.norm(path_to_cal[path_id][-1, 3:] - path_to_cal[path_id][-1, :3])
                vec2 = (path_to_cal[path_id+1][0, 3:] - path_to_cal[path_id+1][0, :3]) / np.linalg.norm(path_to_cal[path_id+1][0, 3:] - path_to_cal[path_id+1][0, :3])
                theta = np.arccos(np.dot(vec1, vec2))
                # print(theta * 180 / np.pi)
                theta_diff_list.append(np.exp(6 * theta / np.pi)-1)

        # print("MW_list: ", theta_diff_list)
        print("AOL: ", sum(theta_diff_list) / self.total_len)
        return sum(theta_diff_list) / self.total_len
    
    def calAll(self):
        feasibility = self.calFeasibility()
        visibility = self.calVisibility(self.guide_path)
        MW = self.calMW(self.guide_path)
        print("difficulty:", feasibility+visibility+MW)
        return feasibility+visibility+MW


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("\033[31mPlease use: 'python3 cal_difficulty.py <scene_path>' to calculate scene difficuly.\033[0m")
        exit(-1)
    else:
        scene_path = os.path.join(os.environ["FLIGHTMARE_PATH"],sys.argv[1])
        # scene_path = os.path.join(os.environ["FLIGHTMARE_PATH"],"scene/maze-mid")
        print("scene path:", scene_path)
        calculator = difficultyCalculator(scene_path)
        calculator.calAll()