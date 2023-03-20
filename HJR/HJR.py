import os
import warnings
import numpy as np
import laspy
import open3d as o3d
from torch.utils.data import Dataset
import matplotlib.pyplot as plt
from mayavi import mlab

r = 1
edg_dis = 0.004
end_dis = 0.028
bais = 0.08


def viz_mayavi(points, color, joint):
    x = points[:, 0]  # x position of point
    y = points[:, 1]  # y position of point
    z = points[:, 2]  # z position of point
    joint = np.array(joint)
    fig = mlab.figure(bgcolor=(0, 0, 0), size=(640, 360))
    human = mlab.points3d(x, y, z, scale_factor=0.01)

    human.glyph.scale_mode = 'data_scaling_off'
    human.mlab_source.dataset.point_data.scalars = color


    # jcolor = joint[:, 3:6]
    # pose = mlab.points3d(joint[:, 0], joint[:, 1], joint[:, 2], scale_factor=0.04)
    # pose.glyph.scale_mode = 'scale_by_vector'
    # pose.mlab_source.dataset.point_data.scalars = jcolor

    mlab.show()


def creat_human_joint(path):
    head = []
    L_boom = []
    L_forearm = []
    R_boom = []
    R_forearm = []
    Trunk = []
    L_thigh = []
    L_shank = []
    R_thigh = []
    R_shank = []
    trunk_edg_index = []
    boom_trunk_edg_index = []
    a = []
    b = []
    human_joint = []
    data = []
    index = 0


    data = np.loadtxt(path).astype(np.float32)
    seg = data[:, -1].astype(np.int32)  # 拿到小类别的标签
    color = data[:, 3:6]
    point_set = data[:, 0:3]
    with open(path, 'r') as f:
        for line in f:
            ls = line.strip().split()
            temp = data[index]
            temp[6] = index
            if ls[6] == '1':
                head.append(temp)
            elif ls[6] == '3':
                L_boom.append(temp)
            elif ls[6] == '2':
                L_forearm.append(temp)
            elif ls[6] == '5':
                R_boom.append(temp)
            elif ls[6] == '4':
                R_forearm.append(temp)
            elif ls[6] == '6':
                Trunk.append(temp)
            elif ls[6] == '7':
                L_thigh.append(temp)
            elif ls[6] == '8':
                L_shank.append(temp)
            elif ls[6] == '9':
                R_thigh.append(temp)
            elif ls[6] == '10':
                R_shank.append(temp)
            index = index + 1

    head_trunk_index, trunk_head_index, trunk = head_trunk_edg(head, Trunk, 1)

    for index in range(head_trunk_index.__len__()):
        color[head_trunk_index[index]] = [0, 0.3, 0.3]
    for index in range(trunk_head_index.__len__()):
        color[trunk_head_index[index]] = [0, 0.5, 0]

    R_boom_trunk_index, trunk_R_boom_index, c = head_trunk_edg(Trunk, R_boom, 3)

    for index in range(R_boom_trunk_index.__len__()):
        color[R_boom_trunk_index[index]] = [0, 0.3, 0.3]
    for index in range(trunk_R_boom_index.__len__()):
        color[trunk_R_boom_index[index]] = [0, 0.5, 0]

    L_boom_trunk_index, trunk_L_boom_index, d = head_trunk_edg(Trunk, L_boom, 3)

    for index in range(L_boom_trunk_index.__len__()):
        color[L_boom_trunk_index[index]] = [0, 0.3, 0.3]
    for index in range(trunk_L_boom_index.__len__()):
        color[trunk_L_boom_index[index]] = [0, 0.5, 0]

    L_boom_L_forearm_index, L_forearm_L_boom_index, e = head_trunk_edg(L_boom, L_forearm, 1)

    for index in range(L_boom_L_forearm_index.__len__()):
        color[L_boom_L_forearm_index[index]] = [0, 0.3, 0.3]
    for index in range(L_forearm_L_boom_index.__len__()):
        color[L_forearm_L_boom_index[index]] = [0, 0.5, 0]

    R_boom_R_forear_index, R_forearm_R_boom_index, f = head_trunk_edg(R_boom, R_forearm, 1)

    for index in range(R_boom_R_forear_index.__len__()):
        color[R_boom_R_forear_index[index]] = [0, 0.3, 0.3]
    for index in range(R_forearm_R_boom_index.__len__()):
        color[R_forearm_R_boom_index[index]] = [0, 0.5, 0]

    Trunk_L_thigh_index, L_thigh_Trunk_index, g = head_trunk_edg(Trunk, L_thigh, 1)

    for index in range(Trunk_L_thigh_index.__len__()):
        color[Trunk_L_thigh_index[index]] = [0, 0.3, 0.3]
    for index in range(L_thigh_Trunk_index.__len__()):
        color[L_thigh_Trunk_index[index]] = [0, 0.5, 0]

    R_thigh_Trunk_index, Trunk_R_thigh_index, h = head_trunk_edg(Trunk, R_thigh, 1)

    for index in range(R_thigh_Trunk_index.__len__()):
        color[R_thigh_Trunk_index[index]] = [0, 0.3, 0.3]
    for index in range(Trunk_R_thigh_index.__len__()):
        color[Trunk_R_thigh_index[index]] = [0, 0.5, 0]

    L_thigh_L_shank_index, L_shank_L_thigh_index, i = head_trunk_edg(L_thigh, L_shank, 1)

    for index in range(L_thigh_L_shank_index.__len__()):
        color[L_thigh_L_shank_index[index]] = [0, 0.3, 0.3]
    for index in range(L_shank_L_thigh_index.__len__()):
        color[L_shank_L_thigh_index[index]] = [0, 0.5, 0]

    R_shank_R_thigh_index, R_thigh_R_shank_index, j = head_trunk_edg(R_thigh, R_shank, 1)
    for index in range(R_shank_R_thigh_index.__len__()):
        color[R_shank_R_thigh_index[index]] = [0, 0.3, 0.3]
    for index in range(R_thigh_R_shank_index.__len__()):
        color[R_thigh_R_shank_index[index]] = [0, 0.5, 0]

    human_joint.append(head_joint(head))
    human_joint.append(trunk)
    human_joint.append(c)
    human_joint.append(d)
    human_joint.append(e)
    human_joint.append(f)
    human_joint.append(g)
    human_joint.append(h)
    human_joint.append(i)
    human_joint.append(j)

    head_edg_index, k = wrist_joint(R_forearm, f)
    head_edg_index, l = wrist_joint(L_forearm, e)
    human_joint.append(k)
    human_joint.append(l)

    leg_len = (h[0] - j[0])**2 + (h[1] - j[1])**2 + (h[2] - j[2] - bais)**2

    clox, m = ankle_joint(L_shank, i, leg_len)
    clo, n = ankle_joint(R_shank, j, leg_len)
    human_joint.append(m)
    human_joint.append(n)
    #
    # for i in range(clox.__len__()):
    #     color[clox[i]] = [0, 0, 255]
    # for i in range(clo.__len__()):
    #     color[clo[i]] = [0, 0, 255]


    viz_mayavi(point_set, color, human_joint)
    np.savetxt(r'D:\Workplace\human_part_seg\data\human\person\head\joint.txt', head, fmt='%f')
    np.savetxt(r'D:\Workplace\human_part_seg\data\human\person\head\joint.txt', human_joint, fmt='%f')
    #head_joint(head)
    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(L_boom)
    # o3d.visualization.draw_geometries([pcd])


def conform(x1, x2, y1, y2, z1, z2, dis):
    k = (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2) + (z1-z2)*(z1-z2)
    if k < dis :
        conf = 0
    else :
        conf = 1
    return conf


def center(point):
    headz_list = sorted(point, key=lambda x: x[2], reverse=True)
    heady_list = sorted(point, key=lambda x: x[1], reverse=True)
    headx_list = sorted(point, key=lambda x: x[0], reverse=True)
    x_center = ((headx_list[0][0] - headx_list[headx_list.__len__() - 1][0]) / 2) + headx_list[headx_list.__len__() - 1][0]
    y_center = ((heady_list[0][1] - heady_list[heady_list.__len__() - 1][1]) / 2) + heady_list[heady_list.__len__() - 1][1]
    z_center = ((headz_list[0][2] - headz_list[headz_list.__len__() - 1][2]) / 2) + headz_list[headz_list.__len__() - 1][2]

    return x_center, y_center, z_center


def head_trunk_edg(head, trunk, index):
    head_edg_index = []
    trunk_head_edg_index = []
    head_edg_point = []
    trunk_head_edg_point = []
    shoulder_max = 0
    log = np.zeros(2048, dtype=int)
    for i in range(head.__len__()):
        for j in range(trunk.__len__()):
            if conform(head[i][0], trunk[j][0], head[i][1], trunk[j][1], head[i][2], trunk[j][2], edg_dis) == 0:
                if log[(trunk[j][6]).astype('int32')] == 0:
                    log[(trunk[j][6]).astype('int32')] = 1
                    trunk_head_edg_point.append(trunk[j])
                    if trunk[j][2] > shoulder_max:
                        shoulder_max = trunk[j][2]

                if log[(head[i][6]).astype('int32')] == 0:
                    log[(head[i][6]).astype('int32')] = 2
                    head_edg_point.append(head[i])
    for i in range(log.__len__()):
        if log[i] == 1:
            trunk_head_edg_index.append(i)
        elif log[i] == 2:
            head_edg_index.append(i)

    x_center, y_center, z_center = center(trunk_head_edg_point)
    if index == 3:
        z_center = shoulder_max - 0.05
    joint = [x_center, y_center, z_center, 0, 120, 0, r]
    return head_edg_index, trunk_head_edg_index, joint


def wrist_joint(point_set, joint):
    end_joint_index = []
    end_joint_point = []
    log = np.zeros(2048, dtype=int)
    for j in range(point_set.__len__()):
        if conform(point_set[j][0], joint[0], point_set[j][1], joint[1], point_set[j][2], joint[2], end_dis) == 1:
            if log[(point_set[j][6]).astype('int32')] == 0:
                log[(point_set[j][6]).astype('int32')] = 1
                end_joint_point.append(point_set[j])
    for i in range(log.__len__()):
        if log[i] == 1:
            end_joint_index.append(i)
    x_center, y_center, z_center = center(end_joint_point)
    joint = [x_center, y_center, z_center, 0, 255, 0, r]
    return end_joint_index, joint


def head_joint(point):
    joint = []
    x_center, y_center, z_center = center(point)
    joint = [x_center, y_center, z_center, 0, 0, 255, r]
    return joint


def ankle_joint(point_set, joint, leg_len):
    ankle_joint_index = []
    ankle_joint_point = []
    log = np.zeros(2048, dtype=int)
    for j in range(point_set.__len__()):
        if leg_len < ((point_set[j][0]-joint[0]) ** 2 + (point_set[j][1]-joint[1]) ** 2 + (point_set[j][2]-joint[2]) ** 2) < leg_len + 1:
            if log[(point_set[j][6]).astype('int32')] == 0:
                log[(point_set[j][6]).astype('int32')] = 1
                ankle_joint_point.append(point_set[j])
    for i in range(log.__len__()):
        if log[i] == 1:
            ankle_joint_index.append(i)
    x_center, y_center, z_center = center(ankle_joint_point)
    joint = [x_center, y_center, z_center, 0, 255, 0, r]
    return ankle_joint_index, joint




if __name__ == '__main__':

    path = r'D:\Workplace\human_part_seg\data\human\person\Person_4_0005.txt'
    creat_human_joint(path)

