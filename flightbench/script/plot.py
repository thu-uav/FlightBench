import matplotlib
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns
import numpy as np

plt.close()
print(matplotlib.get_cachedir())
# plt.rcParams['axes.unicode_minus'] = False

# success-speed
speed = [1, 3, 5, 7, 9, 11]
success_agile_autonomy = [1, 1, 1, 0.8, 0.6, 0.3] # 9 11 not sure
success_lmt = [1]
speed_lmt = [11.68]
success_sb = [0.8]
speed_sb = [14.99]
speed_lpa = [9.74]
success_lpa = [1.0]
success_ego = [1, 0.9, 0.3, 0.0, 0.0, 0.0]
success_fast = [1, 0.8, 0.0, 0.0, 0.0, 0.0]
success_tgk = [1, 0.8, 0.2, 0.0, 0.0, 0.0]

plt.figure(figsize=(7.5, 5), dpi = 150)
plt.style.use("ggplot")
plt.plot(speed, success_ego, c = 'royalblue', marker = 'v', markersize = 12, linewidth = 4, label = 'EGO')
plt.plot(speed, success_fast, c = 'seagreen', marker = '^', markersize = 12,  linewidth = 4, label = 'Fast')
plt.plot(speed, success_tgk, c = 'slategray', marker = 'P', markersize = 12,  linewidth = 4, label = 'TGK')

plt.plot(speed, success_agile_autonomy, c = 'orangered', marker = 's', markersize = 12,  linewidth = 4, label = 'Agile')
plt.plot(speed_lpa, success_lpa, c = 'goldenrod', marker = 'D', markersize = 12, label = 'LPA')
plt.plot(speed_sb, success_sb, c = 'darkred', marker = 'p', markersize = 15, label = 'SBMT')
plt.plot(speed_lmt, success_lmt, c = 'indigo', marker = 'o', markersize = 15, label = 'LMT')

# plt.plot(speed, success_ego, marker = 'v', markersize = 6, linewidth = 1, label = 'Ego-Planner')
# plt.plot(speed, success_fast, marker = '^', markersize = 6,  linewidth = 1, label = 'Fast-Planner')
# plt.plot(speed, success_tgk, marker = 'P', markersize = 6,  linewidth = 1, label = 'TGK-Planner')

# plt.plot(speed, success_agile_autonomy, marker = 's', markersize = 6,  linewidth = 1, label = 'L21')
# plt.plot(speed_lmt, success_lmt, marker = 'o', markersize = 6, label = 'L22')
# plt.plot(speed_sb, success_sb, marker = 'o', markersize = 6, label = 'S22')
# plt.plot(speed_lpa, success_lpa, marker = 'o', markersize = 6, label = 'L23')
plt.legend(prop={'family': 'Times New Roman', 'size': 18})
# plt.grid()
plt.xticks(fontproperties = 'Times New Roman', fontsize=18)
plt.yticks(fontproperties = 'Times New Roman', fontsize=18)
plt.xlabel("Avg. Speed (m/s)", fontdict={'family': 'Times New Roman', 'size':18})
plt.ylabel("Success Rate", fontdict={'family': 'Times New Roman', 'size':18})
plt.savefig('success-speed.png', bbox_inches='tight')


# rationality
# AOL
AOL = [0.000764, 0.0016227, 0.005678, 0.001360, 0.01098, 0.61, 0.08, 0.94]
acc_over_l_lmt = [9.50E+00, 1.16E+01, 1.08E+01, 1.39E+01, 1.64E+01, 2.68E+01, 2.09E+01, 3.15E+01]
acc_over_l_sb = [3.21E+01, 3.54E+01, 2.87E+01, 3.55E+01, 3.23E+01, 6.16E+01, 4.89E+01, 6.81E+01]
kappa_lmt = [0.06082, 0.06637, 0.07495, 0.0793, 0.08835, 0.1337, 0.08679, 0.2591]
kappa_sb = [0.07291, 0.06863, 0.06462, 0.08692, 0.08223, 0.3132, 0.1767, 0.4280]

# plt.figure(figsize=(7.5, 5), dpi = 150)
# plt.style.use("ggplot")
# plt.xscale('log')
# plt.yscale('log')
# plt.scatter(AOL, acc_over_l_lmt, c = 'indigo', marker = 'o', s = 40, label = 'L22')
# plt.scatter(AOL, acc_over_l_sb, c = 'darkred', marker = 'p', s = 40, label = 'S22')
# plt.legend()
# plt.xlabel("AOL")
# plt.ylabel("Acc over Length (m/s^3)")
# plt.savefig('acc-aol.png')

# plt.figure(figsize=(7.5, 5), dpi = 150)
# plt.style.use("ggplot")
# plt.xscale('log')
# plt.yscale('log')
# plt.scatter(AOL, kappa_lmt, c = 'indigo', marker = 'o', s = 40, label = 'L22')
# plt.scatter(AOL, kappa_sb, c = 'darkred', marker = 'p', s = 40, label = 'S22')
# plt.legend()
# plt.xlabel("AOL")
# plt.ylabel("Average curvature")
# plt.savefig('kappa-aol.png')

# VO
VO = [0.3, 0.44, 0.6, 0.51, 1.01, 1.39, 0.55, 1.13]
success = []

# TO
# TO = [0.76, 0.92, 0.9, 1.42, 1.51, 1.54, 1.81, 1.58]
# success_lmt = [1, 1, 1, 1, 0.9, 1, 0.8, 0.8]
# success_sb = [0.9, 0.8, 0.7, 0.8, 0.8, 0.6, 0.6, 0.7]

# plt.figure(figsize=(7.5, 5), dpi = 150)
# plt.style.use("ggplot")
# #plt.xscale('log')
# # plt.yscale('log')
# plt.scatter(TO, success_lmt, c = 'indigo', marker = 'o', s = 40, label = 'L22')
# plt.scatter(TO, success_sb, c = 'darkred', marker = 'p', s = 40, label = 'S22')
# plt.legend()
# plt.xlabel("TO")
# plt.ylabel("success rate")
# plt.savefig('success-to.png')


# heatmap
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = 'NSimSun,Times New Roman'
x_label = ['Success\nRate', 'Avg.\nCurv.', 'Avg.\nSpeed', 'Avg.\nAcc.', 'Avg.\nJerk', 'Comp.\nTime']
y_label = ['TO', 'VO', 'AOL']

correl_lmt = np.array([
    [-0.723249346, 0.448187099, -0.598490978, 0.737802416, 0.122409204, 0.198544589],
    [-0.60610328, 0.632782077, -0.742295895, 0.822084068, 0.147744155, -0.11106857],
    [-0.769094051, 0.935921761, -0.930263619, 0.92959929, 0.486424189, -0.574696358]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_lmt[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
plt.style.use('ggplot')
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of LMT")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-lmt.png', bbox_inches='tight')

correl_sb = np.array([
    [-0.870973803, 0.560581103, -0.790859827, 0.572795952, 0.360245287, 0.035848552],
    [-0.703766939, 0.726265999, -0.751637305, 0.683474239, 0.511069454, 0.302989604],
    [-0.482030659, 0.984189313, -0.874234208, 0.969759736, 0.940415486, 0.71011475]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_sb[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of SBMT")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-sb.png', bbox_inches='tight')

correl_privilege = (correl_sb + correl_lmt) / 2
data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_privilege[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-privilege.png', bbox_inches='tight')

correl_agile_autonomy = np.array([
    [-0.443603618, 0.283513366, -0.73278867, 0.37469626, 0.380213108, 0.151585355],
    [-0.980359702, 0.656834754, -0.543330062, 0.694816615, 0.628394282, -0.057616766],
    [-0.794010216, 0.876240757, -0.313872739, 0.940104875, 0.844916687, 0.261676599],
]).T


data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_agile_autonomy[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of Agile")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-agile.png', bbox_inches='tight')

correl_lpa = np.array([
    [-0.714266345, 0.772795146, -0.625614215, 0.742737374, -0.455411632, -0.315501687],
    [-0.896474931, 0.424365507, -0.31213119, 0.60574599, -0.685541576, 0.116134526],
    [-0.890706305, 0.614671473, -0.382653173, 0.790361226, -0.37089382, 0.255999382]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_lpa[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of LPA")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-lpa.png', bbox_inches='tight')

correl_ego = np.array([
    [-0.505144951, 0.485169919, -0.508338939, 0.382538675, 0.463482177, 0.244649657],
    [-0.966706751, 0.657754935, -0.592137228, 0.534057393, 0.499630437, 0.438022484],
    [-0.760835762, 0.968944323, -0.352463277, 0.899519226, 0.657091314, 0.870934386]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_ego[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of EGO")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-ego.png', bbox_inches='tight')

correl_fast = np.array([
    [-0.336394074, 0.520537264, -0.575453729, 0.657763987, 0.586101896, 0.66489883],
    [-0.917387071, 0.718508202, -0.568246639, 0.720496342, 0.528909007, 0.528152915],
    [-0.747772826, 0.980248009, -0.669725466, 0.947910301, 0.879074557, 0.806075082]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_fast[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of Fast")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-fast.png', bbox_inches='tight')

correl_tgk = np.array([
    [-0.561035688, 0.44245611, -0.815062619, 0.618787919, 0.479301336, 0.500286883],
    [-0.907173429, 0.55427635, -0.676479571, 0.584006834, 0.54531097, -0.289507249],
    [-0.954454253, 0.86830165, -0.798220688, 0.895579526, 0.912414268, -0.112888013]
]).T

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_tgk[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of TGK")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-tgk.png', bbox_inches='tight')

correl_vision = (correl_agile_autonomy+correl_ego+correl_fast+correl_tgk+correl_lpa) / 5

data={}
for i in range(len(x_label)):
    data[x_label[i]] = np.abs(correl_vision[i])
pd_data=pd.DataFrame(data,index=y_label,columns=x_label)
print(pd_data)

plt.figure(figsize=(7.5, 4.0), dpi = 150)
ax = sns.heatmap(pd_data, annot=True, annot_kws = {'fontdict': {'size': 14}}, fmt=".2f", cmap="YlGnBu", square=True, linewidths=1, vmin = 0.0, vmax = 1.0)
# plt.title("correlation coefficient of ego-vision methods")
plt.xticks(fontsize=14)
plt.yticks(fontsize=14)
cax = plt.gcf().axes[-1]
cax.tick_params(labelsize=14)
plt.savefig('heat-vision.png', bbox_inches='tight')

# polar
# 使用ggplot的绘图风格
plt.style.use('ggplot')

# 构造数据
forest_low = [0.76,0.3,0.000764]
forest_mid = [0.92,0.44,0.0016227]
forest_high = [0.9, 0.6, 0.005678]
maze_low = [1.42, 0.51, 0.00136]
maze_mid = [1.51, 1.01, 0.01098]
maze_high = [1.54, 1.39, 0.61]
mw_low = [1.81, 0.55, 0.08]
mw_high = [1.58, 1.13, 0.94]

feature = ['TO','VO','AOL']

N = len(forest_low)
# 设置雷达图的角度，用于平分切开一个圆面
angles=np.linspace(0, 2*np.pi, N, endpoint=False)
# 为了使雷达图一圈封闭起来，需要下面的步骤
forest_low=np.concatenate((forest_low,[forest_low[0]]))
forest_mid=np.concatenate((forest_mid,[forest_mid[0]]))
forest_high=np.concatenate((forest_high,[forest_high[0]]))
maze_low=np.concatenate((maze_low,[maze_low[0]]))
maze_mid=np.concatenate((maze_mid,[maze_mid[0]]))
maze_high=np.concatenate((maze_high,[maze_high[0]]))
mw_low = np.concatenate((mw_low,[mw_low[0]]))
mw_high = np.concatenate((mw_high,[mw_high[0]]))
angles=np.concatenate((angles,[angles[0]]))

# 绘图
fig=plt.figure()
ax = fig.add_subplot(111, polar=True)
# 绘制折线图
forest = (forest_high+forest_mid+forest_low) / 3
ax.plot(angles, forest, 'o-', linewidth=2, label = 'forest')
# 填充颜色
ax.fill(angles, forest, alpha=0.25)
# 绘制第二条折线图
maze = (maze_high+maze_mid+maze_low) / 3
ax.plot(angles, maze, 's-', linewidth=2, label = 'maze')
ax.fill(angles, maze, alpha=0.25)

mw = (mw_low + mw_high) / 2
ax.plot(angles, mw, 's-', linewidth=2, label = 'multi-waypoint')
ax.fill(angles, mw, alpha=0.25)

# 添加每个特征的标签
ax.set_thetagrids(angles[:N] * 180/np.pi, feature)
# 设置雷达图的范围
ax.set_ylim(0,1.8)
# 添加标题
plt.title('Difficulty metrics of each scenarios')

# 添加网格线
ax.grid(True)
# 设置图例
plt.legend(loc = 'best')
# 显示图形
plt.savefig('lidar.png')