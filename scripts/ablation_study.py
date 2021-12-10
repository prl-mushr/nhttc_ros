import matplotlib.pyplot as plt
import numpy as np

full_sys = np.load('/home/stark/catkin_mushr/src/nhttc_ros/bags/full_sys_ded_solve/output_raw.npy', allow_pickle = True)
locl_sys = np.load('/home/stark/catkin_mushr/src/nhttc_ros/bags/nhttc_standalone/output_raw.npy', allow_pickle = True)
glob_sys = np.load('/home/stark/catkin_mushr/src/nhttc_ros/bags/globl_pure_pursuit/output_raw.npy', allow_pickle = True)
full_sys_ded = np.load('/home/stark/catkin_mushr/src/nhttc_ros/bags/full_sys_wo_ded/output_raw.npy', allow_pickle = True)
idot_sys = np.load('/home/stark/catkin_mushr/src/nhttc_ros/bags/output_bags/output_raw.npy', allow_pickle = True)

print( np.min(idot_sys[4,:]) )

def filter(x):
    exclude = [ 4,  5,  6,  9, 14, 16, 19, 24, 26, 29, 34, 35, 36, 39, 44, 45, 46,
       49, 54, 56, 59, 64, 66, 69, 74, 76, 79, 84, 85, 86, 89, 94, 96, 99]
    N = 100 - len(exclude)
    y = np.zeros((5,N))
    for i in range(5):
        y[i,:] = np.delete(x[i,:],exclude)
    return y
# full_sys = filter(full_sys)
# locl_sys = filter(locl_sys)
# glob_sys = filter(glob_sys)
# full_sys_ded = filter(full_sys_ded)

def get_m_std(x, bernoulli):
    y = x[np.isfinite(x)]  # because sometimes we get inf
    m, s = np.mean(y), np.std(y)
    if(bernoulli):
        s = 0  # np.sqrt(m*(1-m))
    return m, s

def update_collision(x):
    x[1,:] = np.array(x[4,:] < 0.5, dtype=float)
    x[0,:] = x[0,:]*np.array(x[1,:] < 1, dtype =float)
    return x

mean = np.zeros((5,5))  # order: success, collisions, final error, relative travel time, minimum separation (center to center)
stds = np.zeros((5,5))
locl_sys = update_collision(locl_sys)
glob_sys = update_collision(glob_sys)
full_sys = update_collision(full_sys)
full_sys_ded = update_collision(full_sys_ded)
idot_sys = update_collision(idot_sys)

for i in range(5):
    mean[i,0], stds[i,0] = get_m_std(full_sys[i, :], i<2)
    mean[i,1], stds[i,1] = get_m_std(full_sys_ded[i,:], i<2)
    mean[i,2], stds[i,2] = get_m_std(locl_sys[i, :], i<2)
    mean[i,3], stds[i,3] = get_m_std(glob_sys[i, :], i<2)
    mean[i,4], stds[i,4] = get_m_std(idot_sys[i, :], i<2)

metrics = ['success percentage(x100)', 'collision percentage(x100)', 'final_error(meters)', 'relative travel time (ratio)', 'minimum separation(meters)']
x_pos = np.arange(len(metrics))

fig, ax = plt.subplots()
labels = ['full_sys', 'full_sys_without deadlock','local', 'global', 'blind local']
colors = ['red', 'purple','blue', 'green', 'grey']
bar_width = 0.15

for i in range(5):
    ax.bar(x_pos + bar_width*i, mean[:,i], bar_width, yerr=stds[:,i], align='center', color = colors[i], capsize=10, label = labels[i])
ax.set_xticks(x_pos)
ax.set_ylabel('performance')
ax.set_xticklabels(metrics)
ax.set_title('performance comparison between systems')
ax.yaxis.grid(True)
ax.legend()

plt.show()
