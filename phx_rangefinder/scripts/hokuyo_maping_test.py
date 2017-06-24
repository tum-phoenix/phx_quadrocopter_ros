from pyvis import LinePlot2D, Plotter3D
from pyqtgraph.Qt import QtCore, QtGui
import numpy as np
import scipy.ndimage as nd
import rospy
from sensor_msgs.msg import LaserScan

data = None
best_rot = 0
global_rot = 0
global_shift_x = 0
global_shift_y = 0
plot_live = LinePlot2D(color=(0, 0, 255))
plot_old = LinePlot2D(widget=plot_live.widget)
plot_dif = LinePlot2D(widget=plot_live.widget)
plot_dir = LinePlot2D(widget=plot_live.widget, color=(0, 255, 20))

plot_3d = Plotter3D(app=plot_live.app)


def validate(f):
    out = np.copy(f)
    if np.isnan(f[0]):
        out[0] = 0
    for i in range(1, f.shape[0]):
        if np.isnan(f[i]):
            out[i] = out[i-1]
        else:
            out[i] = f[i]
    return out


def radial_plot(data, start_angle=-2.0862, stop_angle=2.0862, angular_res=None):
    if angular_res is not None:
        start_angle = -1 * angular_res * data.shape[0] / 2
        stop_angle = -1 * start_angle
    angle = np.linspace(start_angle, stop_angle, data.shape[0])
    x = np.sin(angle) * data
    y = np.cos(angle) * data
    return x, y


def line_plot(data, start_angle=-2.0862, stop_angle=2.0862):
    angle = np.linspace(start_angle, stop_angle, data.shape[0])
    x = np.sin(angle) * data
    y = np.cos(angle) * data
    return x, y


def callback_LaserScan(new_LaserScan=LaserScan()):
    global data
    in_data = np.array(new_LaserScan.ranges)[::-1]
    in_data[in_data > 6] = np.NaN
    val_data = in_data      # validate(in_data)
    if not isinstance(data, np.ndarray):
        data = np.array(val_data.tolist() * 5).reshape((len(new_LaserScan.ranges), 5))
    else:
        data[:, 1:] = data[:, :-1]
        data[:, 0] = val_data


rospy.init_node('hokuyo_test')
ros_subscribe_LaserScan = rospy.Subscriber('/scan', LaserScan, callback_LaserScan, queue_size=1)


def main_loop_diff():
    global data

    speed = np.diff(data, n=1, axis=1)
    c = line_plot(data[:, 0])
    plot_live.setData(c[0], c[1])
    c = line_plot(data[:, 1])
    plot_old.setData(c[0], c[1])
    #c = line_plot(data[:, 0]-data[:, 1])
    c = line_plot(speed[:, 1])
    plot_dif.setData(c[0], c[1])


def main_loop_fft():
    global data
    a0 = np.fft.fft(data[:, 0])
    a1 = np.fft.fft(data[:, 1])
    auto_corr = np.fft.ifft(a0 * a1.conj())
    c = line_plot(data[:, 0])
    plot_live.setData(c[0], c[1])
    c = line_plot(data[:, 1])
    plot_old.setData(c[0], c[1])
    c = line_plot(np.abs(auto_corr)/2000)
    plot_dif.setData(c[0], c[1])


def main_loop_brute_force_shift():
    global data
    current = data[:, 0]
    old = np.copy(data[:, 1])
    analysis = []
    r = 20
    for i in range(-r, r):
        analysis.append(np.sum(np.abs(current[r:-r]-old[r+i:-r+i])))
    analysis = np.array(analysis)
    best_shift = np.argmin(analysis) - r
    print best_shift

    c = radial_plot(data[:, 0])
    plot_live.setData(c[0], c[1])
    c = radial_plot(data[r+best_shift:-r+best_shift, 1])
    plot_old.setData(c[0], c[1])
    c = radial_plot(data[r:-r, 0]-data[r+best_shift:-r+best_shift, 1])
    plot_dif.setData(c[0], c[1])


def main_loop_individual_shift():
    global data, best_rot, global_rot, global_shift_x, global_shift_y
    d0 = data[50:-50, 0]
    d1 = data[50:-50, 1]

    r = 40
    analysis = []
    for rotation in range(-r, r):
        dist_err = np.abs(d0[r:-r]-d1[r+rotation:-r+rotation])
        dist_err[np.isnan(dist_err)] = 0
        dist_err[dist_err > 2] = 0
        analysis.append(np.sum(dist_err))
    analysis = np.array(analysis)
    best_rot = np.argmin(analysis) - r

    # second try to find x y shift via brute force
    # binned_d0 = (d0[:-1:2] + d0[1::2]) / 2.
    # binned_d1 = (d1[:-1:2] + d1[1::2]) / 2.
    # wr_d0 = radial_plot(binned_d0[r:-r], angular_res=0.00612*2)
    # wr_d1 = radial_plot(binned_d1[r+best_rot//2:-r+best_rot//2], angular_res=0.00612*2)
    # r = 5           # 20
    # scale = 0.02    # 0.01
    # analysis = []
    # config = []
    # offset = np.ones_like(wr_d0)
    # for dx in range(-r//2, r//2):
    #     offset[0, :] = dx * scale
    #     for dy in range(-r, r):
    #         offset[1, :] = dy * scale
    #         dist_err = np.abs(wr_d0 + offset - wr_d1)
    #         dist_err[np.isnan(dist_err)] = 0
    #         dist_err[dist_err > 0.5] = 0
    #         analysis.append(np.sum(dist_err))
    #         config.append([dx * scale, dy * scale])
    # analysis = np.array(analysis)
    # best_shift = np.argmin(analysis)
    # best_shift = config[best_shift]
    # offset = np.ones_like(wr_d0)
    # offset[0, :] = best_shift[0] * scale
    # offset[1, :] = best_shift[1] * scale
    #
    # swr_d0 = wr_d0 + offset

    print best_rot, '\t', global_rot
    # print best_rot, '\t', best_shift[0], '\t', best_shift[1], global_rot

    # c = swr_d0
    # plot_live.setData(c[0], c[1])
    # c = wr_d1
    # plot_old.setData(c[0], c[1])

    global_rot += best_rot * 0.00612
    angles = np.linspace(-1 * data.shape[0] * 0.00612 * 0.5 + global_rot,
                         data.shape[0] * 0.00612 * 0.5 + global_rot,
                         data.shape[0])
    x = np.sin(angles[::2]) * data[::2, 0]
    y = np.cos(angles[::2]) * data[::2, 0]
    z = np.ones_like(y)
    plot_3d.add_point(x.tolist(), y.tolist(), z.tolist())
    plot_3d.update()

    # first try to get move direction from direct difference
    # r_d0 = d0[r:-r]
    # r_d1 = d1[r+best_rot:-r+best_rot]
    #
    # dist = r_d1 - r_d0
    # dist[np.isnan(dist)] = 0
    # dist = nd.gaussian_filter(dist, 20)
    # best_dir = np.argmax(dist[r:-r]) + r
    # best_speed = dist[best_dir]
    #
    # print best_rot, '\t', best_dir, '\t', best_speed
    #
    # c = radial_plot(r_d0, angular_res=0.00612)
    # plot_live.setData(c[0], c[1])
    # c = radial_plot(r_d1, angular_res=0.00612)
    # plot_old.setData(c[0], c[1])
    # c = radial_plot(dist, angular_res=0.00612)
    # plot_dif.setData(c[0], c[1])
    # plot_dir.setData(x=np.array((np.sin(0.00612*best_dir)*best_speed, 0, np.sin(0.00612*best_rot)*2)),
    #                  y=np.array((np.cos(0.00612*best_dir)*best_speed, 0, np.cos(0.00612*best_rot)*2)))


timer = QtCore.QTimer()
timer.timeout.connect(main_loop_individual_shift)
timer.start(100)


plot_live.run()

ros_subscribe_LaserScan.unregister()

timer.stop()
