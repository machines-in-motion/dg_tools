import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt

### Parameters
ti = 0.0
tf = 3.6
T = 0.001
nb_pt = int((tf-ti)/T)

### Time
time_0 = 0.0
time_1 = tf/3.0
time_2 = 2*tf/3.0
time_3 = tf
time = np.linspace(0.0, time_3, num=nb_pt, endpoint=True)
phase0 = np.linspace(time_0, time_1, num=int((time_1-time_0)/T), endpoint=True)
phase1 = np.linspace(time_1, time_2, num=int((time_2-time_1)/T), endpoint=True)
phase2 = np.linspace(time_2, time_3, num=int((time_3-time_2)/T), endpoint=True)

### CoM
com0_x = 0.2
com0_y = 0.00044412
com0_z = 0.19240999
com_x = np.ones(nb_pt) * com0_x
com_y = np.ones(nb_pt) * com0_y
com_z = np.ones(nb_pt) * com0_z

### HL
# ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
hl0_x = 0.01
hl0_y = 0.14
hl0_z = -0.00294615
hl_x = np.ones(nb_pt) * hl0_x
hl_y = np.ones(nb_pt) * hl0_y
hl_z = np.ones(nb_pt) * hl0_z
hl_dx = np.zeros(nb_pt)
hl_dy = np.zeros(nb_pt)
hl_dz = np.zeros(nb_pt)

### HR
# ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
hr0_x = 0.01
hr0_y = -0.14
hr0_z = 0.0
hr_x = np.ones(nb_pt) * hr0_x
hr_y = np.ones(nb_pt) * hr0_y
hr_z = np.ones(nb_pt) * hr0_z
hr_dx = np.zeros(nb_pt)
hr_dy = np.zeros(nb_pt)
hr_dz = np.zeros(nb_pt)

### FL
# ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
fl0_x = 0.39
fl0_y = 0.14
fl0_z = 0.0

fl1_x = fl0_x + 0.1
fl1_z = fl0_z + 0.2

t_s = np.array([time_0, time_0+T, time_0+2*T, (time_1-time_0)/3.0, 2.0*(time_1-time_0)/3.0, time_1-2*T, time_1-T, time_1])
x = np.array([fl0_x, fl0_x, fl0_x, fl0_x, fl0_x, fl1_x, fl1_x, fl1_x])
z = np.array([fl0_z, fl0_z, fl0_z, fl1_z/3.0, fl1_z + 0.1, fl1_z, fl1_z, fl1_z])
x_s = interpolate.splrep(t_s, x, s=0)
z_s = interpolate.splrep(t_s, z, s=0)

fl_x = np.append(interpolate.splev(phase0, x_s, der=0), np.ones(phase1.shape[0]+phase2.shape[0]) * fl1_x)
fl_y = np.ones(nb_pt) * fl0_y
fl_z = np.append(interpolate.splev(phase0, z_s, der=0), np.ones(phase1.shape[0]+phase2.shape[0]) * fl1_z)

fl_dx = np.append(interpolate.splev(phase0, x_s, der=1), np.zeros(phase1.shape[0]+phase2.shape[0])) 
fl_dy = np.zeros(nb_pt)
fl_dz = np.append(interpolate.splev(phase0, z_s, der=1), np.zeros(phase1.shape[0]+phase2.shape[0])) 

### FR
# ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))
fr0_x = 0.39
fr0_y = -0.14
fr0_z = 0.0

fr1_x = fr0_x + 0.1
fr1_z = fr0_z + 0.2

t_s = np.array([time_1, time_1+T, time_1+2*T, time_1 + (time_2-time_1)/3.0, time_1 + 2.0*(time_2-time_1)/3.0, time_2-2*T, time_2-T, time_2])
print (t_s)
x = np.array([fr0_x, fr0_x, fr0_x, fr0_x, fr0_x, fr1_x, fr1_x, fr1_x])
z = np.array([fr0_z, fr0_z, fr0_z, fr1_z/3.0, fr1_z + 0.1, fr1_z, fr1_z, fr1_z])
x_s = interpolate.splrep(t_s, x, s=0)
z_s = interpolate.splrep(t_s, z, s=0)

fr_x = np.append(np.ones(phase0.shape[0]) * fr0_x, interpolate.splev(phase1, x_s, der=0))
fr_x = np.append(fr_x, np.ones(phase2.shape[0]) * fr1_x)
fr_y = np.ones(nb_pt) * fr0_y
fr_z = np.append(np.ones(phase0.shape[0]) * fr0_z, interpolate.splev(phase1, z_s, der=0))
fr_z = np.append(fr_z, np.ones(phase2.shape[0]) * fr1_z)

fr_dx = np.append(np.zeros(phase1.shape[0]),  interpolate.splev(phase1, x_s, der=1))
fr_dx = np.append(fr_dx, np.zeros(phase2.shape[0]))
fr_dy = np.zeros(nb_pt)
fr_dz = np.append(np.zeros(phase1.shape[0]),  interpolate.splev(phase1, z_s, der=1))
fr_dz = np.append(fr_dz, np.zeros(phase2.shape[0]))

### creation of the data files
quadruped_com = np.vstack((time, com_x, com_y, com_z))


print ("quadruped_com", quadruped_com)
# quadruped_positions_eff = np.append([hl_x,
#                                      hl_y,
#                                      hl_z,
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      hr_x,
#                                      hr_y,
#                                      hr_z,
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      fl_x,
#                                      fl_y,
#                                      fl_z,
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      fr_x,
#                                      fr_y,
#                                      fr_z,
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt),
#                                      np.zeros(nb_pt)], axis=0)
# #
# quadruped_velocities_eff = np.append([hl_dx,
#                                       hl_dy,
#                                       hl_dz,
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       hr_dx,
#                                       hr_dy,
#                                       hr_dz,
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       fl_dx,
#                                       fl_dy,
#                                       fl_dz,
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       fr_dx,
#                                       fr_dy,
#                                       fr_dz,
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt),
#                                       np.zeros(nb_pt)], axis=0)

### plots
# plt.figure("com")
# plt.plot(time, com_x, ':',
#          time, com_y, '-',
#          time, com_z, '--')
# plt.legend(['com_x', 'com_y', 'com_z'], loc='best')
#
# plt.figure("fl")
# plt.plot(time, fl_x, ':',
#          time, fl_y, '-',
#          time, fl_z, '--')
# plt.legend(['fl_x', 'fl_y', 'fl_z'], loc='best')
#
# plt.figure("fx")
# plt.plot(time, fl_x, ':',
#          time, fl_y, '-',
#          time, fl_z, '--')
# plt.legend(['fl_x', 'fl_y', 'fl_z'], loc='best')
#
plt.figure("fl pos")
plt.plot(time, fl_x, ':',
         time, fl_y, '-',
         time, fl_z, '--',
         time, fr_x, ':',
         time, fr_y, '-',
         time, fr_z, '--')
plt.legend(['fl_x', 'fl_y', 'fl_z', 'fr_x', 'fr_y', 'fr_z'], loc='best')
#
plt.figure("fl vel")
plt.plot(time, fl_dx, ':',
         time, fl_dy, '-',
         time, fl_dz, '--',
         time, fr_dx, ':',
         time, fr_dy, '-',
         time, fr_dz, '--')
plt.legend(['fl_dx', 'fl_dy', 'fl_dz', 'fr_dx', 'fr_dy', 'fr_dz'], loc='best')

plt.show()

