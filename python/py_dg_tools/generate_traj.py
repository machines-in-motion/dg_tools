"""
@package py_dg_tools
@author Maximilien Naveau
@license License BSD-3-Clause
@copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
@date 2019-08-01
@brief Generate trajectories in python using scypy
"""

import numpy as np
from scipy import interpolate
import matplotlib.pyplot as plt


def traj_generator(time_way_points, way_points, time):

    # print ("time_way_points = ", time_way_points)
    # print ("way_points[0] = ", way_points[0])
    # print ("way_points[1] = ", way_points[1])
    # print ("way_points[2] = ", way_points[2])
    k0=3
    if len(way_points[0])<3: k0=1
    
    k1=3
    if len(way_points[1])<3: k1=1

    k2=3
    if len(way_points[2])<3: k2=1

    x_s = interpolate.splrep(time_way_points, np.array(way_points[0]), s=0, k=k0)
    y_s = interpolate.splrep(time_way_points, np.array(way_points[1]), s=0, k=k1)
    z_s = interpolate.splrep(time_way_points, np.array(way_points[2]), s=0, k=k2)
    
    pos = np.array( [interpolate.splev(time, x_s, der=0),
                     interpolate.splev(time, y_s, der=0),
                     interpolate.splev(time, z_s, der=0)] )

    vel = np.array( [interpolate.splev(time, x_s, der=1),
                     interpolate.splev(time, y_s, der=1),
                     interpolate.splev(time, z_s, der=1)] )

    return pos, vel

if __name__ == "__main__":
    ### Parameters
    time_phase = 1.2
    T = 0.001
    
    ### Time
    
    time_0 = 0.0
    time_1 = tf/3.0
    time_2 = 2*tf/3.0
    time_3 = tf
    time = np.linspace(0.0, time_3, num=nb_pt, endpoint=True)
    
    phase0 = np.linspace(time_0, time_1, num=int((time_1-time_0)/T), endpoint=True)
    phase1 = np.linspace(time_1, time_2, num=int((time_2-time_1)/T), endpoint=True)
    phase2 = np.linspace(time_2, time_3, num=int((time_3-time_2)/T), endpoint=True)
    nb_pt_phase0 = phase0.shape[0]
    nb_pt_phase1 = phase1.shape[0]
    nb_pt_phase2 = phase2.shape[0]

    ### Phase 0 ################################################################
    
    ## CoM
    # oMcom ('com: ', matrix([[0.2       , 0.00044412, 0.19240999]]))
    # bMcom 
    com0_x = 0.2
    com0_y = 0.00044412
    com0_z = 0.19240999
    com, dcom = traj_generator([time_0, time_1],
      [[0.2, 0.2], [0.00044412, 0.00044412], [0.19240999, 0.19240999]], phase0)

    ## HL
    # oMhl ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
    # bMhl ('hl: ', matrix([[-0.19      ,  0.14205   , -0.22294615]]))
    hl, dhl = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.22294615, -0.1]], phase0)

    ## HR
    # ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
    # bMhr ('hr: ', matrix([[-0.19      , -0.14205   , -0.22294615]]))
    hr, dhr = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.22294615, -0.1]], phase0)

    ### FL
    # ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
    # bMfl ('fl: ', matrix([[ 0.19      ,  0.14205   , -0.22294615]]))
    x_init = 0.0
    z_init = -0.22294615
    x_final = 0.1
    z_final = 0.2
    t_s = [time_0, time_0+T, time_0+2*T, (time_1-time_0)/3.0, 2.0*(time_1-time_0)/3.0, time_1-2*T, time_1-T, time_1]
    x = [x_init, x_init, x_init, x_init, x_init, x_final, x_final, x_final]
    y = [0.0] * len(t_s)
    z = [z_init, z_init, z_init, z_final/3.0, z_final + 0.1, z_final, z_final, z_final]
    fl, dfl = traj_generator(t_s, [x, y, z], phase0)
    
    ### FR
    # ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))
    # bMfr ('fr: ', matrix([[ 0.19      , -0.14205   , -0.22294615]]))
    fr, dfr = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.22294615, -0.22294615]], phase0)


    ### Phase 1 ################################################################
    
    ## CoM
    # oMcom ('com: ', matrix([[0.2       , 0.00044412, 0.19240999]]))
    # bMcom 
    com0_x = 0.2
    com0_y = 0.00044412
    com0_z = 0.19240999
    com_phase, dcom_phase = traj_generator([time_0, time_1],
      [[0.2, 0.2], [0.00044412, 0.00044412], [0.19240999, 0.19240999]], phase0)

    ## HL
    # oMhl ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
    # bMhl ('hl: ', matrix([[-0.19      ,  0.14205   , -0.22294615]]))
    hl_phase, dhl_phase = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.1, -0.1]], phase0)

    ## HR
    # ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
    # bMhr ('hr: ', matrix([[-0.19      , -0.14205   , -0.22294615]]))
    hr_phase, dhr_phase = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.1, -0.1]], phase0)

    ### FL
    # ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
    # bMfl ('fl: ', matrix([[ 0.19      ,  0.14205   , -0.22294615]]))
    fl_phase, dfl_phase = traj_generator([time_0, time_1],
      [[0.1, 0.1], [0.0, 0.0], [0.2, 0.2]], phase0)
    
    ### FR
    # ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))
    # bMfr ('fr: ', matrix([[ 0.19      , -0.14205   , -0.22294615]]))
    x_init = 0.0
    z_init = -0.22294615
    x_final = 0.1
    z_final = 0.2
    t_s = [time_0, time_0+T, time_0+2*T, (time_1-time_0)/3.0, 2.0*(time_1-time_0)/3.0, time_1-2*T, time_1-T, time_1]
    x = [x_init, x_init, x_init, x_init, x_init, x_final, x_final, x_final]
    y = [0.0] * len(t_s)
    z = [z_init, z_init, z_init, z_final/3.0, z_final + 0.1, z_final, z_final, z_final]
    fr_phase, dfr_phase = traj_generator(t_s, [x, y, z], phase0)

    # stack the trajectories
    com = np.concatenate((com, com_phase), axis=1)
    hl = np.concatenate((hl, hl_phase), axis=1)
    dhl = np.concatenate((dhl, dhl_phase), axis=1)
    hr = np.concatenate((hr, hr_phase), axis=1)
    dhr = np.concatenate((dhr, dhr_phase), axis=1)
    fl = np.concatenate((fl, fl_phase), axis=1)
    dfl = np.concatenate((dfl, dfl_phase), axis=1)
    fr = np.concatenate((fr, fr_phase), axis=1)
    dfr = np.concatenate((dfr, dfr_phase), axis=1)

    ### Phase 2 ################################################################

    ## CoM
    # oMcom ('com: ', matrix([[0.2       , 0.00044412, 0.19240999]]))
    # bMcom 
    com0_x = 0.2
    com0_y = 0.00044412
    com0_z = 0.19240999
    com_phase, dcom_phase = traj_generator([time_0, time_1],
      [[0.2, 0.2], [0.00044412, 0.00044412], [0.19240999, 0.19240999]], phase0)

    ## HL
    # oMhl ('hl: ', matrix([[ 0.01      ,  0.14205   , -0.00294615]]))
    # bMhl ('hl: ', matrix([[-0.19      ,  0.14205   , -0.22294615]]))
    hl_phase, dhl_phase = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.1, -0.22294615]], phase0)

    ## HR
    # ('hr: ', matrix([[ 0.01      , -0.14205   , -0.00294615]]))
    # bMhr ('hr: ', matrix([[-0.19      , -0.14205   , -0.22294615]]))
    hr_phase, dhr_phase = traj_generator([time_0, time_1],
      [[0.0, 0.0], [0.0, 0.0], [-0.1, -0.22294615]], phase0)

    ### FL
    # ('fl: ', matrix([[ 0.39      ,  0.14205   , -0.00294615]]))
    # bMfl ('fl: ', matrix([[ 0.19      ,  0.14205   , -0.22294615]]))
    fl_phase, dfl_phase = traj_generator([time_0, time_1],
      [[0.1, 0.1], [0.0, 0.0], [0.2, 0.2]], phase0)
    
    ### FR
    # ('fr: ', matrix([[ 0.39      , -0.14205   , -0.00294615]]))
    # bMfr ('fr: ', matrix([[ 0.19      , -0.14205   , -0.22294615]]))
    fr_phase, dfr_phase = traj_generator([time_0, time_1],
      [[0.1, 0.1], [0.0, 0.0], [0.2, 0.2]], phase0)

    com = np.concatenate((com, com_phase), axis=1)
    hl = np.concatenate((hl, hl_phase), axis=1)
    dhl = np.concatenate((dhl, dhl_phase), axis=1)
    hr = np.concatenate((hr, hr_phase), axis=1)
    dhr = np.concatenate((dhr, dhr_phase), axis=1)
    fl = np.concatenate((fl, fl_phase), axis=1)
    dfl = np.concatenate((dfl, dfl_phase), axis=1)
    fr = np.concatenate((fr, fr_phase), axis=1)
    dfr = np.concatenate((dfr, dfr_phase), axis=1)

    ### creation of the data files #############################################
    print ("com.shape", com.shape)
    print ("time.shape", time.shape)
    quadruped_com = np.vstack((time, com))
    np.savetxt("/tmp/quadruped_com.dat", quadruped_com.T, delimiter=" ")
    #
    quadruped_positions_eff = np.vstack((
      fl, np.zeros((3,nb_pt)), fr, np.zeros((3,nb_pt)),
      hl, np.zeros((3,nb_pt)), hr, np.zeros((3,nb_pt)),
    ))
    np.savetxt("/tmp/quadruped_positions_eff.dat", quadruped_positions_eff.T, delimiter=" ")
    #
    quadruped_velocities_eff = np.vstack((
      dfl, np.zeros((3,nb_pt)), dfr, np.zeros((3,nb_pt)),
      dhl, np.zeros((3,nb_pt)), dhr, np.zeros((3,nb_pt)),
    ))
    np.savetxt("/tmp/quadruped_velocities_eff.dat", quadruped_velocities_eff.T, delimiter=" ")

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
    plt.figure("pos")
    plt.plot(time, fl[0,:], ':',
             time, fl[1,:], '-',
             time, fl[2,:], '--',
             time, fr[0,:], ':',
             time, fr[1,:], '-',
             time, fr[2,:], '--')
    plt.legend(['fl_x', 'fl_y', 'fl_z', 'fr_x', 'fr_y', 'fr_z'], loc='best')
    #
    plt.figure("vel")
    plt.plot(time, dfl[0,:], ':',
             time, dfl[1,:], '-',
             time, dfl[2,:], '--',
             time, dfr[0,:], ':',
             time, dfr[1,:], '-',
             time, dfr[2,:], '--')
    plt.legend(['fl_dx', 'fl_dy', 'fl_dz', 'fr_dx', 'fr_dy', 'fr_dz'], loc='best')

    plt.show()

