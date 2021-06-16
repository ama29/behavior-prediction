"""

Extended kalman filter (EKF) localization sample

author: Atsushi Sakai (@Atsushi_twi)

"""
import math
import random

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot


# Covariance for EKF simulation
Q = np.diag([
    0.1,  # variance of location on x-axis
    0.1,  # variance of location on y-axis
    np.deg2rad(1.0),  # variance of yaw angle
    1.0  # variance of velocity
]) ** 2  # predict state covariance
R = np.diag([1.0, 1.0]) ** 2  # Observation x,y position covariance

#  Simulation parameter
INPUT_NOISE = np.diag([1.0, np.deg2rad(30.0)]) ** 2
GPS_NOISE = np.diag([0.5, 0.5]) ** 2

DT = 0.1  # time tick [s]
SIM_TIME = 50.0  # simulation time [s]

show_animation = True

def calc_input():
    v = 1.0  # [m/s]
    yawrate = 0.1  # [rad/s]
    u = np.array([[v], [yawrate]])
    return u


def observation(xTrue, xd, u, timestep, trajectory):
    xTrue = trajectory[timestep]
    

    # add noise to gps x-y
    z = observation_model(xTrue) + GPS_NOISE @ np.random.randn(2, 1)

    # add noise to input
    ud = u + INPUT_NOISE @ np.random.randn(2, 1)

    xd = motion_model(xd, ud)

    return xTrue, z, xd, ud


def motion_model(x, u):
    F = np.array([[1.0, 0, 0, 0],
                  [0, 1.0, 0, 0],
                  [0, 0, 1.0, 0],
                  [0, 0, 0, 0]])

    B = np.array([[DT * math.cos(x[2, 0]), 0],
                  [DT * math.sin(x[2, 0]), 0],
                  [0.0, DT],
                  [1.0, 0.0]])

    x = F @ x + B @ u
    
    return x


def observation_model(x):
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    z = H @ x

    return z


def jacob_f(x, u):
    """
    Jacobian of Motion Model

    motion model
    x_{t+1} = x_t+v*dt*cos(yaw)
    y_{t+1} = y_t+v*dt*sin(yaw)
    yaw_{t+1} = yaw_t+omega*dt
    v_{t+1} = v{t}
    so
    dx/dyaw = -v*dt*sin(yaw)
    dx/dv = dt*cos(yaw)
    dy/dyaw = v*dt*cos(yaw)
    dy/dv = dt*sin(yaw)
    """
    yaw = x[2, 0]
    v = u[0, 0]
    jF = np.array([
        [1.0, 0.0, -DT * v * math.sin(yaw), DT * math.cos(yaw)],
        [0.0, 1.0, DT * v * math.cos(yaw), DT * math.sin(yaw)],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0]])

    return jF


def jacob_h():
    # Jacobian of Observation Model
    jH = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])

    return jH


def ekf_estimation(xEst, PEst, z, u):
    #  Predict
    xPred = motion_model(xEst, u)
    jF = jacob_f(xEst, u)
    PPred = jF @ PEst @ jF.T + Q

    #  Update
    jH = jacob_h()
    zPred = observation_model(xPred)
    y = z - zPred
    S = jH @ PPred @ jH.T + R
    K = PPred @ jH.T @ np.linalg.inv(S)
    xEst = xPred + K @ y
    PEst = (np.eye(len(xEst)) - K @ jH) @ PPred
    return xEst, PEst


def plot_covariance_ellipse(xEst, PEst):  # pragma: no cover
    Pxy = PEst[0:2, 0:2]
    eigval, eigvec = np.linalg.eig(Pxy)

    if eigval[0] >= eigval[1]:
        bigind = 0
        smallind = 1
    else:
        bigind = 1
        smallind = 0

    t = np.arange(0, 2 * math.pi + 0.1, 0.1)
    a = math.sqrt(eigval[bigind])
    b = math.sqrt(eigval[smallind])
    x = [a * math.cos(it) for it in t]
    y = [b * math.sin(it) for it in t]
    angle = math.atan2(eigvec[1, bigind], eigvec[0, bigind])
    rot = Rot.from_euler('z', angle).as_matrix()[0:2, 0:2]
    fx = rot @ (np.array([x, y]))
    px = np.array(fx[0, :] + xEst[0, 0]).flatten()
    py = np.array(fx[1, :] + xEst[1, 0]).flatten()
    plt.plot(px, py, "--r")


def plot_extended_kalman(agents):
    time = 0.0
    timestep = 0
    
    trajectories = {}
    
    # State Vector [x y yaw v]'
    xEst = np.zeros((4, 1))
    xTrue = np.zeros((4, 1))
    PEst = np.eye(4)
    xDR = np.zeros((4, 1))  # Dead reckoning
    z = np.zeros((2,1))

    # traj, xEst, xTrue, PEst, xDR, hz
    for i in range(len(agents)):
        trajectories[i] = [agents[i], xEst, xTrue, PEst, xDR, z]

    histories = {}
    for agent_id, traj in trajectories.items():
        histories[agent_id] = [traj, xEst, xTrue, PEst, xDR, z]
        
    while SIM_TIME >= time:
        time += DT
        timestep += 1
        u = calc_input()

        for agent_id, traj in trajectories.items():
            traj[2], traj[5], traj[4], ud = observation(traj[2], traj[4], u, timestep, traj[0])
            traj[1], traj[3] = ekf_estimation(traj[1], traj[3], traj[5], ud)

            # push the new estimations, true trajectories, predictions onto the stack
            hxEst = np.hstack((histories[agent_id][1], traj[1]))
            hxDR = np.hstack((histories[agent_id][4], traj[4]))
            hxTrue = np.hstack((histories[agent_id][2], traj[2]))
            hz = np.hstack((histories[agent_id][5], traj[5]))

            histories[agent_id] = [traj[0], hxEst, hxTrue, PEst, hxDR, hz]

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
            # estimated trajectory in red, actual trajectory in black, confidence ellipse in red
            for agent_id, traj in histories.items():
                plt.plot(traj[5][0, :], traj[5][1, :], ".g")
                plt.plot(traj[2][0, :].flatten(),
                     traj[2][1, :].flatten(), "-b")
                plt.plot(traj[1][0, :].flatten(),
                     traj[1][1, :].flatten(), "-r")
                plot_covariance_ellipse(trajectories[agent_id][1], trajectories[agent_id][3])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.001)


#if __name__ == '__main__':
#    main()
