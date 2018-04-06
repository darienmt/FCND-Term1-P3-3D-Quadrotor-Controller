"""
PID Controller

components:
    follow attitude commands
    gps commands and yaw
    waypoint following
"""
import numpy as np
from frame_utils import euler2RM
from math import sin, cos, tan, sqrt

DRONE_MASS_KG = 0.5
GRAVITY = -9.81
MOI = np.array([0.005, 0.005, 0.01])
MAX_THRUST = 10.0
MAX_TORQUE = 1.0

class NonlinearController(object):

    def __init__(self):
        """Initialize the controller object and control gains"""

        # Body-rate parameters
        self.body_rate_k_p = np.array([0.01, 0.01, 0.01])

        # Altitude controller parameters
        self.altitude_delta = 0.7  # 0.7 < delta < 1.0
        self.altitude_t_rise = 0.01
        return

    def trajectory_control(self, position_trajectory, yaw_trajectory, time_trajectory, current_time):
        """Generate a commanded position, velocity and yaw based on the trajectory

        Args:
            position_trajectory: list of 3-element numpy arrays, NED positions
            yaw_trajectory: list yaw commands in radians
            time_trajectory: list of times (in seconds) that correspond to the position and yaw commands
            current_time: float corresponding to the current time in seconds

        Returns: tuple (commanded position, commanded velocity, commanded yaw)

        """

        ind_min = np.argmin(np.abs(np.array(time_trajectory) - current_time))
        time_ref = time_trajectory[ind_min]


        if current_time < time_ref:
            position0 = position_trajectory[ind_min - 1]
            position1 = position_trajectory[ind_min]

            time0 = time_trajectory[ind_min - 1]
            time1 = time_trajectory[ind_min]
            yaw_cmd = yaw_trajectory[ind_min - 1]

        else:
            yaw_cmd = yaw_trajectory[ind_min]
            if ind_min >= len(position_trajectory) - 1:
                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min]

                time0 = 0.0
                time1 = 1.0
            else:

                position0 = position_trajectory[ind_min]
                position1 = position_trajectory[ind_min + 1]
                time0 = time_trajectory[ind_min]
                time1 = time_trajectory[ind_min + 1]

        position_cmd = (position1 - position0) * \
                        (current_time - time0) / (time1 - time0) + position0
        velocity_cmd = (position1 - position0) / (time1 - time0)

        # return (position_cmd, velocity_cmd, yaw_cmd)
        return (np.array([0.0, 0.0, -3.0]), velocity_cmd, yaw_cmd)

    def lateral_position_control(self, local_position_cmd, local_velocity_cmd, local_position, local_velocity,
                               acceleration_ff = np.array([0.0, 0.0])):
        """Generate horizontal acceleration commands for the vehicle in the local frame

        Args:
            local_position_cmd: desired 2D position in local frame [north, east]
            local_velocity_cmd: desired 2D velocity in local frame [north_velocity, east_velocity]
            local_position: vehicle position in the local frame [north, east]
            local_velocity: vehicle velocity in the local frame [north_velocity, east_velocity]
            acceleration_cmd: feedforward acceleration command

        Returns: desired vehicle 2D acceleration in the local frame [north, east]
        """
        return np.array([0.0, 0.0])

    def R(self, altitude):
        """
        Calculate the rotation matrix for altitude (roll, pitch, yaw)
        (phi, theta, psi)
        """
        phi, theta, psi = altitude
        r_x = np.array([[1, 0, 0],
                    [0, cos(phi), -sin(phi)],
                    [0, sin(phi), cos(phi)]])

        r_y = np.array([[cos(theta), 0, sin(theta)],
                        [0, 1, 0],
                        [-sin(theta), 0, cos(theta)]])

        r_z = np.array([[cos(psi), -sin(psi), 0],
                        [sin(psi), cos(psi), 0],
                        [0,0,1]])

        return np.matmul(r_z,np.matmul(r_y,r_x))

    def altitude_control(self, altitude_cmd, vertical_velocity_cmd, altitude, vertical_velocity, attitude, acceleration_ff=0.0):
        """Generate vertical acceleration (thrust) command

        Args:
            altitude_cmd: desired vertical position (+up)
            vertical_velocity_cmd: desired vertical velocity (+up)
            altitude: vehicle vertical position (+up)
            vertical_velocity: vehicle vertical velocity (+up)
            attitude: the vehicle's current attitude, 3 element numpy array (roll, pitch, yaw) in radians
            acceleration_ff: feedforward acceleration command (+up)

        Returns: thrust command for the vehicle (+up)
        """
        z_err = altitude_cmd - altitude
        z_err_dot = vertical_velocity_cmd - vertical_velocity
        b_z = self.R(attitude)[2,2]

        wn = 1.57/self.altitude_t_rise
        k_p = wn*wn
        k_d = 2*self.altitude_delta*wn

        u_1 = k_p * z_err + k_d * z_err_dot + acceleration_ff

        return (u_1 + GRAVITY)/b_z


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        return np.array([0.0, 0.0])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        return np.multiply(self.body_rate_k_p, body_rate_cmd - body_rate)

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """
        return 0.0
