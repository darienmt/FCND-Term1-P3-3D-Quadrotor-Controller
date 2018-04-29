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

    def better_pid_config(self, t_rise, delta):
        """
        From `t_rise` and `delta` returns kp and kd
        """
        w = 1/(1.57*t_rise)
        return w * w, 2 * delta * w

    def __init__(self):
        """Initialize the controller object and control gains"""

        # Body-rate controller parameters
        self.body_rate_k_p = np.array([20., 20., 5.])

        # Altitude controller parameters
        self.altitude_k_p, self.altitude_k_d = self.better_pid_config(.1, .85)
        # Yaw controller parameters
        self.yaw_k_p = 4.5

        # Roll-pitch controller parameters
        self.roll_pitch_k_p_roll = 7.
        self.roll_pitch_k_p_pitch = 7.

        # Lateral controller parameters
        self.lateral_k_p, self.lateral_k_d = self.better_pid_config(.28, .95)
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

        return (position_cmd, velocity_cmd, yaw_cmd)

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

        err_p = local_position_cmd - local_position
        err_dot = local_velocity_cmd - local_velocity

        return self.lateral_k_p * err_p + self.lateral_k_d * err_dot + acceleration_ff

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
        b_z = euler2RM(*attitude)[2,2]

        u_1 = self.altitude_k_p * z_err + self.altitude_k_d * z_err_dot + acceleration_ff
        acc = (u_1 - GRAVITY)/b_z

        thrust = DRONE_MASS_KG * acc
        if thrust > MAX_THRUST:
            thrust = MAX_THRUST
        else:
            if thrust < 0.:
                thurst = 0.

        return thrust


    def roll_pitch_controller(self, acceleration_cmd, attitude, thrust_cmd):
        """ Generate the rollrate and pitchrate commands in the body frame

        Args:
            target_acceleration: 2-element numpy array (north_acceleration_cmd,east_acceleration_cmd) in m/s^2
            attitude: 3-element numpy array (roll, pitch, yaw) in radians
            thrust_cmd: vehicle thruts command in Newton

        Returns: 2-element numpy array, desired rollrate (p) and pitchrate (q) commands in radians/s
        """
        if thrust_cmd > 0.:
            c = - thrust_cmd / DRONE_MASS_KG;
            b_x_c, b_y_c = np.clip(acceleration_cmd / c, -1., 1)

            rot_mat = euler2RM(*attitude)

            b_x = rot_mat[0, 2]
            b_x_err = b_x_c - b_x
            b_x_p_term = self.roll_pitch_k_p_roll * b_x_err

            b_y = rot_mat[1,2]
            b_y_err = b_y_c - b_y
            b_y_p_term = self.roll_pitch_k_p_pitch * b_y_err

            b_x_commanded_dot = b_x_p_term
            b_y_commanded_dot = b_y_p_term

            rot_mat1=np.array([[rot_mat[1,0],-rot_mat[0,0]],[rot_mat[1,1],-rot_mat[0,1]]])/rot_mat[2,2]

            rot_rate = np.matmul(rot_mat1,np.array([b_x_commanded_dot,b_y_commanded_dot]).T)
            p_c = rot_rate[0]
            q_c = rot_rate[1]
            # print(f'{b_x_err} {b_y_err}')
            return np.array([p_c, q_c])
        else:
            return np.array([0., 0.])

    def body_rate_control(self, body_rate_cmd, body_rate):
        """ Generate the roll, pitch, yaw moment commands in the body frame

        Args:
            body_rate_cmd: 3-element numpy array (p_cmd,q_cmd,r_cmd) in radians/second^2
            body_rate: 3-element numpy array (p,q,r) in radians/second^2

        Returns: 3-element numpy array, desired roll moment, pitch moment, and yaw moment commands in Newtons*meters
        """
        taus = MOI * np.multiply(self.body_rate_k_p, ( body_rate_cmd - body_rate ))

        taus_mod = np.linalg.norm(taus)

        if taus_mod > MAX_TORQUE: # Normalize!
            taus = taus * MAX_TORQUE / taus_mod

        return taus

    def yaw_control(self, yaw_cmd, yaw):
        """ Generate the target yawrate

        Args:
            yaw_cmd: desired vehicle yaw in radians
            yaw: vehicle yaw in radians

        Returns: target yawrate in radians/sec
        """

        yaw_error = yaw_cmd - yaw
        if yaw_error > np.pi:
            yaw_error = yaw_error - 2.0*np.pi
        elif yaw_error < -np.pi:
            yaw_error = yaw_error + 2.0*np.pi

        yawrate_cmd = self.yaw_k_p*yaw_error
        return yawrate_cmd
