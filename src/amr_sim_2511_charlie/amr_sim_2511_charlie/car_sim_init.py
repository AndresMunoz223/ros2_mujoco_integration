try:
#   print('Revisando instalación....')
  import mujoco
  mujoco.MjModel.from_xml_string('<mujoco/>')
except Exception as e:
  raise e from RuntimeError(
      'Algo salio mal en la instalación (pip install mujoco)')

# print('Revisión realizada.')

# Other imports and helper functions
import time
import threading
import numpy as np
from math import cos, sin


import os
from glob import glob
from ament_index_python.packages import get_package_share_directory
# import keyboard

# Graphics and plotting.
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from terminaltexteffects.effects.effect_beams import Beams


# ! Generating odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

import matplotlib.pyplot as plt
import cv2 
# mejores prints
np.set_printoptions(precision=3, suppress=True, linewidth=100)

#! ------------------------Descripción--------------------------

package_name = 'mujoco_sim'

MJCF = os.path.join(
                get_package_share_directory(package_name),'descriptions','robot_differential.xml')

#!---------------------------------------------------------------------------------



def euler_to_quaternion(roll, pitch, yaw):
    """
    Converts Euler angles (roll, pitch, yaw) to a quaternion (x, y, z, w).

    Args:
        roll (float): Rotation around the x-axis (in radians).
        pitch (float): Rotation around the y-axis (in radians).
        yaw (float): Rotation around the z-axis (in radians).

    Returns:
        tuple: A tuple representing the quaternion (x, y, z, w).
    """

    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (x, y, z, w)


class MujocoSimulator(Node):
    def __init__(self, Description):
        super().__init__('Simulator_pendulum')

        #* User interaction side
        self.twist_sub = self.create_subscription(Twist,"/cmd_vel_mux_output", self.__twist_callback, 10)
        self.u_input = 0.
        self.w_input = 0.

        #* Odometry publishing side
        self.laser_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        self.odom_timer = self.create_timer(0.05, self.__publish_odom_transform)
        self.laser_timer = self.create_timer(0.1, self.__publish_laser_scan)

        #* Transform publishing side 
        self.tf_broadcaster = TransformBroadcaster(self)
        self.parent_frame = 'odom'
        self.child_frame = 'base_link'

        self.x_coordinate_reconstruct = 0
        self.y_coordinate_reconstruct = 0
        self.theta_angle_reconstruct = 0
        
        #* Simulator parameters
        self.simulation_object = mujoco.MjModel.from_xml_path(Description)
        self.simulation_data = mujoco.MjData(self.simulation_object)
        self.render_height = 480
        self.render_width = 480
        self.video_fps = 60
        self.sim_time = 30
        self.simulation_object.opt.timestep = 0.005
        self.lidar_measurements = []

        #* Camera and simulation objects
        self.car_camera_name = "top_view"
        camera_id = self.simulation_object.camera(self.car_camera_name).id
        self.car_camera = mujoco.MjvCamera()
        self.car_camera.type = mujoco.mjtCamera.mjCAMERA_FIXED
        self.car_camera.fixedcamid = camera_id


        self.rrw_angle_prev = 0
        self.lrw_angle_prev = 0


    def __publish_laser_scan(self):

        msg = LaserScan()
        msg.header.stamp = self.get_clock().now().to_msg()

        pass

    def __publish_odom_transform(self):

        # Create the transform message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Robot's position in the odom frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        roll = 0.0
        pitch = 0.0
        yaw = self.theta
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw


        #! Create the lidar transform
        self.tf_broadcaster.send_transform(t)
        self.get_logger().info(f'Published transform from {self.parent_frame} to {self.child_frame}')

                t.header.frame_id = self.parent_frame
        t.child_frame_id = self.child_frame

        # Robot's position in the odom frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0

        roll = 0.0
        pitch = 0.0
        yaw = self.theta
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw


        # Publish the transform
        self.tf_broadcaster.send_transform(t)
        self.get_logger().info(f'Published transform from {self.parent_frame} to {self.child_frame}')


    def __twist_callback(self, data):
        self.u_input = data.linear.x*4.
        self.w_input = data.angular.z*4.

    def get_odom(self):

        dlrw = self.simulation_data.qpos[8] - self.lrw_angle_prev
        drrw = self.simulation_data.qpos[7] - self.rrw_angle_prev 

        self.theta_angle_reconstruct += .05*(drrw - dlrw) / (2.*0.12)

        ddistance = .05*(drrw + dlrw) / (2.)

        self.x_coordinate_reconstruct += ddistance*cos(self.theta_angle_reconstruct)
        self.y_coordinate_reconstruct += ddistance*sin(self.theta_angle_reconstruct)

        self.lrw_angle_prev = self.simulation_data.qpos[8]
        self.rrw_angle_prev = self.simulation_data.qpos[7]

    def start_sim(self):
        #* Helper functs

        # effect = Beams("""Iniciando simulador....                        
        # """)
        # with effect.terminal_output() as terminal:
        #     for frame in effect:
        #         terminal.print(frame)

        n_frames = self.sim_time*self.video_fps
        frames = []
        times = []
        sensordata = []

        #* Señal para nuestro motor
        mujoco.mj_resetData(self.simulation_object, self.simulation_data)
        self.simulation_data.ctrl[0] = 0.1
        self.simulation_data.ctrl[1] = 0.1


        with mujoco.Renderer(self.simulation_object, self.render_height, self.render_width) as renderer:
            renderer._scene_option.flags[mujoco.mjtVisFlag.mjVIS_RANGEFINDER] = False
            for i in range(n_frames):
                
                while self.simulation_data.time < i/self.video_fps:

                    self.simulation_data.ctrl[0] = self.u_input - self.w_input
                    self.simulation_data.ctrl[1] = self.u_input + self.w_input

                    if (self.simulation_data.time > 3): 
                        self.get_odom()

                    #* Propagate simulation
                    mujoco.mj_step(self.simulation_object, self.simulation_data)

                    #* Obtaining lidar measurements
                    self.lidar_measurements = self.simulation_data.sensordata[3:]

                    #* Append time
                    times.append(self.simulation_data.time)
                    sensordata.append(self.simulation_data.sensor('accelerometer').data.copy())



                renderer.update_scene(self.simulation_data, "fixed")

                img_frame = renderer.render()
                img_frame = cv2.cvtColor(img_frame,cv2.COLOR_BGR2RGB)

                renderer.update_scene(self.simulation_data, self.car_camera_name)

                car_frame = renderer.render()
                car_frame = cv2.cvtColor(car_frame,cv2.COLOR_BGR2RGB)
                
                cv2.imshow("Global",img_frame)
                cv2.imshow("Car_perspective", car_frame)
                cv2.waitKey(25) & 0xFF == ord('q') # Exit if 'q' is pressed
                # frames.append(frame)

            cv2.destroyAllWindows()



def main():
    rclpy.init(args=None)
    simulator = MujocoSimulator(MJCF)
    t = threading.Thread(target=simulator.start_sim, args=())
    t.start()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()