import rospy
from bitbots_msgs.msg import FootPressure, JointCommand
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from roscpp.srv import Empty
from rosgraph_msgs.msg import Clock
from bitbots_msgs.srv import SetRobotPose
from sensor_msgs.msg import JointState, Imu, Image, CameraInfo
from wolfgang_webots_sim.webots_robot_controller import RobotController
from wolfgang_webots_sim.webots_supervisor_controller import SupervisorController


class LearningController(SupervisorController, RobotController):
    """Modified controller that allows running in a namespace, which is needed for learning"""

    def __init__(self, own_node=False, mode='normal', robot='wolfgang', namespace="", camera_active=False):
        # first initialize the controllers without ros
        SupervisorController.__init__(self, ros_active=False, mode=mode)
        # now add the necessary parts manually
        if own_node:
            rospy.init_node("webots_ros_supervisor", argv=['clock:=' + namespace + '/clock'])
        self.clock_publisher = rospy.Publisher(namespace + "/clock", Clock, queue_size=1)
        self.model_state_publisher = rospy.Publisher(namespace + "/model_states", ModelStates, queue_size=1)
        self.reset_service = rospy.Service(namespace + "/reset", Empty, self.reset)
        self.initial_poses_service = rospy.Service(namespace + "/initial_pose", Empty, self.set_initial_poses)
        self.set_robot_position_service = rospy.Service(namespace + "/set_robot_position", SetRobotPose,
                                                        self.robot_pose_callback)
        self.reset_ball_service = rospy.Service(namespace + "/reset_ball", Empty, self.reset_ball)
        # This is used so that SupervisorController and RobotController can use the same underlying controller
        self.robot_node = self.supervisor
        RobotController.__init__(self, ros_active=False, robot=robot, do_ros_init=False, external_controller=True)
        self.l_sole_frame = rospy.get_param(namespace + "/l_sole_frame", "l_sole")
        self.r_sole_frame = rospy.get_param(namespace + "/r_sole_frame", "r_sole")
        self.camera_optical_frame = rospy.get_param(namespace + "/camera_optical_frame", "camera_optical_frame")
        self.head_imu_frame = rospy.get_param(namespace + "/head_imu_frame", "imu_frame_2")
        self.imu_frame = rospy.get_param(namespace + "/imu_frame", "imu_frame")
        self.pub_js = rospy.Publisher(namespace + "/joint_states", JointState, queue_size=1)
        self.pub_imu = rospy.Publisher(namespace + "/imu/data_raw", Imu, queue_size=1)

        self.pub_imu_head = rospy.Publisher(namespace + "/imu_head/data", Imu, queue_size=1)
        self.pub_pres_left = rospy.Publisher(namespace + "/foot_pressure_left/filtered", FootPressure, queue_size=1)
        self.pub_pres_right = rospy.Publisher(namespace + "/foot_pressure_right/filtered", FootPressure, queue_size=1)
        self.cop_l_pub_ = rospy.Publisher(namespace + "/cop_l", PointStamped, queue_size=1)
        self.cop_r_pub_ = rospy.Publisher(namespace + "/cop_r", PointStamped, queue_size=1)
        rospy.Subscriber(namespace + "/DynamixelController/command", JointCommand, self.command_cb)

        if camera_active:
            self.pub_cam = rospy.Publisher(namespace + "/camera/image_proc", Image, queue_size=1)
            self.pub_cam_info = rospy.Publisher(namespace + "/camera/camera_info", CameraInfo, queue_size=1, latch=True)
            self.cam_info = CameraInfo()
            self.cam_info.header.stamp = rospy.Time.from_seconds(self.time)
            self.cam_info.header.frame_id = self.camera_optical_frame
            self.cam_info.height = self.camera.getHeight()
            self.cam_info.width = self.camera.getWidth()
            f_y = self.mat_from_fov_and_resolution(
                self.h_fov_to_v_fov(self.camera.getFov(), self.cam_info.height, self.cam_info.width),
                self.cam_info.height)
            f_x = self.mat_from_fov_and_resolution(self.camera.getFov(), self.cam_info.width)
            self.cam_info.K = [f_x, 0, self.cam_info.width / 2,
                               0, f_y, self.cam_info.height / 2,
                               0, 0, 1]
            self.cam_info.P = [f_x, 0, self.cam_info.width / 2, 0,
                               0, f_y, self.cam_info.height / 2, 0,
                               0, 0, 1, 0]
            self.pub_cam_info.publish(self.cam_info)

        self.ros_active = True

    def step(self):
        super().step()
        if self.ros_active:
            RobotController.publish_ros(self)
