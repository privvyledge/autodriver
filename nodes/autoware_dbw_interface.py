#!/usr/bin/env python
"""
See these for details:
    https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-interface/
    https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/vehicle/
    https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-interfaces/components/vehicle-dimensions/

See these for implementation examples:
    https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/ne_raptor_interface/src/ne_raptor_interface.cpp
    https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/ssc_interface/src/ssc_interface.cpp


Miscellaneous: https://www.autoware.org/_files/ugd/984e93_b4e43111b1dd420cb3bc2c3e13071e71.pdf?index=true
Todo: create DBW state machine/watchdog, create sensor state machine/watchdog, create overall state machine/watchdog, odom publisher.
For Autoware.Auto state machine, see: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/drivers/vehicle_interface

Todo:
    Get Autoware units [Done]
    Get dbw steering units [Done]
    Get dbw speed units [Done]
    Add steering wheel conversion methods [Done]
    Receive commands from Autoware [Done]
    Implement/send steering commands to DBW, i.e choose steering wheel commands [Done]
    Get DBW state. Check DBW enabled [Done]
    Handle mode change request [Done]
    Implement switch case statements [Done]
    Send mode to DBW [Done]
    Process/prepare commands to DBW [Done]
    Receive commands from DBW [Done]
    Send mode to Autoware [Done]
    Process/prepare commands to Autoware Vehicle Status [Done]
    Implement ne_raptor_interface odometry [Done]
    Implement kinematic_bicycle_model [Done]
    Decide how I will publish commands [on_change {from subscriber}, at constant rate {timer}, on_change+constant rate {timer plus time check}) [Done]
    Implement publishing modes from above [Done]
    Make sure all message variables are updated (VehicleStatus, Odometry, Raptor, VehicleCmd) [Done]
    Send commands to DBW [Done]
    Send commands to Autoware Vehicle Status [Done]
    Send commands to Autoware Odom [Done]
    Add option to publish odom tf [Done]
    Create state report message or find Autoware.AI's, i.e VehicleKinematicState [Done]
    Compile message [Done]
    Add to state report message [Done]
    Publish to Vehicle Kinematic State message [Done]
    Test all of the above [Done]
    Fix Quarternion type/format issues [Done]
    Find out why Odometry is not published [Done]
    Fix bug where RosException is called in callbacks (like cmdCallback) because Ctrl-C is called [Done]
    Create Dbw State Machine [Done]
    Create Dbw State message [Done]
    Create Dbw State Machine message [Done]
    Send feedback to state machine [Done]
    Test Dbw State message [Done]
    Test Dbw State machine message [Done]
    Test Dbw State machine [Done]
    Publish to DbwStateMachine message [Done]
    Test all the above [Done]
    Create vehicle_state_report message [Done]
    Compare steering angle, steering wheel angle, curvature and yaw rate [Done]
    Output all speed conversions and find correct units for misc_report speed [Done]
    Find what report message is updated when obstacles are near the car.
    Analyze dbw rosbag data to see reverse message [Done]
    Publish to vehicle state report
    Create Vehicle State message
    Create Vehicle State Machine
    Create Vehicle State Machine message
    Test Vehicle State machine
    Implement mode change request
    Use mode command to mux between this node and teleop node
    Insert missing lines
    Initialize all values
    Remove placeholders/defaults
    Compare final code and ensure it's the same as Autoware.Auto NE Raptor interface (check rospy.log format)
    Implement automatic gear shift logic based on speed for Autoware and DBW
    Test localization
    Test and visualize
    Remove remaining todos
    Split to two nodes
    Clean up, make more pythonic and efficient
    Check race conditions
    Find out what header is published as if nothing is specified for header but other messages are populated
    Import myOdometry package
    Fix ne_raptor odometry based on location of baselink (cog or rear_axle)
    Create base VehicleInterface node and let this inherit from that.

"""

# import legacy modules first (compulsorily at the top)
from __future__ import division, print_function, absolute_import

# standard library imports
import math
from math import sin, cos, tan, pi, fabs, atan, atan2
import sys

# third-party library imports
import rospy
import tf  # todo: switch to tf2_ros and tf_conversions

# import ros messages
from std_msgs.msg import Empty, Bool
from nav_msgs.msg import Odometry
# from sensor_msgs.msg import Joy, Imu
# from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

# import custom modules (Autoware Control and NewEagle to control both simultaneously).
# New Eagle Messages
from raptor_dbw_msgs.msg import AcceleratorPedalReport, Brake2Report, BrakeReport, GearReport, GlobalEnableCmd, \
    Steering2Report, SteeringReport, WheelPositionReport, WheelSpeedReport, WheelSpeedType
from raptor_dbw_msgs.msg import DoorState, DriverInputReport, FaultActionsReport, HighBeamState, HmiGlobalEnableReport
from raptor_dbw_msgs.msg import HornState, Ignition, LowVoltageSystemReport, MiscReport, OtherActuatorsReport
from raptor_dbw_msgs.msg import ParkingBrake, TirePressureReport, TurnSignal, WatchdogStatus, ActuatorControlMode
from raptor_dbw_msgs.msg import Gear as RaptorGear

from raptor_dbw_msgs.msg import AcceleratorPedalCmd, SteeringCmd, GearCmd, MiscCmd, GlobalEnableCmd
from raptor_dbw_msgs.msg import ActuatorControlMode, TurnSignal
from raptor_dbw_msgs.msg import Gear as RaptorGear
from raptor_dbw_msgs.msg import BrakeCmd as BrakePedalCmd
from raptor_dbw_msgs.msg import Brake2Report
from raptor_dbw_msgs.msg import DoorRequest, WiperRear, Ignition, TurnSignal, LowBeam, HighBeam, WiperFront

# Autoware Messages (https://github.com/autowarefoundation/autoware_ai_messages/tree/master/autoware_msgs/msg)
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped, Twist
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3, Vector3Stamped, PoseWithCovarianceStamped
from autoware_msgs.msg import AccelCmd, BrakeCmd, ControlCommand, ControlCommandStamped, SteerCmd, VehicleCmd, \
    VehicleStatus
from autoware_msgs.msg import Gear as AutowareGear
from autoware_system_msgs.msg import NodeStatus

# Autodriver messages
from autodriver.msg import VehicleKinematicState
from autodriver.msg import DbwStateMachine as DbwStateMachineMsg
from autodriver.msg import DbwState as DbwStateMsg

# DbwStateMachine
from DbwStateMachine import DbwStateMachine
from autodriver.DbwState import DbwState

__author__ = 'Boluwatife Olabiran'
__license__ = 'GPLv3'
__maintainer__ = 'Boluwatife Olabiran'
__email__ = 'bso19a@fsu.edu'


class AutowareDbwInterfaceNode(object):
    """docstring for ClassName"""

    def __init__(self, ):
        """Constructor for AutowareDbwInterfaceNode"""
        super(AutowareDbwInterfaceNode, self).__init__()

        # parameters.
        # ####### vehicle_info parameters
        self.WHEELBASE = float(rospy.get_param('~wheelbase', 3.08864))  # m.

        # ####### ne_raptor parameters
        self.ecu_build_num = int(rospy.get_param('~ecu_build_number', 0xFFFF))  #
        self.front_axle_to_cog = float(rospy.get_param('~front_axle_to_cog', 1.3744448))  # m
        self.rear_axle_to_cog = float(rospy.get_param('~rear_axle_to_cog', 1.7141952))  # m
        self.yaw_moment_of_inertia = float(rospy.get_param('~yaw_moment_of_inertia', 4271.217627996))  # kg.m^2
        self.steer_to_tire_ratio = float(
            rospy.get_param('~steer_to_tire_ratio', 15.667))  # # 470 degrees on steering wheel ~= 30 degrees on tire
        # get maximum steering wheel angle in degrees and convert to radians
        self.max_steering_wheel_angle = math.radians(float(rospy.get_param('~max_steering_wheel_angle', 470.0)))  # rads
        self.acceleration_limit = float(rospy.get_param('~acceleration_limit', 1.0))  # m/s^2, 0 = no limit
        self.deceleration_limit = float(rospy.get_param('~deceleration_limit', 1.0))  # m/s^2, 0 = no limit
        self.acceleration_positive_jerk_limit = float(
            rospy.get_param('~acceleration_positive_jerk_limit', 0.0))  # m/s^3
        self.deceleration_negative_jerk_limit = float(
            rospy.get_param('~deceleration_negative_jerk_limit', 0.0))  # m/s^3
        self.dbw_pub_rate = int(
            rospy.get_param('~dbw_pub_rate', 50))  # Hz. 10 - 200. Recommended: 50.
        self.counter = 0

        # dbw control mode parameters
        self.enable_speed_cmd = rospy.get_param('~enable_speed_cmd', True)  # e.g for longitudinal control only
        self.enable_steer_cmd = rospy.get_param('~enable_steer_cmd', True)  # e.g for lateral control only
        self.enable_gear_cmd = rospy.get_param('~enable_gear_cmd', True)  # e.g no gear control like cruise control
        # todo (/test/raptor): test, understand then implement "ignore"
        self.ignore_speed_cmd = rospy.get_param('~ignore_speed_cmd', False)  # e.g for longitudinal control only
        self.ignore_steer_cmd = rospy.get_param('~ignore_steer_cmd', False)  # e.g for lateral control only
        self.ignore_gear_cmd = rospy.get_param('~ignore_gear_cmd', False)  # e.g no gear control like cruise control
        # only closed_loop_vehicle (speed [m/s]) speed_control_type is supported/implemented
        self.speed_control_type = rospy.get_param('~speed_control_type', 'speed')
        # closed_loop_vehicle (curvature) or closed_loop_actuator (steering wheel angular position)
        self.steering_control_type = rospy.get_param('~control_type',
                                                     'curvature')

        # dbw control modes
        if self.speed_control_type.lower() == 'speed':
            self.speed_control_mode = ActuatorControlMode.closed_loop_vehicle  # speed [m/s]
        else:
            rospy.loginfo("Only speed control is supported for acceleration.")
            self.speed_control_mode = ActuatorControlMode.closed_loop_vehicle  # speed [m/s]

        if self.steering_control_type.lower() == 'curvature':
            self.steering_control_mode = ActuatorControlMode.closed_loop_vehicle  # curvature
        elif self.steering_control_type.lower() == 'angle':
            self.steering_control_mode = ActuatorControlMode.closed_loop_actuator  # steering wheel angle

        # ####### state_machine parameters
        # initialize a DbwStateMachine for a car
        self.dbw_state_machine = DbwStateMachine(dbw_disabled_debounce=3)
        self.gear_shift_velocity_threshold_mps = float(rospy.get_param('~gear_shift_velocity_threshold_mps', 0.5))

        # ####### autoware parameters
        self.autoware_pub_rate = int(rospy.get_param('~autoware_pub_rate', 50))  # Hz 30-200. 50 recommended

        # parameters for publish mode (in order of recommendation = change, periodic_change, periodic)
        self.dbw_pub_mode = rospy.get_param('~dbw_pub_mode', 'periodic')  # periodic recommended for Raptor.
        self.autoware_pub_mode = rospy.get_param('~autoware_pub_mode', 'change')  # recommended for Autoware
        self.vehicle_odom_pub_mode = rospy.get_param('~odom_pub_mode', 'change')  # recommended for odometry

        # parameters. mine.
        self.odom_frame_id = rospy.get_param('~odom_frame_id', "odom")
        self.vehicle_status_frame_id = rospy.get_param('~vehicle_frame_id', 'base_link')  # base_link or rear_axle
        self.publish_odom_tf = rospy.get_param('~publish_odom_tf', True)  # Recommended: False. Localization node can fuse and publish better odom. Todo: set to False
        self.vehicle_odom_publish_rate = rospy.get_param('~odom_publish_rate', 50)  # Hz. Might use just for odom_tf

        # declare constants
        self.MS_TO_MPH = 2.23694  # 1 m/s = 2.23694 MPH
        self.MPH_TO_MS = 0.44704  # 1 MPH = 0.44704 m/s
        self.MS_TO_KMPH = 3.6  # 1 m/s = 3.5 KM/h
        self.KMPH_TO_MS = 0.277778  # 1 KM/h = 0.277778 m/s

        # State machine variables
        self.drive_by_wire_enabled = False
        self.by_wire_ready = False
        self.general_driver_activity = True

        # Initialize message variables
        self.accel_cmd = AcceleratorPedalCmd()
        self.brake_cmd = BrakePedalCmd()
        self.steer_cmd = SteeringCmd()
        self.gear_cmd = GearCmd()
        self.misc_cmd = MiscCmd()
        self.global_enable_cmd = GlobalEnableCmd()
        self.empty_cmd = Empty()
        self.vehicle_cmd = VehicleCmd()
        self.vehicle_feedback_status = VehicleStatus()  # feedback data to Autoware (DBW2Autoware data)
        self.vehicle_odom = Odometry()  # feedback data to Autoware (DBW2Autoware data)
        self.vehicle_kinematic_state = VehicleKinematicState()  # complete feedback data
        self.vehicle_dbw_state_message = DbwStateMachineMsg()

        # publisher/subscriber variables
        # autonomy data from Autoware (Autoware2DBW data)
        self.autonomy_data = {'stamp': rospy.Time.now(),
                              # 'brake_pedal_cmd': 0.0,  # [0, 100] %
                              # 'accelerator_pedal_cmd': 0.0,  # [0, 100] %
                              'abs_speed': fabs(0.0),  # m/s
                              'speed_cmd': 0.0,  # m/s,
                              'steering_angle_cmd': 0.0,  # rads. Steer (tire) angle
                              'acceleration_valid': True,# check if change is requested or constant message only. Todo: implement
                              # 'steering_wheel_angle': 0.0,  # rads
                              # 'steering_curvature': 0.0,  # 1 / m
                              'gear_cmd': AutowareGear.NONE,
                              'turn_signal_cmd': 0,  # VehicleStatus.LAMP_LEFT, etc.
                              'mode_cmd': 0,
                              'emergency_cmd': False,
                              'accel_decel_limits': 1.0,  # acceleration/deceleration limit in m/s^2
                              'twist_cmd': TwistStamped(),
                              'road_slope': 0.0}

        # feedback data to Autoware (DBW2Autoware data).
        self.drivemode = 0
        self.steeringmode = 0
        self.speed = 0.0
        self.steering_angle = 0.0  # rads
        self.curvature = 0.0  # 1 / m
        self.drive_pedal = 0
        self.brake_pedal = 0
        self.gear_shift = 0
        self.lamp = 0
        self.light = 0

        # feedback data state, i.e "seen" variables.
        self.seen_vehicle_state_cmd = False
        self.seen_brake_rpt = False
        self.seen_gear_rpt = False
        self.seen_misc_rpt = False
        self.seen_steering_rpt = False
        self.seen_wheel_speed_rpt = False

        # state conversion
        self.autoware_to_dbw_gear = {AutowareGear.DRIVE: RaptorGear.DRIVE, AutowareGear.REVERSE: RaptorGear.REVERSE,
                                     AutowareGear.PARK: RaptorGear.PARK, AutowareGear.NEUTRAL: RaptorGear.NEUTRAL,
                                     AutowareGear.NONE: RaptorGear.NONE, AutowareGear.LOW: RaptorGear.LOW}
        self.autoware_to_dbw_drive_mode = {VehicleStatus.MODE_AUTO: True,
                                           VehicleStatus.MODE_MANUAL: False}
        self.dbw_to_autoware_gear = {RaptorGear.DRIVE: AutowareGear.DRIVE, RaptorGear.REVERSE: AutowareGear.REVERSE,
                                     RaptorGear.PARK: AutowareGear.PARK, RaptorGear.NEUTRAL: AutowareGear.NEUTRAL,
                                     RaptorGear.NONE: AutowareGear.NONE, RaptorGear.LOW: AutowareGear.LOW}
        self.dbw_to_autoware_drive_mode = {True: VehicleStatus.MODE_AUTO, False: VehicleStatus.MODE_MANUAL}
        self.dbw_turn_signal_to_autoware_lamp = {TurnSignal.NONE: 0,
                                                 TurnSignal.LEFT: VehicleStatus.LAMP_LEFT,
                                                 TurnSignal.RIGHT: VehicleStatus.LAMP_RIGHT,
                                                 TurnSignal.HAZARDS: VehicleStatus.LAMP_HAZARD, TurnSignal.SNA: 7}

        # odometry variables.
        self.travel_direction = 0.0  # stationary
        eps = 0.1
        if abs(self.front_axle_to_cog + self.rear_axle_to_cog - self.WHEELBASE) > eps:
            rospy.logerr("abs(front_axle_to_cog + rear_axle_to_cog - wheelbase) > {}. \n"
                         "Setting wheelbase = front_axle_to_cog + rear_axle_to_cog".format(eps))
            self.WHEELBASE = self.front_axle_to_cog + self.rear_axle_to_cog

        if self.publish_odom_tf:
            self.vehicle_odom_broadcaster = tf.TransformBroadcaster()

        # Initialize command values
        self.global_enable_cmd.ecu_build_number = self.ecu_build_num
        self.global_enable_cmd.enable_joystick_limits = False

        self.accel_cmd.control_type.value = self.speed_control_mode
        self.accel_cmd.ignore = False
        self.accel_cmd.accel_limit = self.acceleration_limit
        self.accel_cmd.accel_positive_jerk_limit = self.acceleration_positive_jerk_limit

        self.brake_cmd.control_type.value = self.speed_control_mode
        self.brake_cmd.decel_limit = self.deceleration_limit
        self.brake_cmd.decel_negative_jerk_limit = self.deceleration_negative_jerk_limit

        self.steer_cmd.control_type.value = self.steering_control_mode
        self.steer_cmd.ignore = False

        self.gear_cmd.cmd.gear = RaptorGear.NONE

        self.misc_cmd.door_request_right_rear.value = DoorRequest.NO_REQUEST
        self.misc_cmd.door_request_left_rear.value = DoorRequest.NO_REQUEST
        self.misc_cmd.door_request_lift_gate.value = DoorRequest.NO_REQUEST
        self.misc_cmd.rear_wiper_cmd.status = WiperRear.OFF
        self.misc_cmd.ignition_cmd.status = Ignition.NO_REQUEST
        self.misc_cmd.cmd.value = TurnSignal.NONE
        self.misc_cmd.low_beam_cmd.status = LowBeam.OFF
        self.misc_cmd.high_beam_cmd.status = HighBeam.OFF
        self.misc_cmd.front_wiper_cmd.status = WiperFront.OFF

        # Initialize feedback messages
        self.vehicle_feedback_status.header.frame_id = self.vehicle_status_frame_id  # same as base_frame
        self.vehicle_odom.header.frame_id = self.odom_frame_id
        self.vehicle_odom.child_frame_id = self.vehicle_feedback_status.header.frame_id
        self.vehicle_kinematic_state.header.frame_id = self.odom_frame_id
        self.vehicle_dbw_state_message.header.frame_id = self.vehicle_status_frame_id

        # subscribers (from Raptor DBW)
        subscriber_queue_size = 20
        self.dbw_enable_status = False
        self.accelerator_pedal_report_sub = rospy.Subscriber("accelerator_pedal_report",
                                                             AcceleratorPedalReport,
                                                             self.accelerator_pedal_report_callback,
                                                             queue_size=subscriber_queue_size)
        # self.brake_2_report_sub = rospy.Subscriber("brake_2_report",
        #                                            Brake2Report,
        #                                            self.brake_2_report_callback,
        #                                            queue_size=subscriber_queue_size)
        self.brake_report_sub = rospy.Subscriber("brake_report",
                                                 BrakeReport,
                                                 self.brake_report_callback,
                                                 queue_size=subscriber_queue_size)
        self.gear_report_sub = rospy.Subscriber("gear_report",
                                                GearReport,
                                                self.gear_report_callback,
                                                queue_size=subscriber_queue_size)
        # self.global_enable_cmd_sub = rospy.Subscriber("global_enable_cmd",
        #                                               GlobalEnableCmd,
        #                                               self.global_enable_callback,
        #                                               queue_size=subscriber_queue_size)
        self.steering_2_report_sub = rospy.Subscriber("steering_2_report",
                                                      Steering2Report,
                                                      self.steering_2_report_callback,
                                                      queue_size=subscriber_queue_size)
        self.steering_report_sub = rospy.Subscriber("steering_report", SteeringReport,
                                                    self.steering_report_callback, queue_size=subscriber_queue_size)
        # self.wheel_position_report_sub = rospy.Subscriber("wheel_position_report", WheelPositionReport,
        #                                                   self.wheel_position_report_callback,
        #                                                   queue_size=subscriber_queue_size)
        self.wheel_speed_report_sub = rospy.Subscriber("wheel_speed_report", WheelSpeedReport,
                                                       self.wheel_speed_report_callback,
                                                       queue_size=subscriber_queue_size)
        # self.driver_input_report_sub = rospy.Subscriber("driver_input_report", DriverInputReport,
        #                                                 self.driver_input_report_callback,
        #                                                 queue_size=subscriber_queue_size)
        # self.fault_actions_report_sub = rospy.Subscriber("fault_actions_report", FaultActionsReport,
        #                                                  self.fault_actions_report_callback,
        #                                                  queue_size=subscriber_queue_size)
        # self.low_voltage_system_report_sub = rospy.Subscriber("low_voltage_system_report",
        #                                                       LowVoltageSystemReport,
        #                                                       self.low_voltage_system_report_callback,
        #                                                       queue_size=subscriber_queue_size)
        self.misc_report_sub = rospy.Subscriber("misc_report", MiscReport,
                                                self.misc_report_callback,
                                                queue_size=subscriber_queue_size)
        # self.tire_pressure_report_sub = rospy.Subscriber("tire_pressure_report", TirePressureReport,
        #                                                  self.tire_pressure_report_callback,
        #                                                  queue_size=subscriber_queue_size)
        self.dbw_enable_sub = rospy.Subscriber("dbw_enabled", Bool, self.dbw_enable_callback,
                                               queue_size=subscriber_queue_size)

        # subscribers (from Autoware).
        self.ctrl_mode_sub = rospy.Subscriber("/ctrl_mode", String, self.ctrl_mode_status_callback,
                                              queue_size=subscriber_queue_size)  # If Autoware is in Autonomous mode
        self.vehicle_sub = rospy.Subscriber("/vehicle_cmd", VehicleCmd, self.vehicle_cmd_callback,
                                            queue_size=subscriber_queue_size)
        self.twist_cmd_sub = rospy.Subscriber("/twist_cmd", TwistStamped, self.twist_cmd_callback,
                                              queue_size=subscriber_queue_size)  # useful for desired yaw rate

        # publishers (to Raptor DBW)
        dbw_publisher_queue_size = 1
        self.accel_cmd_pub = rospy.Publisher("accelerator_pedal_cmd", AcceleratorPedalCmd,
                                             queue_size=dbw_publisher_queue_size)
        self.brake_cmd_pub = rospy.Publisher("brake_cmd", BrakePedalCmd, queue_size=dbw_publisher_queue_size)
        self.misc_cmd_pub = rospy.Publisher("misc_cmd", MiscCmd, queue_size=dbw_publisher_queue_size)
        self.steering_cmd_pub = rospy.Publisher("steering_cmd", SteeringCmd, queue_size=dbw_publisher_queue_size)
        self.global_enable_cmd_pub = rospy.Publisher("global_enable_cmd", GlobalEnableCmd,
                                                     queue_size=dbw_publisher_queue_size)
        self.gear_cmd_pub = rospy.Publisher("gear_cmd", GearCmd, queue_size=dbw_publisher_queue_size)
        self.dbw_enable_cmd_pub = rospy.Publisher("enable", Empty, queue_size=dbw_publisher_queue_size)
        self.dbw_disable_cmd_pub = rospy.Publisher("disable", Empty, queue_size=dbw_publisher_queue_size)
        self.dbw_state_machine_pub = rospy.Publisher("dbw_state_machine", DbwStateMachineMsg,
                                                     queue_size=dbw_publisher_queue_size)

        # publishers (to Autoware)
        autoware_publisher_queue_size = 1
        self.vehicle_status_pub = rospy.Publisher("/vehicle_status", VehicleStatus,
                                                  queue_size=autoware_publisher_queue_size)
        self.vehicle_odom_pub = rospy.Publisher("odom_test", Odometry, queue_size=autoware_publisher_queue_size)
        self.vehicle_kinematic_state_pub = rospy.Publisher("vehicle_kinematic_state", VehicleKinematicState,
                                                           queue_size=autoware_publisher_queue_size)

        if 'periodic' in self.dbw_pub_mode.lower():
            # publish periodically at constant rate. Recommended
            self.dbw_timer = rospy.Timer(rospy.Duration(1/self.dbw_pub_rate), self.cmd_callback, reset=True)
        if 'periodic' in self.autoware_pub_mode.lower():
            # publish periodically at constant rate. Not recommended
            self.autoware_timer = rospy.Timer(rospy.Duration(1/self.autoware_pub_rate), self.vehicle_status_callback,
                                              reset=True)
            self.autoware_update_timer = rospy.Timer(rospy.Duration(1 / self.autoware_pub_rate),
                                                     self.send_vehicle_commands, reset=True)
        if 'periodic' in self.vehicle_odom_pub_mode.lower():
            # publish periodically at constant rate. Not recommended
            self.odom_timer = rospy.Timer(rospy.Duration(1/self.vehicle_odom_publish_rate), self.vehicle_odometry_publisher,
                                          reset=True)

    # ################################Raptor DBW Publishers Block Begins#######################################
    def cmd_callback(self, event):
        """
        This callback sends commands to Raptor DBW. Raptor DBW expects messages to be sent continuously
                even if no changes are made, while Autoware only sends/expects when changes are made.

        Could be triggered by:
            1. On change: i.e called from /vehicle_cmd callback when new messages arrive
            2. Periodically [Recommended]: through timer callback. Wasteful as the messages won't be updated and could be confusing.
            3. Periodically + on change [good alternative]: a timer callback runs very fast but the header is checked
                                                                    to see if the messages are stale.
        :param event:
        :return:
        """
        # Note: steer_cmd actually takes radians although the documentation claims degrees
        # Note: steer_report uses radians instead of degree
        # Increment rolling counter
        self.counter += 1

        # counter reset/overflow
        if self.counter > 15:
            self.counter = 0

        # Set rolling counters
        self.accel_cmd.rolling_counter = self.counter
        self.brake_cmd.rolling_counter = self.counter
        self.steer_cmd.rolling_counter = self.counter
        self.gear_cmd.rolling_counter = self.counter
        self.misc_cmd.rolling_counter = self.counter
        self.global_enable_cmd.rolling_counter = self.counter

        # dbw_enabled = self.dbw_state()
        dbw_enabled = self.dbw_state_machine.get_state() != DbwState.DISABLED

        # Set enables based on current DBW mode
        if dbw_enabled:
            '''
            could enable actuators separately, e.g cruise control does not control steering
            '''
            self.accel_cmd.enable = self.enable_speed_cmd
            self.brake_cmd.enable = self.enable_speed_cmd
            self.steer_cmd.enable = self.enable_steer_cmd  # could enable separately
            self.gear_cmd.enable = self.enable_gear_cmd
            self.misc_cmd.block_standard_cruise_buttons = True
            self.misc_cmd.block_adaptive_cruise_buttons = True
            self.misc_cmd.block_turn_signal_stalk = True
            self.global_enable_cmd.global_enable = True

        else:
            self.accel_cmd.enable = False
            self.brake_cmd.enable = False
            self.steer_cmd.enable = False
            self.gear_cmd.enable = False
            self.misc_cmd.block_standard_cruise_buttons = False
            self.misc_cmd.block_adaptive_cruise_buttons = False
            self.misc_cmd.block_turn_signal_stalk = False
            self.global_enable_cmd.global_enable = False

        # Publish commands to NE Raptor DBW
        self.accel_cmd_pub.publish(self.accel_cmd)
        self.brake_cmd_pub.publish(self.brake_cmd)
        self.steering_cmd_pub.publish(self.steer_cmd)
        self.gear_cmd_pub.publish(self.gear_cmd)
        self.misc_cmd_pub.publish(self.misc_cmd)
        self.global_enable_cmd_pub.publish(self.global_enable_cmd)

        # Set state flags. This changes the DBW state machine from ENABLE_REQUESTED to ENABLE_SENT
        self.dbw_state_machine.control_cmd_sent()
        self.dbw_state_machine.state_cmd_sent()
        self.publish_dbw_state_machine_cmd()

    def send_state_command(self):
        # send gear, Turnsignal/blinker, and parking brake commands to DBW. lines 193-262
        gear_ret = True
        turn_signal_ret = True

        # Set gear values.
        desired_gear = self.autonomy_data["gear_cmd"]
        # gear_cmd = self.autoware_to_dbw_gear.get(desired_gear, RaptorGear.NONE)
        gear_cmd = self.autoware_to_dbw_gear.get(desired_gear)
        if gear_cmd is None:
            # default case, i.e error/invalid signal
            gear_cmd = RaptorGear.NONE
            rospy.loginfo("Received command for invalid gear state.")
            gear_ret = False
        self.gear_cmd.cmd.gear = gear_cmd

        # Set TurnSignal/Blinker value.
        desired_turn_signal = self.autonomy_data["turn_signal_cmd"]
        if desired_turn_signal.l == 0 and desired_turn_signal.r == 0:
            # no blinker on
            lamp_cmd = TurnSignal.NONE
        elif desired_turn_signal.l == 1 and desired_turn_signal.r == 0:
            # left blinker
            lamp_cmd = TurnSignal.LEFT
        elif desired_turn_signal.l == 0 and desired_turn_signal.r == 1:
            # right blinker
            lamp_cmd = TurnSignal.RIGHT
        elif desired_turn_signal.l == 1 and desired_turn_signal.r == 1:
            # hazard lights
            lamp_cmd = TurnSignal.HAZARDS
        else:
            # invalid command
            lamp_cmd = TurnSignal.SNA
            rospy.loginfo("Received command for invalid turn signal state.")
            turn_signal_ret = False
        self.misc_cmd.cmd.value = lamp_cmd

        # Set brake command from parking brake status. (Not implemented since Autoware.AI does not send parking break)

        self.seen_vehicle_state_cmd = True
        return gear_ret, turn_signal_ret

    def send_control_command(self):
        # Sends control commands from autoware to dbw. See lines 264-459
        # Since Autoware.AI has only one complete ControlCommand (i.e. vehicle_cmd), use this or twist.
        # No need for high_level, raw, or VehicleControlCommand
        ret = True
        velocity_checked = 0.0
        angle_checked = 0.0

        # Set limits
        self.steer_cmd.angle_velocity = math.degrees(self.max_steering_wheel_angle)  # in degrees

        desired_long_accel_mps2 = self.autonomy_data["accel_decel_limits"]

        if desired_long_accel_mps2 > 0.0 and desired_long_accel_mps2 < self.acceleration_limit:
            self.accel_cmd.accel_limit = desired_long_accel_mps2
        else:
            self.accel_cmd.accel_limit = self.acceleration_limit

        if desired_long_accel_mps2 < 0.0 and desired_long_accel_mps2 > (-1.0 * self.deceleration_limit):
            self.brake_cmd.decel_limit = fabs(desired_long_accel_mps2)
        else:
            self.brake_cmd.decel_limit = self.deceleration_limit

        if fabs(desired_long_accel_mps2) > self.acceleration_limit or fabs(desired_long_accel_mps2) > self.deceleration_limit:
            rospy.loginfo("Desired acceleration/deceleration of {} is greater than the set limit. Saturating...".format(desired_long_accel_mps2))

        # Check for invalid changes in direction.
        desired_gear = self.autonomy_data["gear_cmd"]
        desired_velocity_mps = self.autonomy_data["speed_cmd"]  # in m/s.
        if ((desired_gear == AutowareGear.DRIVE) and (desired_velocity_mps < 0.0)) \
                or ((desired_gear == AutowareGear.REVERSE) and (desired_velocity_mps > 0.0)):
            velocity_checked = 0.0
            rospy.loginfo("Got invalid speed request value: speed direction does not match current gear.")
            ret = False
        else:
            velocity_checked = fabs(desired_velocity_mps)

        # Listen to emergency command to stop instantly
        emergency_cmd = self.autonomy_data["emergency_cmd"]  # True: stop instantly
        velocity_checked *= int(not emergency_cmd)

        # Steering -> tire angle conversion is linear except for extreme angles
        desired_steer_cmd = self.autonomy_data["steering_angle_cmd"]  # rads
        desired_front_wheel_angle_rad = desired_steer_cmd  # rads.
        # to convert from tire (steer) angle to steering wheel angle
        angle_checked = self.steering_angle_to_steering_wheel_angle(desired_front_wheel_angle_rad)  # in rads
        # angle_checked_deg = math.degrees(angle_checked)  # angle_checked_rads / self.deg2rad  # in degrees

        # Limit steering angle to valid range
        if angle_checked > self.max_steering_wheel_angle:
            angle_checked = self.max_steering_wheel_angle
            rospy.logerr("Got invalid steering angle value: request exceeds max angle.")
            ret = False

        if angle_checked < (-1.0 * self.max_steering_wheel_angle):
            angle_checked = -1.0 * self.max_steering_wheel_angle
            rospy.logerr("Got invalid steering angle value: request exceeds max angle.")
            ret = False

        # Set commands
        desired_curvature = self.steering_angle_to_curvature(desired_steer_cmd)  # 1/m.
        desired_steering_wheel_cmd = angle_checked  # rads
        desired_yaw_rate = self.autonomy_data["twist_cmd"].twist.angular.z  # rads/s
        # convert yaw rate from twist_cmd to steering wheel angle
        steering_wheel_cmd_from_yaw_rate = self.yaw_rate_to_steering_wheel_angle(desired_yaw_rate, velocity_checked)
        self.accel_cmd.speed_cmd = velocity_checked
        # todo: (/test/raptor) test raptor by sending one steer control mode and sending the other value
        self.steer_cmd.vehicle_curvature_cmd = desired_curvature
        self.steer_cmd.angle_cmd = angle_checked

        return ret

    def publish_dbw_state_machine_cmd(self):
        self.vehicle_dbw_state_message.header.stamp = rospy.Time.now()

        self.vehicle_dbw_state_message.first_control_cmd_sent = float(self.dbw_state_machine.first_control_cmd_sent)
        self.vehicle_dbw_state_message.first_state_cmd_sent = float(self.dbw_state_machine.first_state_cmd_sent)

        self.vehicle_dbw_state_message.disabled_feedback_count = int(self.dbw_state_machine.disabled_feedback_count)
        self.vehicle_dbw_state_message.disabled_feedback_thresh = int(self.dbw_state_machine.DISABLED_FEEDBACK_THRESH)

        dbw_state = DbwStateMsg()
        dbw_state.state = self.dbw_state_machine.get_state()
        self.vehicle_dbw_state_message.state = dbw_state

        self.dbw_state_machine_pub.publish(self.vehicle_dbw_state_message)

    def handle_mode_change_request(self):
        # this mode takes the dbw_machine from DISABLED to ENABLE_REQUESTED if mode = True
        # todo: move to State Machine
        # get mode change request either from Autoware /vehicle_cmd, joystick or VehicleStateMachine. For now (1)
        # See lines 461-478
        ret = True

        # get mode from vehicle_cmd or user request, e.g joystick
        autoware_mode = self.autonomy_data["mode_cmd"]  # todo: find out how to publish Autoware mode via joystick. Check runtime manager
        mode_cmd = self.autoware_to_dbw_drive_mode.get(autoware_mode)
        if not mode_cmd:
            # if manual mode is active/requested
            self.dbw_state_machine.user_request(False)  # could move to state machine node. todo: test
            self.dbw_disable_cmd_pub.publish(self.empty_cmd)
        elif mode_cmd:
            # if autonomous mode is active/requested
            self.dbw_state_machine.user_request(True)  # could move to state machine node. todo: test
            self.dbw_enable_cmd_pub.publish(self.empty_cmd)
        else:
            rospy.logerr("Got invalid autonomy mode request value.")
            ret = False

        self.publish_dbw_state_machine_cmd()
        return ret

    def send_headlights_command(self):
        # see lines 480-505
        # Autoware.AI does not publish headlight data
        self.misc_cmd.low_beam_cmd.status = LowBeam.OFF
        self.misc_cmd.high_beam_cmd.status = HighBeam.OFF

    def send_horn_command(self):
        # see lines 507-511
        # Autoware.AI does not publish horn command
        horn_cmd = False
        self.misc_cmd.horn_cmd = horn_cmd

    def send_wipers_command(self):
        # see lines 513-542
        # Autoware.AI does not publish wipers command
        self.misc_cmd.front_wiper_cmd.status = WiperFront.OFF
        self.misc_cmd.rear_wiper_cmd.status = WiperRear.OFF

    def send_vehicle_commands(self, event=None):
        """
        This callback updates control commands sent from Autoware. Raptor DBW expects messages to be sent continuously
        even if no changes are made, while Autoware only sends/expects when changes are made.

        Could be triggered by:
            1. On change [preferred/default]: i.e called from /vehicle_cmd callback when new messages arrive
            2. Periodically: through timer callback. Wasteful as the messages won't be updated and could be confusing.
            3. Periodically + on change [good alternative]: a timer callback runs very fast but the header is checked
                                                            to see if the messages are stale.
        :param event:
        :return:
        """
        gear_return, turn_signal_return = self.send_state_command()
        control_return = self.send_control_command()
        mode_change_request_return = self.handle_mode_change_request()
        # self.send_headlights_command()  # unimplemented
        # self.send_horn_command()  # unimplemented
        # self.send_wipers_command()  # unimplemented
        return

    def dbw_state(self):
        # todo: (/test/raptor)
        # Check if DBW is enabled
        report_state = False  # check dbw reports for status
        dbw_enabled = False  # check enable message for status
        # From misc_report ("misc_report")
        if self.by_wire_ready and self.drive_by_wire_enabled:
            report_state = True
        # From enable message ("dbw_enabled")
        if self.dbw_enable_status:
            dbw_enabled = True
        return report_state and dbw_enabled

    # ################################Raptor DBW Publishers Block Ends#######################################

    # ################################Raptor DBW Subscribers Block Begins####################################
    def dbw_enable_callback(self, msg):
        # todo: push to state machine
        self.dbw_enable_status = msg.data

    def brake_report_callback(self, msg):
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)

        # Brake pedal (0 to 100%)
        pedal_input = float(msg.pedal_position)
        pedal_output = float(msg.pedal_output)

        # Status (Bools)
        enabled = msg.enabled
        driver_activity = msg.driver_activity

        fault_brake_system = msg.fault_brake_system  # bool

        # Faults (Bools)
        fault_ch1 = msg.fault_ch1
        fault_ch2 = msg.fault_ch2

        rolling_counter = int(msg.rolling_counter)
        brake_torque_actual = float(msg.brake_torque_actual)  # 0 to 100%

        # Anti-lock Brakes and  Stability Control
        intervention_active = msg.intervention_active
        intervention_ready = msg.intervention_ready

        # Parking Brake. todo: implement parking brake switch case and set to vehicle state (see lines 544-563)
        parking_brake = msg.parking_brake.status  # todo: could check if this is enabled and pass to state machine and watchdog to prevent moving with parking brake on.

        control_type = msg.control_type
        self.brake_pedal = self.vehicle_feedback_status.brakepedal = int(pedal_output)
        self.vehicle_kinematic_state.brake_pedal = int(pedal_output)
        self.vehicle_kinematic_state.parking_brake = parking_brake == 1  # todo: implement parking brake to bool
        # rospy.loginfo("Brake pedal: {} \n".format(self.brake_pedal))
        self.seen_brake_rpt = True

    def gear_report_callback(self, msg):
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)

        # Current gear enumeration
        current_gear = int(msg.state.gear)  # int(msg.state.gear)

        # Gear reject flag
        reject = msg.reject

        # Status (Bools)
        enabled = msg.enabled
        driver_activity = msg.driver_activity

        # Faults
        gear_select_system_fault = msg.gear_select_system_fault  # bool

        # send gear shift to vehicle state
        self.gear_shift = current_gear
        # gear_status = self.dbw_to_autoware_gear.get(self.gear_shift, AutowareGear.NONE)
        gear_status = self.dbw_to_autoware_gear.get(self.gear_shift)
        if gear_status is None:
            # default case, i.e error/invalid signal
            gear_status = AutowareGear.NONE
            rospy.loginfo("Received invalid gear value from NE Raptor DBW.")
        self.vehicle_feedback_status.current_gear.gear = gear_status
        self.vehicle_kinematic_state.gear_state = gear_status
        # if self.autoware_pub_mode.lower() == 'change':
        #     # todo: test but I suspect should be commented out so as not to mess with the timestamps
        #     self.vehicle_feedback_status.header.stamp = msg_time
        #     self.vehicle_status_pub.publish(self.vehicle_feedback_status)
        # rospy.loginfo("Raptor Gear: {}, Autoware Gear: {} \n".format(current_gear, self.gear_shift))
        self.seen_gear_rpt = True

    def misc_report_callback(self, msg):
        # see lines 594-674
        # Todo: note that calculating odometry here slows down the callback. Might need to implement in a separate thread
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)
        fuel_level = float(msg.drive_by_wire_enabled)  # %. Todo: add to state report for state machine
        self.drive_by_wire_enabled = msg.drive_by_wire_enabled  # bool.
        vehicle_speed = float(msg.vehicle_speed)  # m/s. Note that documentation says km/h
        # todo: (test/raptor)see if it takes negative and send negative if in reverse
        # todo: convert to m/s and compare to localization of results from wheel speed or LIDAR
        vehicle_speed_kmph = vehicle_speed * self.MS_TO_KMPH * self.travel_direction  # km/h. Todo: compare to speedometer while driving
        software_build_number = int(msg.software_build_number)
        general_actuator_fault = msg.general_actuator_fault  # bool
        self.by_wire_ready = msg.by_wire_ready  # bool
        self.general_driver_activity = msg.general_driver_activity  # bool
        comms_fault = msg.comms_fault
        ambient_temp = float(msg.ambient_temp)  # degrees-C

        # Update state machine/state report: fuel, drive by wire enabled, by wire ready, general driver activity
        mode = self.dbw_to_autoware_drive_mode.get(self.drive_by_wire_enabled)  # todo: remove. todo: send mode to state machine/report
        #  Update vehicle status (autoware drive mode). Autonomous or Manual. Todo: get from elsewhere
        # drive and steering modes can be different if only one is being controlled autonomously
        self.drivemode = self.vehicle_feedback_status.drivemode = mode
        self.steeringmode = self.vehicle_feedback_status.drivemode = mode

        dbw_feedback = self.by_wire_ready and not self.general_driver_activity
        self.dbw_state_machine.dbw_feedback(dbw_feedback)
        self.publish_dbw_state_machine_cmd()

        '''
           * Input velocity is (assumed to be) measured at the rear axle, but we're
           * producing a velocity at the center of gravity.
           * Lateral velocity increases linearly from 0 at the rear axle to the maximum
           * at the front axle, where it is tan(delta)*v_lon.
        '''
        prev_speed = 0.0

        delta = self.vehicle_feedback_status.angle
        # delta = self.vehicle_kinematic_state.front_wheel_angle
        if self.seen_misc_rpt and self.seen_wheel_speed_rpt:
            prev_speed_kmph = self.vehicle_feedback_status.speed
            # prev_speed_kmph = self.vehicle_kinematic_state.longitudinal_velocity_kmph
            prev_speed = prev_speed_kmph * self.KMPH_TO_MS
        self.vehicle_feedback_status.speed = vehicle_speed_kmph  # update vehicle speed status.
        self.vehicle_kinematic_state.longitudinal_velocity_kmph = vehicle_speed_kmph
        self.vehicle_odom.twist.twist.linear.x = vehicle_speed
        self.vehicle_kinematic_state.longitudinal_velocity = vehicle_speed
        lateral_velocity = (self.rear_axle_to_cog / self.WHEELBASE) * vehicle_speed * tan(delta)
        self.vehicle_odom.twist.twist.linear.y = lateral_velocity
        self.vehicle_kinematic_state.lateral_velocity = lateral_velocity

        # need to receive >1 message to calculate dT
        if not self.seen_misc_rpt:
            self.seen_misc_rpt = True
            self.vehicle_odom.header.stamp = msg_time
            self.vehicle_feedback_status.header.stamp = msg_time
            self.vehicle_kinematic_state.header.stamp = msg_time
            # Position = (0, 0) at time = 0
            # self.vehicle_odom.pose.pose.position.x = 0.0
            # self.vehicle_odom.pose.pose.position.y = 0.0
            # self.vehicle_odom.pose.pose.orientation = tf.transformations.quaternion_from_euler(0, 0, 0.0)

            # initialize Odometry position and quaternion
            initial_pose = Pose(Point(0.0, 0.0, 0.), Quaternion(*tf.transformations.quaternion_from_euler(0, 0, 0.0)))
            self.vehicle_odom.pose.pose = initial_pose
            self.vehicle_kinematic_state.location.pose = initial_pose
            if self.publish_odom_tf:
                # todo: move to separate timer callback
                translation = self.vehicle_odom.pose.pose.position
                rotation = self.vehicle_odom.pose.pose.orientation
                self.vehicle_odom_broadcaster.sendTransform(
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                        msg_time,
                        self.vehicle_odom.child_frame_id,
                        self.vehicle_odom.header.frame_id
                )
            return

        # calculate dT (seconds).
        #  ####method 1
        # dT = (msg_time - self.vehicle_kinematic_state.header.stamp).to_sec()
        dT = (msg_time - self.vehicle_odom.header.stamp).to_sec()
        # dT = (msg_time - self.vehicle_feedback_status.header.stamp).to_sec()

        # ####method 2
        dT = float(msg_time.secs - self.vehicle_odom.header.stamp.secs)
        # dT = float(msg_time.secs - self.vehicle_kinematic_state.header.stamp.secs)
        # dT = float(msg_time.secs - self.vehicle_feedback_status.header.stamp.secs)

        # convert nanoseconds to seconds
        # dT += float(msg_time.nsecs - self.vehicle_odom.header.stamp.nsecs) / 1000000000.0
        dT += float(msg_time.nsecs - self.vehicle_odom.header.stamp.nsecs) * 1e-9

        if dT < 0.0:
            # this means odom time is ahead of this (misc_report) time.
            # could alse be triggered on ROS time reset, e.g ROSBAG loop. Todo: handle exceptions
            rospy.logerr("Received inconsistent timestamps.")
            return

        # update Odom/vehicle status time
        self.vehicle_odom.header.stamp = msg_time
        self.vehicle_feedback_status.header.stamp = msg_time
        self.vehicle_kinematic_state.header.stamp = msg_time

        if self.seen_steering_rpt and self.seen_wheel_speed_rpt:
            # todo: handle division by zero error or check if dT=0.0
            self.vehicle_kinematic_state.longitudinal_acceleration = (vehicle_speed - prev_speed) / dT  # m/s^2
            self.vehicle_kinematic_state.time_change = rospy.Duration(dT)

            beta = atan2(self.rear_axle_to_cog * tan(delta), self.WHEELBASE)
            heading_rate = cos(beta) * tan(delta) / self.WHEELBASE
            self.vehicle_kinematic_state.heading_rate = heading_rate
            self.vehicle_odom.twist.twist.angular.z = heading_rate

            # update position (x, y), yaw
            self.kinematic_bicycle_model(dT)

            # publish on change, i.e new message like this. Todo: catch ROSException error due to publisher being shutdown on Ctrl-C.
            if self.autoware_pub_mode.lower() == 'change':
                self.vehicle_status_pub.publish(self.vehicle_feedback_status)
            if self.vehicle_odom_pub_mode.lower() == 'change':
                self.vehicle_odom_pub.publish(self.vehicle_odom)
            if self.vehicle_odom_pub_mode.lower() == 'change':
                self.vehicle_kinematic_state_pub.publish(self.vehicle_kinematic_state)

            if self.publish_odom_tf:
                # todo: move to separate timer callback and compare times
                translation = self.vehicle_odom.pose.pose.position
                rotation = self.vehicle_odom.pose.pose.orientation
                self.vehicle_odom_broadcaster.sendTransform(
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                        msg_time,
                        self.vehicle_odom.child_frame_id,
                        self.vehicle_odom.header.frame_id
                )

        self.speed = self.vehicle_feedback_status.speed = vehicle_speed_kmph  # [km/h]
        # rospy.loginfo("Speed: {} \n".format(self.speed))

    def driver_input_report_callback(self, msg):
        # todo: verify that this is the same as OtherActuators report by commanding turnsignal and checking change here
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)

        # Turn Signal enumeration
        turn_signal = msg.turn_signal.value

        # High beams
        high_beam_headlights = msg.high_beam_headlights.status

        # Front Windshield Wipers enumeration
        wiper = msg.wiper

        # Buttons (Bools)
        cruise_resume_button = msg.cruise_resume_button
        cruise_cancel_button = msg.cruise_cancel_button
        cruise_accel_button = msg.cruise_accel_button
        cruise_decel_button = msg.cruise_decel_button
        cruise_on_off_button = msg.cruise_on_off_button
        adaptive_cruise_on_off_button = msg.adaptive_cruise_on_off_button
        adaptive_cruise_increase_distance_button = msg.adaptive_cruise_increase_distance_button
        adaptive_cruise_decrease_distance_button = msg.adaptive_cruise_decrease_distance_button

        # Door Status (Bools)
        driver_door_ajar = msg.driver_door_ajar
        passenger_door_ajar = msg.passenger_door_ajar
        rear_left_door_ajar = msg.rear_left_door_ajar
        rear_right_door_ajar = msg.rear_right_door_ajar
        liftgate_ajar = msg.liftgate_ajar
        any_seatbelt_unbuckled = msg.any_seatbelt_unbuckled
        airbag_deployed = msg.airbag_deployed
        door_or_hood_ajar = msg.door_or_hood_ajar

        # blinkers/turnSignals
        self.lamp = self.vehicle_feedback_status.lamp = self.dbw_turn_signal_to_autoware_lamp[turn_signal]
        self.light = self.vehicle_feedback_status.light = high_beam_headlights  # headlight

        # rospy.loginfo("Light: {}, Lamp: {} \n".format(self.light, self.lamp))

    def other_actuators_report_callback(self, msg):
        # ROS1 DBW does not publish this report
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)
        turn_signal_state = msg.turn_signal_state
        right_rear_door_state = msg.right_rear_door_state
        high_beam_state = msg.high_beam_state
        front_wiper_state = msg.front_wiper_state
        rear_wiper_state = msg.rear_wiper_state
        ignition_state = msg.ignition_state
        left_rear_door_state = msg.left_rear_door_state
        liftgate_door_state = msg.liftgate_door_state
        horn_state = msg.horn_state
        low_beam_state = msg.low_beam_state

        rolling_counter = int(msg.rolling_counter)

    def steering_2_report_callback(self, msg):
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)
        vehicle_curvature_actual = float(msg.vehicle_curvature_actual)  # units are 1/m
        steering_angle = self.curvature_to_steering_angle(vehicle_curvature_actual)  # todo: test and compare in my state message
        max_torque_driver = float(msg.max_torque_driver)  # %-Torque
        max_torque_motor = float(msg.max_torque_motor)  # %-Torque

        self.curvature = vehicle_curvature_actual
        self.vehicle_kinematic_state.curvature = vehicle_curvature_actual

        # if self.steering_control_mode == ActuatorControlMode.closed_loop_vehicle:
        #     self.steering_angle = steering_angle

        # rospy.loginfo("Vehicle curvature: {} , steering_angle in degrees: {} \n".format(vehicle_curvature_actual,
        #                                                                                 math.degrees(steering_angle)))

    def steering_report_callback(self, msg):
        # Note: although DBW documents degrees, values are actually in radians
        # Note NE Raptor interface refers to steering angle as f_wheel_angle and steering wheel angle as steer angle
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)
        # Steering Wheel Angle
        steering_wheel_angle = float(msg.steering_wheel_angle)  # rads (but documentation says degrees)
        steering_wheel_angle_deg = math.degrees(steering_wheel_angle)
        steering_angle = self.steering_wheel_angle_to_steering_angle(steering_wheel_angle)  # f_wheel_angle_rad
        # todo: compare output to the last 2 outputs of "joints_states", i.e steer_fl, steer_fr
        steering_wheel_angle_cmd = float(msg.steering_wheel_angle_cmd)  # rads (degrees in documentation)
        steering_wheel_torque = float(msg.steering_wheel_angle)  # 0-100%

        # Status (Bools)
        enabled = msg.enabled
        driver_activity = msg.driver_activity

        fault_steering_system = msg.fault_steering_system  # bool
        overheat_prevention_mode = msg.overheat_prevention_mode  # bool
        rolling_counter = int(msg.rolling_counter)
        control_type = msg.control_type
        steering_overheat_warning = msg.steering_overheat_warning  # bool. Todo: pass to statemachine and watchdog. Should disable autonomous mode if True

        self.vehicle_feedback_status.angle = steering_angle
        self.vehicle_kinematic_state.front_wheel_angle = steering_angle
        self.vehicle_kinematic_state.front_wheel_angle_deg = math.degrees(steering_angle)
        self.vehicle_kinematic_state.rear_wheel_angle = 0.0
        self.vehicle_kinematic_state.steering_wheel_angle = steering_wheel_angle
        self.vehicle_kinematic_state.steering_wheel_angle_deg = steering_wheel_angle_deg

        self.seen_steering_rpt = True
        # Note: steering alone shouldn't cause time update cause Odom=0 if velocity=0 regardless of steering angle
        #self.vehicle_feedback_status.header.stamp = msg_time
        #self.vehicle_odom.header.stamp = msg_time
        #self.vehicle_kinematic_state.header.stamp = msg_time

    def wheel_speed_report_callback(self, msg):
        # todo: use this and wheel_position_to_estimate speed and for kinematics. See AckermannRos implementation.
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)
        # Wheel speeds (rad/sec)
        front_left = float(msg.front_left)
        front_right = float(msg.front_right)
        rear_right = float(msg.rear_right)
        rear_left = float(msg.rear_left)

        self.vehicle_kinematic_state.front_right = front_right
        self.vehicle_kinematic_state.front_left = front_left
        self.vehicle_kinematic_state.rear_left = rear_left
        self.vehicle_kinematic_state.rear_right = rear_right

        travel_direction = self.get_travel_direction_from_wheel_speeds(front_left, front_right, rear_right,
                                                                       rear_left)
        self.travel_direction = travel_direction
        self.vehicle_kinematic_state.travel_direction = int(travel_direction)

        self.seen_wheel_speed_rpt = True

    def accelerator_pedal_report_callback(self, msg):
        msg_time, msg_frame_id = self.extract_header_from_msg(msg)

        # Accelerator pedal (0 to 100%)
        pedal_input = float(msg.pedal_input)  # input from human driver
        pedal_output = float(msg.pedal_output)  # actual pedal status

        # Status (Bools)
        enabled = msg.enabled
        ignore_driver = msg.ignore_driver
        driver_activity = msg.driver_activity

        fault_accel_pedal_system = msg.fault_accel_pedal_system  # bool

        # Faults (Bools)
        fault_ch1 = msg.fault_ch1
        fault_ch2 = msg.fault_ch2

        rolling_counter = int(msg.rolling_counter)
        torque_actual = float(msg.torque_actual)  # 0 to 100%

        control_type = msg.control_type
        self.drive_pedal = self.vehicle_feedback_status.drivepedal = int(pedal_output)
        self.vehicle_kinematic_state.accelerator_pedal = pedal_output
        # rospy.loginfo("Drive pedal: {} \n".format(self.drive_pedal))

    # ################################Raptor DBW Subscribers Block Ends#######################################

    # ################################Autoware Publishers Block Begins#######################################
    def vehicle_status_callback(self, event=None):
        """
        Values are mostly updated from dbw misc_report and steering_report.
        :param event:
        :return:
        """
        # update time in case periodic updates are needed. Note: values will be repeated
        if self.autoware_pub_mode.lower() == 'periodic':
            self.vehicle_feedback_status.header.stamp = rospy.Time.now()
            # self.vehicle_feedback_status.header.frame_id = 'base_link'
            self.vehicle_status_pub.publish(self.vehicle_feedback_status)

        elif 'change' in self.autoware_pub_mode.lower():
            # todo: test
            # since this callback only runs if 'periodic' or 'periodic_change', just check for change
            if not(event.current_real - self.vehicle_feedback_status.header.stamp > rospy.Duration(0.1)):
                self.vehicle_feedback_status.header.stamp = rospy.Time.now()
                # self.vehicle_feedback_status.header.frame_id = 'base_link'
                self.vehicle_status_pub.publish(self.vehicle_feedback_status)

    def vehicle_odometry_publisher(self, event=None):
        """
        Values are updated from dbw misc_report and steering_report.
        :param event:
        :return:
        """
        vehicle_odometry = self.vehicle_odom
        vehicle_odometry = Odometry()

        self.vehicle_odom_pub.publish(self.vehicle_odom)

        # update time in case periodic updates are needed. Note: values will be repeated
        if self.vehicle_odom_pub_mode.lower() == 'periodic':
            self.vehicle_odom.header.stamp = rospy.Time.now()
            # self.vehicle_odom.header.frame_id = 'base_link'
            self.vehicle_odom_pub.publish(self.vehicle_odom)

        elif 'change' in self.autoware_pub_mode.lower():
            # todo: test
            # since this callback only runs if 'periodic' or 'periodic_change', just check for change
            if not (event.current_real - self.vehicle_feedback_status.header.stamp > rospy.Duration(0.1)):
                self.vehicle_odom.header.stamp = rospy.Time.now()
                # self.vehicle_odom.header.frame_id = 'base_link'
                self.vehicle_odom_pub.publish(self.vehicle_odom)

    # ################################Autoware Publishers Block Ends#######################################

    # ################################Autoware Subscribers Block Begins#######################################
    def ctrl_mode_status_callback(self, msg):
        # string subscriber
        self.autoware_ctrl_mode = msg.data
        rospy.loginfo("Autoware ctrl mode: {}".format(msg.data))

    def vehicle_cmd_callback(self, msg):
        """
        Receives actuator (speed and steering angle) commands from a high level control node, e.g. Autoware and sets
        the appropriate commands for Autonole. Uses autonomous_data dictionary similar to joystick_data.
        Todo: see https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/ne_raptor_interface/src/ne_raptor_interface.cpp and https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/drivers/ssc_interface/src/ssc_interface.cpp for details.
        :param msg:
        :return:
        """

        '''
        Source: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/blob/master/src/control/pure_pursuit/src/pure_pursuit.cpp#L309
        To get steering angle from curvature see attached code snippet below. Reverse to get curvature from steering angle
          // Compute the steering angle by arctan(curvature * wheel_distance)
  // link: https://www.ri.cmu.edu/pub_files/2009/2/

        Also see lines 127-139 of ssc_interface.cpp in https://github.com/autowarefoundation/autoware/pull/1945/files#diff-6b1c23f44ef516e949f0d2ad40b71f0bba015afec58072eb95cd652f008be038
        '''
        # Todo: decide if i should filter bad values here or should be implemented per vehicle/dbw
        vehicle_cmd_msg_time = msg.header.stamp
        vehicle_cmd_msg_frame_id = msg.header.frame_id

        # car actuators for raw control. (Not supported)
        vehicle_cmd_msg_steer_cmd = msg.steer_cmd.steer  # steering wheel angle
        vehicle_cmd_msg_accel_cmd = msg.accel_cmd.accel
        vehicle_cmd_msg_brake_cmd = msg.brake_cmd.brake

        # gear.
        vehicle_cmd_msg_gear_cmd = int(msg.gear_cmd.gear)
        gear_cmd = vehicle_cmd_msg_gear_cmd

        # mode
        vehicle_cmd_msg_mode_cmd = int(msg.mode)
        mode_cmd = vehicle_cmd_msg_mode_cmd

        # emergency.
        emergency_cmd = bool(msg.emergency)  # True: stop instantly, False: keep going

        # turn signal
        lamp_cmd = msg.lamp_cmd

        # twist command
        vehicle_cmd_msg_twist_cmd = msg.twist_cmd  # could use the twist callback data here

        # control command.
        vehicle_cmd_msg_ctrl_cmd = msg.ctrl_cmd
        linear_velocity = vehicle_cmd_msg_ctrl_cmd.linear_velocity  # desired forward speed (m/s).
        linear_acceleration = vehicle_cmd_msg_ctrl_cmd.linear_acceleration  # desired acceleration (m/s^2)
        steering_angle = vehicle_cmd_msg_ctrl_cmd.steering_angle  # rads [-0.5, 0.5]

        self.autonomy_data = {'stamp': vehicle_cmd_msg_time,  # or vehicle_cmd_msg_time
                              'abs_speed': fabs(linear_velocity),  # m/s
                              'speed_cmd': float(linear_velocity),  # m/s,
                              'steering_angle_cmd': float(steering_angle), # rads
                              # 'steering_wheel_angle': steering_wheel_angle,  # rads
                              # 'steering_curvature': curvature,  # 1 / m
                              'gear_cmd': gear_cmd,
                              'turn_signal_cmd': lamp_cmd,
                              'mode_cmd': mode_cmd,
                              'emergency_cmd': emergency_cmd,
                              'accel_decel_limits': linear_acceleration,  # acceleration/deceleration limit in m/s^2,
                              'twist_cmd': vehicle_cmd_msg_twist_cmd,
                              'road_slope': 0.0}
        self.vehicle_cmd = msg

        if self.autoware_pub_mode.lower() == 'change':
            self.send_vehicle_commands()

    def twist_cmd_callback(self, msg):
        # to get desired yaw rate
        # rospy.loginfo("Twist cmd callback: {}".format(msg))
        twist_cmd_stamped_cmd_msg_stamp = msg.header.stamp
        twist_cmd_stamped_cmd_msg_frame_id = msg.header.frame_id

        linear_x = msg.twist.linear.x
        angular_z = msg.twist.angular.z
        steering_angle = self.yaw_rate_to_steering_angle(angular_z, linear_x)
        steering_wheel_angle = self.yaw_rate_to_steering_wheel_angle(angular_z, linear_x)

    # ################################Autoware Subscribers Block Ends#######################################

    def steering_wheel_angle_to_steering_angle(self, steering_wheel_angle):
        # steering_wheel_angle in rads
        # output is in radians
        # Steering -> tire angle conversion is linear except for extreme angles
        steering_angle = steering_wheel_angle / self.steer_to_tire_ratio
        return steering_angle

    def steering_wheel_angle_to_curvature(self, steering_wheel_angle):
        # output is in radians
        # method 1
        # steering_angle = self.steering_wheel_angle_to_steering_angle(steering_wheel_angle)
        # curvature = self.steering_angle_to_curvature(steering_angle)

        # method 2: shortcut
        curvature = tan(steering_wheel_angle / self.steer_to_tire_ratio) / self.WHEELBASE
        return curvature

    def steering_wheel_angle_to_yaw_rate(self, steering_wheel_angle, current_speed):
        yaw_rate = (tan(steering_wheel_angle * self.steer_to_tire_ratio) * current_speed) / self.WHEELBASE
        return yaw_rate

    def curvature_to_steering_angle(self, curvature):
        # output is in radians
        steering_angle = atan(curvature * self.WHEELBASE)
        return steering_angle

    def curvature_to_steering_wheel_angle(self, curvature):
        # output is in radians
        # method 1 (curv -> SA -> SWA)
        # steering_angle = self.curvature_to_steering_angle(curvature)
        # steering_wheel_angle = self.steering_angle_to_steering_wheel_angle(steering_angle)

        # method 2 (curv -> SWA)
        steering_wheel_angle = atan(curvature * self.WHEELBASE) * self.steer_to_tire_ratio
        return steering_wheel_angle

    def steering_angle_to_steering_wheel_angle(self, desired_steering_angle):
        """

        :param desired_steering_angle: the high level tire steer angle commanded by Autoware [rads]
        :return: steering_wheel_angle [rads]
        """
        # Tire -> steering wheel angle conversion is linear except for extreme angles
        steering_wheel_angle = desired_steering_angle * self.steer_to_tire_ratio
        return steering_wheel_angle

    def steering_angle_to_curvature(self, desired_steering_angle):
        desired_curvature = tan(desired_steering_angle) / self.WHEELBASE
        return desired_curvature

    def yaw_rate_to_steering_wheel_angle(self, desired_yaw_rate, current_speed):
        # source: file:///C:/Users/boluo/Downloads/ULC_UserGuide-RevA05.pdf#subsection.5.2
        try:
            steering_wheel_angle = atan((self.WHEELBASE * desired_yaw_rate) / current_speed) / self.steer_to_tire_ratio
        except ZeroDivisionError:
            rospy.logerr("Current speed is 0. Using 1e-10 instead")
            steering_wheel_angle = atan((self.WHEELBASE * desired_yaw_rate) / 1e-10) / self.steer_to_tire_ratio
        return steering_wheel_angle

    def yaw_rate_to_steering_angle(self, desired_yaw_rate, current_speed):
        if desired_yaw_rate == 0 or current_speed == 0:
            return 0

        radius = current_speed / desired_yaw_rate
        steering_angle = atan(self.WHEELBASE / radius)
        return steering_angle

    def extract_header_from_msg(self, msg):
        msg_time = msg.header.stamp
        msg_frame_id = msg.header.frame_id
        return msg_time, msg_frame_id

    def get_travel_direction_from_wheel_speeds(self, fl, fr, rr, rl):
        # todo: get travel direction from wheel speeds + gear + vehicle speed
        # todo: make more pythonic and efficient using all() or any()
        # todo: could use state machine to compare against gear and vehicle speed instead of doing that here and use wheel speed only here
        if (fl == 0.0) and (fr == 0.0) and (rr == 0.0) and (rl == 0.0):
            # car is not moving
            travel_direction = 0.0
        elif (fl >= 0.0) and (fr >= 0.0) and (rr >= 0.0) and (rl >= 0.0):
            # car is moving forward
            travel_direction = 1.0
        elif (fl <= 0.0) and (fr <= 0.0) and (rr <= 0.0) and (rl <= 0.0):
            # car is moving backward
            travel_direction = -1.0
        else:
            # Wheels are moving in different directions. This is a sign that somethings wrong.
            travel_direction = 0.0
            rospy.loginfo("Received inconsistent wheel speeds. Stopping now...")

        return travel_direction

    def kinematic_bicycle_model(self, dt):
        # to update x, y, heading and heading_rate
        # see: https://github.com/ros-controls/ros_controllers/blob/noetic-devel/ackermann_steering_controller/src/ackermann_steering_controller.cpp for example
        # see: https://github.com/ros-controls/ros_controllers/blob/noetic-devel/ackermann_steering_controller/src/odometry.cpp for example

        # convert to yaw
        # The below formula could probably be simplified if it would be derived directly for heading, e.g from localizer or IMU
        orientation = self.vehicle_odom.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)

        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)

        if yaw < 0:
            try:
                yaw += math.tau
            except AttributeError:
                # python2 does not have math.tau
                yaw += (math.pi * 2)

        '''
           * delta: tire angle (relative to car's main axis)
           * psi: heading/yaw
           * beta: direction of movement at point of reference (relative to car's main axis)
           * m_rear_axle_to_cog: distance of point of reference to rear axle
           * m_front_axle_to_cog: distance of point of reference to front axle
           * wheelbase: m_rear_axle_to_cog + m_front_axle_to_cog
           * x, y, v are at the point of reference
           * x' = v cos(psi + beta)
           * y' = v sin(psi + beta)
           * psi' = (cos(beta)tan(delta)) / wheelbase
           * v' = a
           * beta = arctan((m_rear_axle_to_cog*tan(delta))/wheelbase)
           */
        '''

        v0_lat = self.vehicle_odom.twist.twist.linear.y
        v0_lon = self.vehicle_odom.twist.twist.linear.y
        v0 = math.sqrt(v0_lat ** 2 + v0_lon ** 2)
        delta = self.vehicle_feedback_status.angle
        a = self.vehicle_kinematic_state.longitudinal_acceleration
        beta = atan2(self.rear_axle_to_cog * tan(delta), self.WHEELBASE)

        # This is the direction in which the POI is moving at the beginning of the
        # integration step. "Course" may not be super accurate, but it's to
        # emphasize that the POI doesn't travel in the heading direction.
        course = yaw + beta

        # How much the yaw changes per meter traveled (at the reference point)
        yaw_change = cos(beta) * tan(delta) / self.WHEELBASE

        # The yaw rate
        yaw_rate = yaw_change * v0

        # Threshold chosen to prevent numerical issues, i.e division by 0
        if abs(yaw_rate) < 1e-18:
            position_x_update = cos(course) * (v0 * dt + 0.5 * a * dt * dt)
            position_y_update = sin(course) * (v0 * dt + 0.5 * a * dt * dt)
            self.vehicle_odom.pose.pose.position.x += position_x_update
            self.vehicle_odom.pose.pose.position.y += position_y_update
            self.vehicle_kinematic_state.location.pose.position.x += position_x_update
            self.vehicle_kinematic_state.location.pose.position.y += position_y_update

        else:
            position_x_update = (v0 + a * dt) / yaw_rate * sin(course + yaw_rate * dt) - \
                                                     v0 / yaw_rate * sin(course) + \
                                                     a / (yaw_rate ** 2) * cos(course + yaw_rate * dt) - \
                                                     a / (yaw_rate ** 2) * cos(course)
            position_y_update = -(v0 + a * dt) / yaw_rate * cos(course + yaw_rate * dt) + \
                                                     v0 / yaw_rate * cos(course) + \
                                                     a / (yaw_rate ** 2) * sin(course + yaw_rate * dt) - \
                                                     a / (yaw_rate ** 2) * sin(course)
            self.vehicle_odom.pose.pose.position.x += position_x_update
            self.vehicle_odom.pose.pose.position.y += position_y_update
            self.vehicle_kinematic_state.location.pose.position.x += position_x_update
            self.vehicle_kinematic_state.location.pose.position.y += position_y_update

        yaw += cos(beta) * tan(delta) / self.WHEELBASE * (v0 * dt + 0.5 * a * dt * dt)
        orientation = tf.transformations.quaternion_from_euler(0., 0., yaw)
        orientaion_quarternion_object = Quaternion(*orientation)
        self.vehicle_odom.pose.pose.orientation = orientaion_quarternion_object
        self.vehicle_kinematic_state.location.pose.orientation = orientaion_quarternion_object

        # Rotations per second or rads per second
        self.vehicle_kinematic_state.heading_rate = yaw_rate
        self.vehicle_odom.twist.twist.angular.z = yaw_rate

    def shutdown(self):
        rospy.loginfo("Beginning clean shutdown routine...")
        # todo: send zero commands to the car
        # todo: update dbw state machine
        # todo: update state machine

        # todo: shutdown all threads and catch exceptions (AttributeError) for those that never started
        self.dbw_timer.shutdown()
        #self.autoware_timer.shutdown()
        #self.autoware_update_timer.shutdown()
        #self.odom_timer.shutdown()
        # todo: disable autonomous node and dbw enable
        # todo: update states like vehicle status
        rospy.loginfo("Shutting down...")


def main(args):
    # args will be a list of commands passed
    optional_nodename = 'ne_raptor_dbw_autoware_interface_node'
    rospy.init_node('{}'.format(optional_nodename))
    nodename = rospy.get_name()  # this gets the actual nodename whether the node is launched or run separately
    rospy.loginfo("{} node started.".format(nodename))  # just log that the node has started.
    ne_raptor_dbw_autoware_interface_instance = AutowareDbwInterfaceNode()

    # use rospy.on_shutdown() to perform clean shutdown tasks, e.g. saving a file, shutting down motors, etc.
    rospy.on_shutdown(ne_raptor_dbw_autoware_interface_instance.shutdown)
    # could also use atexit.register(sample_instance.shutdown) to avoid trusting rospy

    try:
        # run the main functions here
        # autonomous_speed_based_instance.run()
        rospy.spin()  # only necessary if not publishing (i.e. subscribing only)
    except (rospy.ROSInterruptException, rospy.ROSException, KeyboardInterrupt) as e:
        # this exception block most likely won't be called since there is a shutdown method in the class that will
        # override this and shutdown however is needed but is here just in case.
        rospy.loginfo('Encountered {}. Shutting down.'.format(e))

        # try:
        #     sys.exit(0)
        # except SystemExit:
        #     os._exit(0)


if __name__ == '__main__':
    myargv = rospy.myargv(argv=sys.argv)
    main(myargv)  # ROS compatible way to handle command line arguments, i.e main(sys.argv)
