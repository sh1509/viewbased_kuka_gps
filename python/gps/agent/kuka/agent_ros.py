""" This file defines an agent for the PR2 ROS environment. """
import copy
import time
import numpy as np

import rospy
import tf
import math

from gps.agent.agent import Agent
from gps_agent_pkg.srv import *
from gps.agent.agent_utils import generate_noise, setup
from gps.agent.config import AGENT_KUKA
from gps.agent.kuka.ros_utils import ServiceEmulator, msg_to_sample, \
        policy_to_msg, tf_policy_to_action_msg, tf_obs_msg_to_numpy
from gps.proto.gps_pb2 import TRIAL_ARM, AUXILIARY_ARM
from gps_agent_pkg.msg import TrialCommand, SampleResult, PositionCommand, \
        RelaxCommand, DataRequest, TfActionCommand, TfObsData
try:
    from gps.algorithm.policy.tf_policy import TfPolicy
except ImportError:  # user does not have tf installed.
    TfPolicy = None

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import String
from control_msgs.msg import JointTrajectoryControllerState
from geometry_msgs.msg import Pose



class AgentROS(Agent):
    """
    All communication between the algorithms and ROS is done through
    this class.
    """
    def __init__(self, hyperparams, init_node=True):
        """
        Initialize agent.
        Args:
            hyperparams: Dictionary of hyperparameters.
            init_node: Whether or not to initialize a new ROS node.
        """
        config = copy.deepcopy(AGENT_KUKA)
        config.update(hyperparams)
        Agent.__init__(self, config)
        if init_node:
            rospy.init_node('kuka_agent_ros_node')
        self._init_pubs_and_subs()
        self._seq_id = 0  # Used for setting seq in ROS commands.

        conditions = self._hyperparams['conditions']
        self.initial = 0

        self.x0 = []
        for field in ('x0', 'ee_points_tgt', 'reset_conditions'):
            self._hyperparams[field] = setup(self._hyperparams[field],
                                             conditions)
        self.x0 = self._hyperparams['x0']

        r = rospy.Rate(1)
        r.sleep()

        self.use_tf = False
        self.observations_stale = True
        self.joint_sub = rospy.Subscriber('/kuka_arm/arm_controller/state', \
        JointTrajectoryControllerState, self.joint_callback)
        self.current_joint = None
        # self.pub_eep = rospy.Publisher()
        # rospy.spin()

    def keyboard_teleop_client(self, a):
        rospy.wait_for_service('calculate_ik')
    # keyb = keys
        try:
            get_joint_values = rospy.ServiceProxy('calculate_ik', CalculateIK)
            resp1 = get_joint_values(a)
            return resp1.points
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
    
    def joint_callback(self, msg):
        # print(type(msg.actual.positions)
        self.current_joint = msg.actual.positions
        # print(type(self.current_joint))
    
    def _init_pubs_and_subs(self):
        self._trial_service = ServiceEmulator(
            self._hyperparams['trial_command_topic'], JointTrajectory,  #change the name of the msg type to be published
            self._hyperparams['sample_result_topic'], SampleResult    #here as well
        )
        self._reset_service = ServiceEmulator(
            self._hyperparams['reset_command_topic'], JointTrajectory,
            self._hyperparams['sample_result_topic'], SampleResult
        )
        # self._relax_service = ServiceEmulator(
        #     self._hyperparams['relax_command_topic'], RelaxCommand,
        #     self._hyperparams['sample_result_topic'], SampleResult
        # )
        # self._data_service = ServiceEmulator(
        #     self._hyperparams['data_request_topic'], DataRequest,
        #     self._hyperparams['sample_result_topic'], SampleResult
        # )

    def _get_next_seq_id(self):
        self._seq_id = (self._seq_id + 1) % (2 ** 32)
        return self._seq_id

    # def get_data(self, arm=TRIAL_ARM):
    #     """
    #     Request for the most recent value for data/sensor readings.
    #     Returns entire sample report (all available data) in sample.
    #     Args:
    #         arm: TRIAL_ARM or AUXILIARY_ARM.
    #     """
    #     request = DataRequest()
    #     request.id = self._get_next_seq_id()
    #     request.arm = arm
    #     request.stamp = rospy.get_rostime()
    #     result_msg = self._data_service.publish_and_wait(request)
    #     # TODO - Make IDs match, assert that they match elsewhere here.
    #     sample = msg_to_sample(result_msg, self)
    #     return sample

    # TODO - The following could be more general by being relax_actuator
    #        and reset_actuator.
    # def relax_arm(self, arm):
    #     """
    #     Relax one of the arms of the robot.
    #     Args:
    #         arm: Either TRIAL_ARM or AUXILIARY_ARM.
    #     """
    #     relax_command = RelaxCommand()
    #     relax_command.id = self._get_next_seq_id()
    #     relax_command.stamp = rospy.get_rostime()
    #     relax_command.arm = arm
    #     self._relax_service.publish_and_wait(relax_command)

    def reset_arm(self, arm, mode, data):
        """
        Issues a position command to an arm.
        Args:
            arm: Either TRIAL_ARM or AUXILIARY_ARM.
            mode: An integer code (defined in gps_pb2).
            data: An array of floats.
        """
        # write a publisher node here

        tp = tp = JointTrajectory()
        tp.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']

        tp.points = []
        # tp.points[0] = []
        trj_data = JointTrajectoryPoint()
        trj_data.positions = data
        trj_data.velocities = []
        trj_data.accelerations = []
        trj_data.effort = []
        trj_data.time_from_start = rospy.Duration(1)

        tp.points.append(trj_data)

        # reset_command = PositionCommand()
        # reset_command.mode = mode
        # reset_command.data = data
        # reset_command.pd_gains = self._hyperparams['pid_params']
        # reset_command.arm = arm
        timeout = self._hyperparams['trial_timeout']
        # reset_command.id = self._get_next_seq_id()
        self._reset_service.publish_and_wait(tp, timeout=timeout)
        #TODO: Maybe verify that you reset to the correct position.

    def reset(self, condition):
        """
        Reset the agent for a particular experiment condition.
        Args:
            condition: An index into hyperparams['reset_conditions'].
        """
        condition_data = self._hyperparams['reset_conditions'][condition]
        self.reset_arm(TRIAL_ARM, condition_data[TRIAL_ARM]['mode'],  #need this and publish it to the action server and wait for result
                       condition_data[TRIAL_ARM]['data'])
        # self.reset_arm(AUXILIARY_ARM, condition_data[AUXILIARY_ARM]['mode'],   # remove this guy
        #                condition_data[AUXILIARY_ARM]['data'])
        time.sleep(2.0)  # useful for the real robot, so it stops completely

    def sample(self, policy, condition, verbose=True, save=True, noisy=True):
        """
        Reset and execute a policy and collect a sample.
        Args:
            policy: A Policy object.
            condition: Which condition setup to run.
            verbose: Unused for this agent.
            save: Whether or not to store the trial into the samples.
            noisy: Whether or not to use noise during sampling.
        Returns:
            sample: A Sample object.
        """
        if TfPolicy is not None:  # user has tf installed.
            if isinstance(policy, TfPolicy):
                self._init_tf(policy.dU)

        self.reset(condition) 
        self.initial = 0    #
        # Generate noise.
        if noisy:
            noise = generate_noise(self.T, self.dU, self._hyperparams)
        else:
            noise = np.zeros((self.T, self.dU))
        
        '''
        1. Take all the value and add it to the trajectory points. 
        2. publish these values and wait. 
        3. line 207: change trial_command 
        4. line 210: check what is this function here. 
        '''
        # msg = policy_to_msg(policy, noise)
        # First calculate the current x_t: subscribe to current joint angle value and subscribe to the tf for the eepoint
        # Then calculate the U: same as below nothing to change about. And then publish this U on to the same. sleep for 1 0.5 second, 
        #  calculate the next similarly

        # Calculating X_t
        
        if self.initial == 0:
            listener = tf.TransformListener()

            while not rospy.is_shutdown():
                try:
                    trans, rot = listener.lookupTransform('/base_link', '/gripper_link', rospy.Time(0))
                    done = True
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    # print("Exception Occurred {}".format(e))
                    continue
                if done:
                    self.initial = 1
                    break
        X_t = []
        for i in self.current_joint:
            X_t.append(i)
        for i in trans:
            X_t.append(i)
        # tp = JointTrajectory()
        # tp.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        # tp.points = list()
        sample = []
        for t in range(self.T):
            tp = JointTrajectory()
            tp.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
            tp.points = list()
            X_t = np.array(X_t)
            obs_t = []
            obs_t = np.array(obs_t)
            U = np.zeros([self.T, self.dU])
            U[t, :] = policy.act(X_t, obs_t, t, noise[t, :])
            print(U[t,:])

            
            # X_t.append(list(self.current_joint, trans)) 
            # X_t.append(trans) 
            

            

            eef_pose = Pose()
            eef_pose.position.x = U[t, :][0]
            eef_pose.position.y = U[t, :][1]
            eef_pose.position.z = U[t, :][2]

            eef_pose.orientation.x = rot[0]
            eef_pose.orientation.y = rot[1]
            eef_pose.orientation.z = rot[2]
            eef_pose.orientation.w = rot[3]

            poses = []
            poses.append(eef_pose)
            points = self.keyboard_teleop_client(poses)   #This guy uses srv from gps_pkg_agent
            
            points[0].positions = list(points[0].positions)
            points[0].time_from_start = rospy.Duration(1)
            ind_not_update = []
            rel_key = 0
            for i in points[0].positions:
                if math.isnan(i):
                    # print("Replacing with: "),
                    # print(X_t[rel_key])
                    # done = False
                    i = X_t[rel_key]
                    # ind_not_update.append(rel_key)
                    # break
                    rel_key += 1
                else:
                    X_t[rel_key] = i
                    rel_key += 1 

            # tp.points.append(points)
            
            # tp.points[t].positions = X_t[0:6]


            # X_t = []
            # for i in range(6):
            #     if 
            act_i = 6
            for i in U[t, :]:
                X_t[act_i] = i
                act_i += 1
            # done = True
            # tp.points[0].positions = list(tp.points[0].positions)
            # rel_key = 0
            point = JointTrajectoryPoint()
            point.positions = list(X_t[0:6])
            point.velocities = []
            point.accelerations = []
            point.effort = []
            point.time_from_start = rospy.Duration((1))
            # tp.points[0][t].positions = list(X_t[0:6])
            tp.points.append(point)
            print("counter: "), 
            print(t+1)

            
 

            if self.use_tf is False:
                sample_msg = self._trial_service.publish_and_wait(
                    tp, timeout=self._hyperparams['trial_timeout']  #change this guy here.
                )
                sample.append(msg_to_sample(sample_msg, self))
        if save:
            self._samples[condition].append(sample)
        return sample
        # else:
        #     self._trial_service.publish(trial_command)
        #     sample_msg = self.run_trial_tf(policy, time_to_run=self._hyperparams['trial_timeout'])
        #     sample = msg_to_sample(sample_msg, self)
        #     if save:
        #         self._samples[condition].append(sample)
        #     return sample

    def run_trial_tf(self, policy, time_to_run=5):
        """ Run an async controller from a policy. The async controller receives observations from ROS subscribers
         and then uses them to publish actions."""
        should_stop = False
        consecutive_failures = 0
        start_time = time.time()
        while should_stop is False:
            if self.observations_stale is False:
                consecutive_failures = 0
                last_obs = tf_obs_msg_to_numpy(self._tf_subscriber_msg)
                action_msg = tf_policy_to_action_msg(self.dU,
                                                     self._get_new_action(policy, last_obs),
                                                     self.current_action_id)
                self._tf_publish(action_msg)
                self.observations_stale = True
                self.current_action_id += 1
            else:
                rospy.sleep(0.01)
                consecutive_failures += 1
                if time.time() - start_time > time_to_run and consecutive_failures > 5:
                    # we only stop when we have run for the trial time and are no longer receiving obs.
                    should_stop = True
        rospy.sleep(0.25)  # wait for finished trial to come in.
        result = self._trial_service._subscriber_msg
        return result  # the trial has completed. Here is its message.

    def _get_new_action(self, policy, obs):
        return policy.act(None, obs, None, None)

    def _tf_callback(self, message):
        self._tf_subscriber_msg = message
        self.observations_stale = False

    def _tf_publish(self, pub_msg):
        """ Publish a message without waiting for response. """
        self._pub.publish(pub_msg)

    def _init_tf(self, dU):
        self._tf_subscriber_msg = None
        self.observations_stale = True
        self.current_action_id = 1
        self.dU = dU
        if self.use_tf is False:  # init pub and sub if this init has not been called before.
            self._pub = rospy.Publisher('/gps_controller_sent_robot_action_tf', TfActionCommand)
            self._sub = rospy.Subscriber('/gps_obs_tf', TfObsData, self._tf_callback)
            r = rospy.Rate(0.5)  # wait for publisher/subscriber to kick on.
            r.sleep()
        self.use_tf = True
        self.observations_stale = True
