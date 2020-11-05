#! /usr/bin/env python
from geometry_msgs.msg import Pose
from grip_srv_msg_pkg.srv import CartesianPositionMessage, \
    CartesianPositionMessageResponse, GripperPositionMessageResponse, \
    GoalEEPositionMessage, GoalEEPosTrajMessage  # you import the service message python classes
from grip_srv_msg_pkg.srv import JointPositionMessage, \
    JointPositionMessageResponse  # you import the service message python classes
from grip_srv_msg_pkg.srv import GetPositionMessage, \
    GetPositionMessageResponse, GripperPositionMessage  # you import the service message python classes
from tf.transformations import quaternion_from_euler

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    all_equal = True
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False
            else:
                print('Success')

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

    return True


class MoveGroupPythonIntefaceTutorial(object):
    """MoveGroupPythonIntefaceTutorial"""

    def __init__(self):
        # getch = _Getch()
        super(MoveGroupPythonIntefaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:

        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('pose_command_service', anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()


        group_name = "manipulator"
        move_group = moveit_commander.MoveGroupCommander(group_name)

        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory,
                                                       queue_size=1)

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s", planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s", eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # We can get the robot's current state:
        group_names = robot.get_current_state()
        #print("============ Current State:", robot.get_current_state())


        # Misc variables
        self.box_name = 'box'
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names



        print('Adding box...')
        is_box_added = self.add_box()
        print('Added box ! ' if is_box_added else "Not Able to add box ")

    def rest(self):
        joint_position_list = [0, -pi / 4, pi/4, -pi / 2, -pi / 2, pi/2]
        self.go_to_joint_pose(joint_position_list)

    def go_to_joint_pose(self, joint_goal_list):
        # We can get the joint values from the group and adjust some of the values:
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0:6] = joint_goal_list

        is_success = True
        try:
            # The go command can be called with joint values, poses, or without any
            # parameters if you have already set the pose or joint target for the group
            self.move_group.go(joint_goal, wait=True)
            # Calling ``stop()`` ensures that there is no residual movement
            self.move_group.stop()

        except Exception as e:
            print('Error in angles control service : ', e)
            is_success = False
        return is_success

    def plan_trajectory(self, X, Y, Z):
        waypoints = []

        wpose = self.move_group.get_current_pose().pose
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = X
        wpose.position.y = Y
        wpose.position.z = Z

        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,  # waypoints to follow
            0.01,  # eef_step
            0.0)  # jump_threshold
        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan

    def orient_gripper(self, tx, ty, tz):

        current_pose = self.move_group.get_current_pose().pose
        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        pose_goal.position.x = current_pose.position.x
        pose_goal.position.y = current_pose.position.y
        pose_goal.position.z = current_pose.position.z

        quaternion = quaternion_from_euler(tx, ty, tz)
        # type(pose) = geometry_msgs.msg.Pose
        pose_goal.orientation.x = quaternion[0]
        pose_goal.orientation.y = quaternion[1]
        pose_goal.orientation.z = quaternion[2]
        pose_goal.orientation.w = quaternion[3]

        print("new quaternions = ", quaternion)

        plan = self.move_group.plan(pose_goal)
        self.display_trajectory(plan)
        proceed = raw_input("Do you want to proceed ?")
        print("got :", proceed)
        if proceed != "y":
            return False

        self.move_group.set_pose_target(pose_goal)

        is_success = True
        try:
            self.move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()

            # self.move_group.execute(plan, wait=True)
            ## Now, we call the planner to compute the plan and execute it.
            # plan = move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            # self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            # self.move_group.clear_pose_targets()
        except Exception as e:
            print('Error while executing the trajectory : ', e)
            is_success = False

        return is_success

    def go_to_pose_goal(self, X, Y, Z, do_validate=True):
        # type: (object, object, object, object) -> object
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.zz

        current_pose = self.move_group.get_current_pose().pose


        print('Current Position is : ', current_pose)
        print('Trying to go to X : {}, Y: {}, Z: {}'.format(X, Y, Z))

        pose_goal = geometry_msgs.msg.Pose()
        # pose_goal.orientation.w = 1.0
        pose_goal.position.x = X
        pose_goal.position.y = Y
        pose_goal.position.z = Z

        pose_goal.orientation.x = current_pose.orientation.x
        pose_goal.orientation.y = current_pose.orientation.y
        pose_goal.orientation.z = current_pose.orientation.z
        pose_goal.orientation.w = current_pose.orientation.w

        plan = self.move_group.plan(pose_goal)
        #self.display_trajectory(plan)

        if do_validate :
            proceed = raw_input("Do you want to proceed ?")
            print("got :", proceed)
            if proceed != "y":
                return False

        self.move_group.set_pose_target(pose_goal)

        is_success = True
        try:
            plan = self.move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            self.move_group.clear_pose_targets()

            # self.move_group.execute(plan, wait=True)
            ## Now, we call the planner to compute the plan and execute it.
            # plan = move_group.go(wait=True)
            # Calling `stop()` ensures that there is no residual movement
            # self.move_group.stop()
            # It is always good to clear your targets after planning with poses.
            # Note: there is no equivalent function for clear_joint_value_targets()
            # self.move_group.clear_pose_targets()
        except Exception as e:
            print('Error while executing the trajectory : ', e)
            is_success = False

        return is_success

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)
        ## END_SUB_TUTORIAL


    ## Define a function to create the environment of the robot in moveit.
    ## Main object to be defined are :
    ## The table
    ## The Gripper
    ## Any other obstacles present arount the robot can be also defined so that the trajectories
    ## planed by the functions take into account colision avoidance with the obstacles.

    def add_box(self, timeout=4):
        rospy.sleep(2)

        TABLE_HEIGHT = 0.74 # in m
        ROBOT_DEPTH_AT_WORKSPACE_GROUND = -0.0226
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = ROBOT_DEPTH_AT_WORKSPACE_GROUND - TABLE_HEIGHT/2
        self.scene.add_box(self.box_name, box_pose, size=(2.0, 2.0, 0.74))

        wait1 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

    ########################## First Parcel #######################################

        self.box_name = "obstacle"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = -0.1
        #self.scene.add_box(self.box_name, box_pose, size=(0.005, 0.6, 0.2))

        #wait3 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

        self.box_name = "obstacle_1"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0.3 - 0.1
        #self.scene.add_box(self.box_name, box_pose, size=(0.3, 0.005, 0.2))

        #wait4 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

        self.box_name = "obstacle_2"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = -0.3 - 0.1
        #self.scene.add_box(self.box_name, box_pose, size=(0.3, 0.005, 0.2))

        #wait5 = self.wait_for_state_update(box_is_known=True, timeout=timeout)


        self.box_name = "obstacle_3"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.55
        box_pose.pose.position.y = -0.1
        #self.scene.add_box(self.box_name, box_pose, size=(0.005, 0.6, 0.2))

        #wait6 = self.wait_for_state_update(box_is_known=True, timeout=timeout)


    ########################### Second Parcel ######################################

        self.box_name = "obstacle_4"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.25
        box_pose.pose.position.y = 0.6
        #self.scene.add_box(self.box_name, box_pose, size=(0.005, 0.6, 0.2))

        #wait7 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

        self.box_name = "obstacle_5"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = 0.3 - 0.1 + 0.7
        #self.scene.add_box(self.box_name, box_pose, size=(0.3, 0.005, 0.2))

        #wait8 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

        self.box_name = "obstacle_6"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.4
        box_pose.pose.position.y = -0.3 - 0.1 + 0.7
        #self.scene.add_box(self.box_name, box_pose, size=(0.3, 0.005, 0.2))

        #wait9 = self.wait_for_state_update(box_is_known=True, timeout=timeout)


        self.box_name = "obstacle_7"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = self.robot.get_planning_frame()
        box_pose.pose.orientation.w = 0.0
        box_pose.pose.position.z = 0.0
        box_pose.pose.position.x = 0.55
        box_pose.pose.position.y = -0.1 + 0.7
        #self.scene.add_box(self.box_name, box_pose, size=(0.005, 0.6, 0.2))

        #wait10 = self.wait_for_state_update(box_is_known=True, timeout=timeout)


        self.box_name = "gripper"
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "tool0"
        box_pose.pose.orientation.w = 0.0

        tool_dx = 0.12
        tool_dy = 0.12
        tool_dz = 0.15
        box_pose.pose.position.z = tool_dz/2
        box_pose.pose.position.x = 0.0
        box_pose.pose.position.y = 0.0
        self.scene.add_box(self.box_name, box_pose, size=(tool_dx, tool_dy, tool_dz))

        wait2 = self.wait_for_state_update(box_is_known=True, timeout=timeout)

        grasping_group = 'endeffector'
        touch_links = self.robot.get_link_names(group=grasping_group)
        self.scene.attach_box("ee_link", self.box_name, touch_links=touch_links)

        return wait1 and wait2 # and wait3 and wait4 and wait5 and wait6 and wait7 and wait8 and wait9 and wait10


    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
        ## Copy class variables to local variables.
        ## In practice, you should use the class variables directly unless you have a good
        ## reason not to.
        scene = self.scene
        box_name = self.box_name

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
          # Test if the box is in attached objects
          attached_objects = scene.get_attached_objects([box_name])
          is_attached = len(attached_objects.keys()) > 0

          # Test if the box is in the scene.
          # Note that attaching the box will remove it from known_objects
          is_known = box_name in scene.get_known_object_names()

          # Test if we are in the expected state
          if (box_is_attached == is_attached) and (box_is_known == is_known):
            return True

          # Sleep so that we give other threads time on the processor
          rospy.sleep(0.1)
          seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL


r = MoveGroupPythonIntefaceTutorial()

'''
def go_to_angles_CB(request):
    a, b, c, d, e, f = request.a, request.b, request.c, request.d, request.e, request.f
    resp = JointPositionMessageResponse()
    resp.success = True
    proceed = raw_input("Do you want to proceed ?")
    print("got :", proceed)
    if proceed != "Y":
        return False
    r.go_to_joint_pose([a, b, c, d, e, f])
    # except Exception as e:
    #  print('Error while executing the trajectory : ', e)
    #  resp = False
    return resp

def go_to_angles_with_gripper_CB(request):
    a, b, c, d, e, f = request.a, request.b, request.c, request.d, request.e, request.f
    resp = JointPositionMessageResponse()
    resp.success = True
    proceed = raw_input("Do you want to proceed ?")
    print("got :", proceed)
    if proceed != "Y":
        return False
    joint_goal = r.move_group.get_current_joint_values()
    joint_goal[4] = 0
    r.go_to_joint_pose(joint_goal)
    joint_goal[0] = a
    joint_goal[1] = b
    joint_goal[2] = c
    joint_goal[3] = d
    r.go_to_joint_pose(joint_goal)
    joint_goal[4] = e
    joint_goal[5] = f
    r.go_to_joint_pose(joint_goal)
    # except Exception as e:
    #  print('Error while executing the trajectory : ', e)
    #  resp = False
    return resp

def go_to_cartesian_CB(request):
    print ("Request Data==> x=" + str(request.x) + ", y = " + str(request.y) + ", z = " + str(request.z))
    my_response = CartesianPositionMessageResponse()
    x = request.x
    y = request.y
    z = request.z

    # r.rest()
    if r.go_to_pose_goal(x, y, z):
        my_response.success = True
    else:
        my_response.success = False

    return my_response

def go_to_cartesian_CB_no_validation(request):
    print ("Request Data no Validation==> x=" + str(request.x) + ", y = " + str(request.y) + ", z = " + str(request.z))
    my_response = CartesianPositionMessageResponse()
    x = request.x
    y = request.y
    z = request.z

    # r.rest()
    if r.go_to_pose_goal(x, y, z, False):
        my_response.success = True
    else:
        my_response.success = False

    return my_response

def set_gripper_orientation(request):
    my_response = GripperPositionMessageResponse()
    tx = request[0]
    ty = request[1]
    tz = request[2]
    print ("Request Data==> x=" + str(tx) + ", y = " + str(ty) + ", z = " + str(tz))

    # r.rest()
    if r.orient_gripper(tx, ty, tz):
        my_response.success = True
    else:
        my_response.success = False
    # my_response.success = r.go_to_pose_goal(x, y, z)

    return my_response
'''


def move_gripper(position, orientation, do_validate):
    current_pose = r.move_group.get_current_pose().pose
    print(current_pose)

    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.position.x = position[0]
    pose_goal.position.y = position[1]
    pose_goal.position.z = position[2]

    quaternion = quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    pose_goal.orientation.x = quaternion[0]
    pose_goal.orientation.y = quaternion[1]
    pose_goal.orientation.z = quaternion[2]
    pose_goal.orientation.w = quaternion[3]

    print(pose_goal)

    plan = r.move_group.plan(pose_goal)
    r.display_trajectory(plan)

    if do_validate:
        proceed = raw_input("Do you want to proceed ?  y/n : ")
        print("got :", proceed)
        if proceed != "y":
            return False

    is_success = True
    try:
        r.move_group.go(pose_goal, wait=True)
        # Calling `stop()` ensures that there is no residual movement
        r.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        r.move_group.clear_pose_targets()

    except Exception as e:
        print('Error while executing the trajectory : ', e)
        is_success = False
    all_close(pose_goal, r.move_group.get_current_pose().pose, 0.01)
    return is_success


def move_gripper_CB(request):
    position, orientation = ((request.x, request.y, request.z), (request.xt, request.yt, request.zt))
    move_gripper(position, orientation, request.validation)
    return True

'''
def move_gripper_traj(request):
    waypoints = []

    wpose = r.move_group.get_current_pose().pose
    print(wpose)

    for i in range(0, len(request[0])):

        pose_goal = geometry_msgs.msg.Pose()
        
        wpose.position.x = request[0][i]
        wpose.position.y = request[1][i]
        wpose.position.z = request[2][i]
        
        quaternion = quaternion_from_euler(request[3][i], request[4][i], request[5][i])
        wpose.orientation.x = quaternion[0]
        wpose.orientation.y = quaternion[1]
        wpose.orientation.z = quaternion[2]
        wpose.orientation.w = quaternion[3]

        waypoints.append(copy.deepcopy(wpose))
    print("Waypoints : ")
    print(waypoints)
    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0 disabling:
    (plan, fraction) = r.move_group.compute_cartesian_path(
        waypoints,  # waypoints to follow
        0.01,  # eef_step
        0.0)  # jump_threshold
    # Note: We are just planning, not asking move_group to actually move the robot yet:

    ## The trajectory planning at this point dooe not include object avoidance
    ## Therefore for this function to be useful we need to design the trajectory 
    ## ourselfs, by taking into account the obstacles present around the robot!
    ## To be implemented in the future (Design an optimal trajectory between two points
    ## by also avoiding any obstacles in the way.)

    is_success = True
    try:
        print("hello!")
        r.move_group.execute(plan, wait=True)
        print("Yeah")
        # Calling `stop()` ensures that there is no residual movement
        r.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        r.move_group.clear_pose_targets()

    except Exception as e:
        print('Error while executing the trajectory : ', e)
        is_success = False
    all_close(pose_goal, r.move_group.get_current_pose().pose, 0.01)
    return is_success


def move_gripper_traj_CB(request):
    pose = (request.x, request.y, request.z, request.xt, request.yt, request.zt)
    move_gripper_traj(pose)
    return True
'''

def get_global_positions_CB(request):
    get_position_resp = GetPositionMessageResponse()
    car_pos = r.move_group.get_current_pose().pose
    joint_post = r.move_group.get_current_joint_values()
    get_position_resp.x = car_pos.position.x
    get_position_resp.y = car_pos.position.y
    get_position_resp.z = car_pos.position.z
    get_position_resp.ox = car_pos.orientation.x
    get_position_resp.oy = car_pos.orientation.y
    get_position_resp.oz = car_pos.orientation.z
    get_position_resp.ow = car_pos.orientation.w
    get_position_resp.a = joint_post[0]
    get_position_resp.b = joint_post[1]
    get_position_resp.c = joint_post[2]
    get_position_resp.d = joint_post[3]
    get_position_resp.e = joint_post[4]
    get_position_resp.f = joint_post[5]
    return get_position_resp


'''
# create the cartesian pos service
my_cartesian_service = rospy.Service('/pos_command/go_to_cartesian_pos', CartesianPositionMessage, go_to_cartesian_CB)

my_cartesian_service_no_validation = rospy.Service('/pos_command/go_to_cartesian_pos_no_validation',
                                                   CartesianPositionMessage, go_to_cartesian_CB_no_validation)

# create the joint pos service
my_joint_service = rospy.Service('/pos_command/go_to_joint_pos', JointPositionMessage, go_to_angles_CB)

# create the joint pos service with gripper
my_joint_gripper_service = rospy.Service('/pos_command/go_to_joint_pos_with_gripper', JointPositionMessage, go_to_angles_with_gripper_CB)

# gripper to the ground
orient_gripper_to_ground = rospy.Service('/pos_command/set_gripper_orientation', GripperPositionMessage,
                                         set_gripper_orientation)
'''
# get current position
my_global_pos_service = rospy.Service('/pos_command/get_global_positions', GetPositionMessage, get_global_positions_CB)

# EE
ee_service = rospy.Service('/pos_command/go_to_EE', GoalEEPositionMessage, move_gripper_CB)
'''
# EE trajectory (To be implemented)
ee_trajectory_service = rospy.Service('/pos_command/plan_EE_trajectory', GoalEEPosTrajMessage, move_gripper_traj_CB)
'''
print('Ready')

r.rest()

rospy.spin()  # maintain the service open.

## This is a service server, where we initialize the robot, and the environment.
## Moreover, it contains the service to move the robot around,
## in the (x,y,z) plane in position and orientation.

def usage():
    return "%s [x y]" % sys.argv[0]
