from functools import wraps

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose, Quaternion
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotState, WorkspaceParameters, Constraints, PositionConstraint, OrientationConstraint
from moveit_msgs.srv import GetPositionIK, GetPositionFK, GetMotionPlan, GetCartesianPath
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

from moveit_franka.util import Subscriber

IK_LINK = 'panda_hand_tcp'
BASE_LINK = "panda_link0"
PLANNING_GROUP = "panda_arm"


class Arm(Node):

    def __init__(self):
        super().__init__('panda_arm')
        self.cli_executor = SingleThreadedExecutor()
        self.cli_executor.add_node(self)

        self.sub_executor = SingleThreadedExecutor()
        self.joint_subscriber = Subscriber("arm_joints", JointState,
                                           '/franka/joint_states')
        self.sub_executor.add_node(self.joint_subscriber)

        self.fk = self.create_client(GetPositionFK, '/compute_fk')
        self.ik = self.create_client(GetPositionIK, '/compute_ik')

        self.mp = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        self.cart = self.create_client(GetCartesianPath,
                                       '/compute_cartesian_path')
        self.traj = ActionClient(self, ExecuteTrajectory,
                                 '/execute_trajectory')

    def _get_workspace_params(self, workspace_dim=5.0):
        ws_param = WorkspaceParameters()
        ws_param.header.stamp = self.get_clock().now().to_msg()
        ws_param.header.frame_id = BASE_LINK
        ws_param.min_corner.x = workspace_dim
        ws_param.min_corner.y = workspace_dim
        ws_param.min_corner.z = workspace_dim
        ws_param.max_corner.x = workspace_dim
        ws_param.max_corner.y = workspace_dim
        ws_param.max_corner.z = workspace_dim
        return ws_param

    def _get_position_constraint(self, position: Pose,
                                 pos_tolerance: float) -> PositionConstraint:
        pos_constraint = PositionConstraint()
        pos_constraint.header.stamp = self.get_clock().now().to_msg()
        pos_constraint.header.frame_id = BASE_LINK
        pos_constraint.link_name = IK_LINK
        tol_sphere = SolidPrimitive()
        tol_sphere.type = 2  #Sphere code
        tol_sphere.dimensions.append(pos_tolerance)  #Sphere radius
        pos_constraint.constraint_region.primitives.append(tol_sphere)
        pos_constraint.constraint_region.primitive_poses.append(position)
        pos_constraint.weight = 1.0
        return pos_constraint

    def _get_orientation_constraint(
            self, orientation: Quaternion,
            ang_tolerance: float) -> OrientationConstraint:
        #Define all orientation constraints
        constraint = OrientationConstraint()
        constraint.header.stamp = self.get_clock().now().to_msg()
        constraint.header.frame_id = BASE_LINK
        constraint.link_name = IK_LINK
        constraint.orientation = orientation
        constraint.absolute_x_axis_tolerance = ang_tolerance
        constraint.absolute_y_axis_tolerance = ang_tolerance
        constraint.absolute_z_axis_tolerance = ang_tolerance
        constraint.parameterization = 0  #Euler angles
        constraint.weight = 1.0
        return constraint

    def _get_pose_goal_constraints(self, goal_pose, pos_tolerance,
                                   ang_tolerance):
        #Define goal position constraint
        my_pos_constraint = self._get_position_constraint(
            goal_pose, pos_tolerance)
        #Define goal orientation constraint
        my_ori_constraint = self._get_orientation_constraint(
            goal_pose.orientation, ang_tolerance)
        #Populate and return constraints message
        goal_const = Constraints()
        goal_const.name = 'goal_pose_constraint'
        goal_const.position_constraints.append(my_pos_constraint)
        goal_const.orientation_constraints.append(my_ori_constraint)
        return goal_const

    def _process_future(self, future):
        self.cli_executor.spin_until_future_complete(future)
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().info(f"Service call failed {e}")
        return res

    def check(func):

        @wraps(func)
        def wrapper(self, *args, **kwargs):
            clis = ['fk', 'ik', 'mp', 'cart']
            for c in clis:
                s = getattr(self, c)
                if not s.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info("%s client not ready" % c)
                    return False
            if not self.traj.wait_for_server(timeout_sec=1.0):
                self.get_logger().info("traj client not ready")
                return False
            self.sub_executor.spin_once()
            if not self.joint_subscriber.get_current_msg():
                self.get_logger().info("joint states not received")
                return False
            return func(self, *args, **kwargs)

        return wrapper

    @check
    def get_current_joint_state(self):
        self.sub_executor.spin_once()
        state = RobotState()
        state.joint_state = self.joint_subscriber.get_current_msg()
        return state

    @check
    def get_current_pose(self):
        return self.get_fk(self.get_current_joint_state(), IK_LINK)

    @check
    def get_fk(self, robot_state, link_name):
        req = GetPositionFK.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = BASE_LINK
        req.fk_link_names = [link_name]
        req.robot_state = robot_state
        future = self.fk.call_async(req)
        res = self._process_future(future)
        if res.error_code.val != 1:
            self.get_logger().info(
                f"FK request failed. Error code: {res.error_code.val}")
            return None
        else:
            return res.pose_stamped[0].pose

    @check
    def get_ik(self, pose, timeout_s=0, timeout_ns=1e9):
        req = GetPositionIK.Request()
        req.ik_request.group_name = PLANNING_GROUP
        req.ik_request.robot_state = self.get_current_joint_state()
        req.ik_request.avoid_collisions = True
        req.ik_request.ik_link_name = IK_LINK
        req.ik_request.pose_stamped = pose
        timeout = Duration()
        timeout.sec = int(timeout_s)
        timeout.nanosec = int(timeout_ns)
        req.ik_request.timeout = timeout
        future = self.ik.call_async(req)
        res = self._process_future(future)
        if res.error_code.val != 1:
            self.get_logger().info(
                f"IK request failed. Error code: {res.error_code.val}")
            return None
        else:
            return res.solution

    @check
    def plan_pose(
        self,
        goal_pose,
        pos_tolerance,
        ang_tolerance,
        fix_ee_orientation,
        num_attempts,
        timeout,
        path_ang_tol,
        velocity_scaling,
    ):
        req = GetMotionPlan.Request()
        req.motion_plan_request.workspace_parameters = self._get_workspace_params(
        )
        req.motion_plan_request.start_state = self.get_current_joint_state()
        req.motion_plan_request.goal_constraints.append(
            self._get_pose_goal_constraints(goal_pose, pos_tolerance,
                                            ang_tolerance))
        if fix_ee_orientation:
            #Define path constraint such that EE translates without changes in orientation
            path_const = Constraints()
            path_const.name = 'path_constraints'
            path_const.orientation_constraints.append(
                self._get_orientation_constraint(goal_pose.orientation,
                                                 path_ang_tol))
            req.motion_plan_request.path_constraints = path_const
        req.motion_plan_request.group_name = PLANNING_GROUP
        req.motion_plan_request.num_planning_attempts = num_attempts
        req.motion_plan_request.allowed_planning_time = timeout
        req.motion_plan_request.max_velocity_scaling_factor = velocity_scaling
        future = self.mp.call_async(req)
        res = self._process_future(future)
        if res.motion_plan_response.error_code.val != 1:
            self.get_logger().error(
                f"Motion plan failed. Error code: {res.motion_plan_response.error_code.val}"
            )
            return None
        else:
            self.get_logger().info(
                f"Motion plan was successful! Planning took {res.motion_plan_response.planning_time} seconds"
            )
            return res.motion_plan_response.trajectory

    @check
    def plan_cartesian(self, goal_pose, step):
        req = GetCartesianPath.Request()
        req.header.stamp = self.get_clock().now().to_msg()
        req.header.frame_id = BASE_LINK
        req.start_state = self.get_current_joint_state()
        req.group_name = PLANNING_GROUP
        req.link_name = IK_LINK
        req.waypoints.append(goal_pose)
        req.max_step = step
        req.jump_threshold = 0.0
        req.prismatic_jump_threshold = 0.0
        req.revolute_jump_threshold = 0.0
        req.avoid_collisions = True
        future = self.cart.call_async(req)
        res = self._process_future(future)
        if res.error_code.val != 1:
            self.get_logger().error(
                f"Motion plan failed. Error code: {res.error_code.val}")
            return None
        else:
            self.get_logger().info(f"Cartesian plan was successful!")
            return res.solution

    @check
    def execute_trajectory(self, trajectory):
        traj_msg = ExecuteTrajectory.Goal()
        traj_msg.trajectory = trajectory
        future = self.traj.send_goal_async(traj_msg)
        res = self._process_future(future)
        if not res.accepted:
            self.get_logger().info("Trajectory rejected by server")
            return False
        self.get_logger().info("Goal accepted")
        future = res.get_result_async()
        res = self._process_future(future)
        action_result = res.result
        status = res.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info("Trajectory Execution completed!")
            return True
        else:
            self.get_logger().info(
                "Trajectory Execution failed with status: {0} and error code: {1}"
                .format(status, action_result.error_code.val))
            return False

    @check
    def move_pose(
        self,
        pose,
        velocity_scaling=0.3,
        fix_ee_orientation=False,
        pos_tolerance=0.001,
        ang_tolerance=0.01,
        num_attempts=10,
        timeout_s=2.0,
        path_ang_tol=0.1,
    ):
        trajectory = self.plan_pose(
            goal_pose=pose,
            pos_tolerance=pos_tolerance,
            ang_tolerance=ang_tolerance,
            fix_ee_orientation=fix_ee_orientation,
            num_attempts=num_attempts,
            timeout=timeout_s,
            path_ang_tol=path_ang_tol,
            velocity_scaling=velocity_scaling,
        )
        if trajectory is None:
            return False
        return self.execute_trajectory(trajectory)

    @check
    def move_cartesian(self, pose, step=0.0025):
        trajectory = self.plan_cartesian(pose, step)
        if trajectory is None:
            return False
        return self.execute_trajectory(trajectory)