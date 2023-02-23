from functools import wraps

from franka_msgs.action import Grasp, Homing, Move
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class Hand(Node):

    def __init__(self) -> None:
        super().__init__('panda_hand')
        self.cli_executor = SingleThreadedExecutor()
        self.cli_executor.add_node(self)

        self.move_cli = ActionClient(self, Move, '/panda_gripper/move')
        self.homing_cli = ActionClient(self, Homing, '/panda_gripper/homing')
        self.grasp_cli = ActionClient(self, Grasp, '/panda_gripper/grasp')

    def _process_future(self, task, future):

        def func(future):
            self.cli_executor.spin_until_future_complete(future)
            try:
                res = future.result()
            except Exception as e:
                self.get_logger().info(f"Service call failed {e}")
            return res

        res = func(future)
        if not res.accepted:
            self.get_logger().info("Hand %s rejected by server" % task)
            return False
        self.get_logger().info("Hand goal accepted")
        future = res.get_result_async()
        res = func(future).result
        if res.success:
            self.get_logger().info("Hand %s completed!" % task)
            return True
        else:
            self.get_logger().info("Hand %s failed and error msg: %s" %
                                   (task, res.error))
            return False

    def check(func):

        @wraps(func)
        def wrapper(self, *args, **kwargs):
            clis = ['move_cli', 'homing_cli', 'grasp_cli']
            for c in clis:
                s = getattr(self, c)
                if not s.wait_for_server(timeout_sec=1.0):
                    self.get_logger().info("%s client not ready" % c)
                    return False
            return func(self, *args, **kwargs)

        return wrapper

    @check
    def move(self, width, speed):
        if width < 0:
            width = 0
        elif width > 0.08:
            width = 0.08
        msg = Move.Goal()
        msg.width = width
        msg.speed = speed
        future = self.move_cli.send_goal_async(msg)
        return self._process_future('move', future)

    @check
    def homing(self):
        msg = Homing.Goal()
        future = self.homing_cli.send_goal_async(msg)
        return self._process_future('homing', future)

    @check
    def grasp(self, width, speed, force):
        if width < 0:
            width = 0
        elif width > 0.08:
            width = 0.08
        msg = Grasp.Goal()
        msg.width = width
        msg.speed = speed
        msg.force = force
        future = self.grasp_cli.send_goal_async(msg)
        return self._process_future('grasp', future)