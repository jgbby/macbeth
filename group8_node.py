import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *
from types import SimpleNamespace
from TimeHelper import TimeHelper  # TODO add to files downloaded

Hz = 30


class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        assert isinstance(id, int)
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.timer = self.create_timer(1 / Hz, self.timer_callback)
        self.timeHelper = TimeHelper(self)
        self.running = False
        self.done = False

    def compute_trajectories(self):
        """
        Inject Trajectory computation code here...
        """
        trajectories = []
        ### -----Insert Trajectories Here-------

        return trajectories

    def upload_trajectories(crazyflies, trajectories):
        """
        Upload trajectories to crazyflies one by one
        """

        for i, traj in enumerate(trajectories):
            for cf in crazyflies:
                cf.uploadTrajectory(traj, i, 0)

    def start(self):
        """
        Start execution of blocks
        """
        trajectories = self.compute_trajectories()
        self.upload_trajectories(trajectories)
        self.execute_blocks()

    def time(self):
        return self.get_clock().now().nanoseconds / 1e9

    def execute_blocks(self):
        """
        Must be injected into...

        Typical format should be:

        start_time = 0.0
        self.timeHelper.sleepUntil(start_time)
        takeoff(crazyflies, height=1.0, duration=2.0)

        start_time = 5.0
        self.wait_until(start_time)
        land(crazyflies, height=0.0, duration=2.0)

        ...

        Where a new start time is added for each block.
        """
        groupState = SimpleNamespace(
            crazyflies=self.crazyflies, timeHelper=self.timeHelper
        )
        ### ---------Insert Execution Code Here------------

        # Block Name: CF4+5+6 Takeoff
        setLEDColorFromHex(groupState, "#f2ff00")
        takeoff(groupState, 2, 3)

        # Block Name: HOVER1
        start_time = 3.02
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0, 0, 0.25, 1)
        goto_duration_relative(groupState, 0, 0, -1, 4)

        # Block Name: HOVER2
        start_time = 8.04
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0, 0, 1, 4)
        goto_duration_relative(groupState, 0, 0, -1, 4)
        goto_duration_relative(groupState, 0, 0, 1, 4)
        goto_duration_relative(groupState, 0, 0, -1, 4)

        # Block Name: RED LIGHT ON
        start_time = 26.5
        self.timeHelper.sleepUntil(start_time)
        setLEDColorFromHex(groupState, "#ff0000")

        # Block Name: HOVER
        start_time = 27
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0, 0, 1, 4)
        goto_duration_relative(groupState, 0, 0, -1, 4)

        # Change Purple
        start_time = 35.02
        self.timeHelper.sleepUntil(start_time)
        setLEDColorFromHex(groupState, "#7a00b3")

        # Block Name: LONG HOVER
        goto_duration_relative(groupState, 0, 0, 1, 4)
        goto_duration_relative(groupState, 0, 0, -1, 4)
        goto_duration_relative(groupState, 0, 0, 1, 4)
        goto_duration_relative(groupState, 0, 0, -1, 4)

        # Premature landing
        start_time = 60.0
        self.timeHelper.sleepUntil(start_time)
        land(groupState, 0, 3)

        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # # Block Name: LONG HOVER
        # start_time = 71.5939990234375
        # self.timeHelper.sleepUntil(start_time)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,1,0.25)
        # goto_velocity_relative_position(groupState, 0,0,-1,0.25)
        # # Block Name: LAND
        # start_time = 116.482001953125
        # self.timeHelper.sleepUntil(start_time)
        # land(groupState, 0,3)

        self.done = True

    def timer_callback(self):
        if not self.running:
            self.start()
            self.running = True