import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from crazyflie_py import generate_trajectory
import numpy as np
from blocklyTranslations import *
from types import SimpleNamespace
from TimeHelper import TimeHelper # TODO add to files downloaded
Hz = 30

class worker_node(Node):

    def __init__(self, crazyflies, id=0, num_nodes=1):
        """
        id: a unique id between 0 and num_nodes corresponding to the thread number of this worker
        num_nodes: number of nodes (threads) in total
        """
        super().__init__("worker_node_{}".format(id))
        assert(isinstance(id, int))
        self.id = id
        self.num_nodes = num_nodes
        self.crazyflies = crazyflies

        self.timer = self.create_timer(1/Hz, self.timer_callback)
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
        '''
            Upload trajectories to crazyflies one by one
        '''

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
        groupState = SimpleNamespace(crazyflies=self.crazyflies, timeHelper=self.timeHelper)
        ### ---------Insert Execution Code Here------------
        # Block Name: CF1+2+3 Takeoff
        takeoff(groupState, 2, 3)
        setLEDColorFromHex(groupState, "#f2ff00")
        
        # Block Name: CF2 GOTO1
        start_time = 3.13
        self.timeHelper.sleepUntil(start_time)
        goto_duration(groupState, -2,1.5,2,4)

        # Block Name: CF1+2+3 Follow
        start_time = 7.5
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0, -4, 0, 8)
        # (-2, -2.5, 2)

        # Block Name: CF2 To Wall
        start_time = 18.
        self.timeHelper.sleepUntil(start_time)
        goto_duration(groupState, -4.5,0,2,4)

        # Block Name: Down 0.5 & Hover
        start_time = 22.02
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0, 0, -0.5, 2)
        goto_duration_relative(groupState, 0, 0, 0.5, 2)

        # Block Name: RED LIGHT ON
        start_time = 27.
        self.timeHelper.sleepUntil(start_time)
        setLEDColorFromHex(groupState, "#ff002b")

        # Block Name: HOVER1 & PURPLE
        start_time = 28.0
        self.timeHelper.sleepUntil(start_time)
        goto_duration_relative(groupState, 0,0,-0.5, 2)

        # Block Name: LONG HOVER
        start_time = 32.5839990234375
        self.timeHelper.sleepUntil(start_time)
        setLEDColorFromHex(groupState, "#7a00b3")

        # 21 * 4
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)
        goto_duration_relative(groupState, 0, 0, 0.5, 4)
        goto_duration_relative(groupState, 0, 0, -0.5, 4)

        # Block Name: LAND
        start_time = 120.
        self.timeHelper.sleepUntil(start_time)
        land(groupState, 0,3)

        
        self.done = True
    
    def timer_callback(self):
        if not self.running:
            self.start()
            self.running = True