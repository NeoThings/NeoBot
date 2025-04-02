#!/usr/bin/env python3

from task_base import BaseTask
import rospy
from nav_msgs.msg import OccupancyGrid

class MappingTask(BaseTask):
    def __init__(self, task_name="mapping_task"):
        super().__init__(task_name)
    
    def init(self):
        super().init()

    def map_callback(self, msg):
        self.map_data = msg
        self.rospy.loginfo(f"[{self.task_name}] Received map data (size: {len(msg.data)})")

    def execute_task(self):

        if self.map_data:
            self.rospy.loginfo(f"[{self.task_name}] Processing map data...")
            # 在这里添加处理地图数据的逻辑
        else:
            self.rospy.loginfo(f"[{self.task_name}] Waiting for map data...")