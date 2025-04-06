#!/usr/bin/env python3

from tasks.task_base import BaseTask
import rospy

class CommandTask(BaseTask):
    def __init__(self, task_name="command_task"):
        super().__init__(task_name)
    
    def init(self):
        super().init()

    def execute_task(self):
        pass