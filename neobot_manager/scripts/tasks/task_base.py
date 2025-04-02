#!/usr/bin/env python3

import rospy

class BaseTask:
    def __init__(self, task_name):
        """
        初始化机器人任务基类
        :param task_name: 任务名称
        """
        self.task_name = task_name
        self.rospy = rospy
        self.rospy.init_node(self.task_name, anonymous=True)
        self.rospy.loginfo(f"[{self.task_name}] Task initialized.")
        self.init()

    def init(self):
        """
        初始化任务变量（可被子类重写）
        """
        self.rospy.loginfo(f"[{self.task_name}] Initializing variables...")
        # 示例变量
        self.is_running = False

    def start_task(self):
        """
        启动任务（可被子类重写）
        """
        self.rospy.loginfo(f"[{self.task_name}] Starting task...")
        self.is_running = True
        rate = self.rospy.Rate(self.rate)
        while not self.rospy.is_shutdown() and self.is_running:
            self.execute_task()
            rate.sleep()

    def execute_task(self):
        """
        执行任务逻辑（必须由子类实现）
        """
        raise NotImplementedError("execute_task() must be implemented by the subclass.")

    def stop_task(self):
        """
        停止任务
        """
        self.rospy.loginfo(f"[{self.task_name}] Stopping task...")
        self.is_running = False

if __name__ == "__main__":
    try:
        # 示例：直接运行基类会报错，因为 execute_task 未实现
        task = BaseTask("basic_task")
        task.start_task()
    except rospy.ROSInterruptException:
        pass