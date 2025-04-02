#!/usr/bin/env python3

import rospy
from tasks.task_base import BaseTask
from tasks.task_mapping import MappingTask

class TaskLoader:
    def __init__(self):
        """
        初始化任务加载器
        """
        rospy.init_node('task_loader', anonymous=True)
        self.task_queue = []  # 任务队列
        self.current_task = None

    def add_task(self, task):
        """
        添加任务到队列
        :param task: 继承自 RobotTaskBase 的任务实例
        """
        if not isinstance(task, BaseTask):
            rospy.logerr("Task must be an instance of TaskBase.")
            return
        self.task_queue.append(task)
        rospy.loginfo(f"Task [{task.task_name}] added to the queue.")

    def execute_tasks(self):
        """
        执行任务队列中的所有任务
        """
        rospy.loginfo("Starting task execution...")
        while self.task_queue and not rospy.is_shutdown():
            self.current_task = self.task_queue.pop(0)
            rospy.loginfo(f"Executing task: {self.current_task.task_name}")
            try:
                self.current_task.start_task()
            except Exception as e:
                rospy.logerr(f"Error while executing task [{self.current_task.task_name}]: {e}")
            finally:
                rospy.loginfo(f"Task [{self.current_task.task_name}] completed.")
        rospy.loginfo("All tasks completed.")

if __name__ == "__main__":
    try:
        task_loader = TaskLoader()
        task_loader.add_task(MappingTask("mapping_task"))

        #task_loader.execute_tasks()

    except rospy.ROSInterruptException:
        pass