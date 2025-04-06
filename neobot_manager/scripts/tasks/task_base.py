#!/usr/bin/env python3

class BaseTask:
    def __init__(self, task_name):
        """
        初始化机器人任务基类
        :param task_name: 任务名称
        """
        self.task_name = task_name
        print(self.task_name + " task created.")
        self.init()

    def init(self):
        """
        初始化任务变量（可被子类重写）
        """
        print(self.task_name + "task initializing")
        self.is_running = False

    def start_task(self):
        """
        启动任务（可被子类重写）
        """
        print("Starting task: " + self.task_name)
        self.is_running = True

    def execute_task(self):
        """
        执行任务逻辑（必须由子类实现）
        """
        raise NotImplementedError("execute_task() must be implemented by the subclass.")

    def stop_task(self):
        """
        停止任务
        """
        print("Stopping task: " + self.task_name)
        self.is_running = False

if __name__ == "__main__":
    # 示例：直接运行基类会报错，因为 execute_task 未实现
    task = BaseTask("base_task")
    task.start_task()
