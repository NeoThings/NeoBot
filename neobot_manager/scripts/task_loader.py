#!/usr/bin/env python3

from tasks.task_base import BaseTask
from tasks.task_mapping import MappingTask

TASKS = {
    "mapping": MappingTask
}

class TaskLoader:
    def __init__(self):
        self.task_queue = []  # 任务队列
        self.current_task = None

    def add_task(self, task):
        if task in TASKS:
            task_class = TASKS[task](task)
        else:
            print("Unknown task: {task}")

        if not isinstance(task_class, BaseTask):
            print("Task must be an instance of TaskBase.")
            return
        self.task_queue.append(task_class)
        print(task + " task added to the queue.")

    def execute_tasks(self):
        """
        执行任务队列中的所有任务
        """
        print("Starting task execution...")
        while self.task_queue:
            self.current_task = self.task_queue.pop(0)
            print("Executing task: " + self.current_task.task_name)
            try:
                self.current_task.start_task()
            except Exception as e:
                print("Error while executing task [{self.current_task.task_name}]: {e}")
            finally:
                print(self.current_task.task_name + "task completed.")
        print("All tasks completed.")

if __name__ == "__main__":

    task_loader = TaskLoader()
    task_loader.add_task(MappingTask("mapping_task"))
    #task_loader.execute_tasks()