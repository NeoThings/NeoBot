from tasks.task_base import BaseTask
from nav_msgs.msg import OccupancyGrid

class MappingTask(BaseTask):
    def __init__(self, task_name="mapping_task"):
        super().__init__(task_name)
    
    def init(self):
        super().init()

    def start_task(self):
        self.is_running = True
        self.execute_task()

    def execute_task(self):
        print("impl mapping task exec")