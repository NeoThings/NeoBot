import time
import socket
import json
import struct

from fsm import StateMachine
from task_loader import TaskLoader

def send_msg(sock, msg):
    pass

def recv_msg(sock):
    pass

def execute(cmd):
    #trigger by socket msg
    neobot_fsm.trigger(cmd)
    if (neobot_fsm.states[neobot_fsm.state] == 'MAPPING'):
#        task_loader.add_task(MappingTask("mapping_task"))
        task_loader.execute_tasks()

if __name__ == '__main__':
    neobot_fsm = StateMachine("neobot_fsm")
    print('Initial state: ' + neobot_fsm.state)
    task_loader = TaskLoader()

    s = socket.socket()
    s.settimeout(10.0)

    try:
        s.connect(('localhost', 12345))
        print("manager socket connected")
    except socket.timeout:
        print("manager socket conect timeout")
    except Exception as e:
        print(f"manager socket conect failed: {e}")

    # response = recv_msg(s)
    # print("Response:", response)
    #s.close()