import time
import socket
import json
import struct

from fsm import StateMachine
from task_loader import TaskLoader
from socket_client import PersistentClient

def handle_message(msg):
    if msg['event'] != '':
        neobot_fsm.trigger(msg['event'])
        print('current state: ' + neobot_fsm.state)
    if msg['task'] != '':
        task_loader.add_task(msg['task'])
        task_loader.execute_tasks()

def on_message_received(message):
    print(f"收到消息: {message}")
    handle_message(message)

if __name__ == '__main__':
    neobot_fsm = StateMachine("neobot_fsm")
    print('Initial state: ' + neobot_fsm.state)
    task_loader = TaskLoader()
    socket_client = PersistentClient('127.0.0.1', 1234)
    socket_client.run(on_message_received)
    