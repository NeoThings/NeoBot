# 客户端
import socket
import json
import struct

def send_msg(sock, msg):
    # 同上服务端实现
    pass

def recv_msg(sock):
    # 同上服务端实现
    pass

s = socket.socket()
s.connect(('localhost', 12345))

# 发送消息
send_msg(s, {"type": "greeting", "message": "Hello Server"})

# 接收响应
response = recv_msg(s)
print("Response:", response)

s.close()