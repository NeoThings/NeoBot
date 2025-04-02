# 服务端
import socket
import json
import struct

def send_msg(sock, msg):
    msg_json = json.dumps(msg)
    msg_bytes = msg_json.encode()
    # 前4字节为消息长度(网络字节序)
    sock.sendall(struct.pack('>I', len(msg_bytes)) + msg_bytes)

def recv_msg(sock):
    # 读取前4字节获取长度
    raw_len = recv_all(sock, 4)
    if not raw_len:
        return None
    msg_len = struct.unpack('>I', raw_len)[0]
    # 读取消息内容
    msg_json = recv_all(sock, msg_len).decode()
    return json.loads(msg_json)

def recv_all(sock, n):
    data = bytearray()
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data.extend(packet)
    return data

s = socket.socket()
s.bind(('localhost', 12345))
s.listen(1)
conn, addr = s.accept()

# 接收消息
received = recv_msg(conn)
print("Received:", received)

# 发送响应
send_msg(conn, {"status": "OK", "received": received})

conn.close()