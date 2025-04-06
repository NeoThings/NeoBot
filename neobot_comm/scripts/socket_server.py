# 服务端
import socket
import json
import struct
import time

import threading
import signal
import sys

class Server:
    def __init__(self, host='0.0.0.0', port=65432):
        self.host = host
        self.port = port
        self.running = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # 注册信号处理
        signal.signal(signal.SIGINT, self.shutdown) #ctrl+c
        signal.signal(signal.SIGTERM, self.shutdown) #kill
    
    def handle_client(self, conn, addr):
        try:
            print(f"{addr} 连接成功")
            while self.running:
                message = input("请输入消息: ")
                if message.lower() == 'mapping':
                    self.send_msg(conn, {"event": "start_mapping", "task": "mapping", "cmd": ""})
                elif message.lower() == 'save_map':
                    self.send_msg(conn, {"event": "", "task": "mapping", "cmd": "save_map"})
                # data = self.recv_msg(conn)
                # if not data:
                #     break
                # print(f"server 收到来自 {addr} 的消息: {data}")
        except Exception as e:
            print(f"处理 {addr} 时出错: {e}")
        finally:
            conn.close()
            print(f"{addr} 断开连接")
    
    def send_msg(self, sock, msg):
        msg_json = json.dumps(msg)
        msg_bytes = msg_json.encode()
        # 前4字节为消息长度(网络字节序)
        # >I(大端字节序,高位字节在前), !I(网络字节序)
        sock.sendall(struct.pack('>I', len(msg_bytes)) + msg_bytes)

    def recv_msg(self, sock):
        # 读取前4字节获取长度
        raw_len = self.recv_all(sock, 4)
        if not raw_len:
            return None
        msg_len = struct.unpack('>I', raw_len)[0]
        # 读取消息内容
        msg_json = self.recv_all(sock, msg_len).decode()
        return json.loads(msg_json)

    def recv_all(self, sock, n):
        data = bytearray()
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data.extend(packet)
        return data
    
    def run(self):
        try:
            self.socket.bind((self.host, self.port))
            self.socket.listen(5)
            self.running = True
            print(f"服务端启动，监听 {self.host}:{self.port}")
            
            while self.running:
                try:
                    conn, addr = self.socket.accept()
                    thread = threading.Thread(target=self.handle_client, args=(conn, addr))
                    thread.daemon = True
                    thread.start()
                except socket.error:
                    if self.running:
                        print("接受连接时出错")
                    continue
        finally:
            self.shutdown()
    
    def shutdown(self, signum=None, frame=None):
        print("\n正在关闭服务端...")
        self.running = False
        try:
            # 创建虚拟连接以解除accept阻塞
            temp_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            temp_socket.connect((self.host, self.port))
            temp_socket.close()
        except:
            pass
        self.socket.close()
        sys.exit(0)

if __name__ == "__main__":
    server = Server('127.0.0.1', 1234)
    server.run()