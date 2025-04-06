# 客户端
import socket
import json
import struct
import time

class ExClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.running = True
        self.reconnect_delay = 5
    
    def connect(self):
        while self.running:
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.connect((self.host, self.port))
                print(f"Successfully connect to {self.host}:{self.port}")
                return True
            except (ConnectionRefusedError, socket.timeout) as e:
                print(f"Failed connect : {e}, reconnect after {self.reconnect_delay} seconds ...")
                time.sleep(self.reconnect_delay)
            except Exception as e:
                print(f"Unexpected error: {e}")
                time.sleep(self.reconnect_delay)
        return False
    
    def send_msg(self, sock, msg):
        try:
            msg_json = json.dumps(msg)
            msg_bytes = msg_json.encode()
            sock.sendall(struct.pack('>I', len(msg_bytes)) + msg_bytes)
        except ConnectionResetError:
            print("连接断开，尝试重新连接...")
            if self.connect():
                return self.send_msg(sock, msg)  # 重试发送
            return None
        except Exception as e:
            print(f"通信错误: {e}")
            return None

    def recv_msg(self, sock):
        raw_len = self.recv_all(sock, 4)
        if not raw_len:
            return None
        msg_len = struct.unpack('>I', raw_len)[0]
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
        if not self.connect():
            return
        try:
            # self.send_msg(self.socket, {"type": "socket", "message": "check"})
            while self.running:
                message = input("请输入消息: ")
                if message.lower() == 'mapping':
                    self.send_msg(self.socket, {"event": "start_mapping", "task": "mapping", "cmd": ""})
                elif message.lower() == 'save_map':
                    self.send_msg(self.socket, {"event": "", "task": "mapping", "cmd": "save_map"})
        except KeyboardInterrupt:
            print("\n正在关闭客户端...")
        finally:
            self.socket.close()

if __name__ == "__main__":
    client = ExClient('127.0.0.1', 1234)
    client.run()