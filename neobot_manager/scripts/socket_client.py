
import socket
import json
import struct
import time

class PersistentClient:
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
    
    def run(self, callback=None):
        if not self.connect():
            return
        try:
            while self.running:
                response = self.recv_msg(self.socket)
                if response is not None:
                    if callback:
                        callback(response)
                    else:
                        print(f"收到消息: {response}")
                else:
                    print("无法获取服务器响应")
                    self.running = False
                    break
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n正在关闭客户端...")
        finally:
            self.socket.close()

if __name__ == "__main__":
    client = PersistentClient('127.0.0.1', 1234)
    client.run()