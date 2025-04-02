import os
import sys
import time
import psutil
import argparse
import configparser
from multiprocessing import Process
from transitions import Machine
from playaudio import playaudio
from gtts import gTTS

config = configparser.ConfigParser()
config.read('../config/wled.cfg')

class StateMachine(object):
    
    states = ['IDLE', 'MAPPING', 'NAVIGATION', 'CHARGING', 'TRACKING']

    transitions = [
        ['camera_detect_human', 'standby', 'warning']
    ]

    def get_process_pid(self, process_name):
        for proc in psutil.process_iter():
            try:
                if proc.name() == process_name:
                    return proc.pid
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        return None
    
    def terminate_process(self, pid):
        try:
            process = psutil.Process(pid)
            process.terminate()
            process.wait()
            return True
        except psutil.NoSuchProcess:
            return False

    def get_current_hour():
        current_time = time.time()
        current_hour = time.localtime(current_time).tm_hour
        return current_hour
    
    def exec_mapping(self):
      print('exec_mapping')

    # def http_send_night_light(self):
    #     if http_client.get_wled_json()['on']:
    #         pass
    #     else:
    #         data = config.get('effect', 'night_light')
    #         http_client.post_wled_json(wled_addr, data)

    # def at_day(self):
    #     current_time = StateMachine.get_current_hour()
    #     if current_time > 7 and current_time < 24:
    #         return True
    #     else:
    #         print("It's not daytime now")
    #         return False
    
    # def sunny_day(self):
    #     weather_info = http_client.get_weather()
    #     weather = weather_info["weather"][0]["main"]
    #     self.temperature = weather_info["main"]["temp"]

    #     print("current weather is: " + weather)
    #     if weather == "Sun":
    #         return True
    #     else:
    #         return False

    # def broadcast(self):
    #     playaudio_pid = self.get_process_pid("mpv")
    #     if playaudio_pid:
    #         self.terminate_process(playaudio_pid)
    #     else:
    #         pass
    #     cast_text = "今日温度： " + str(round((self.temperature - 273.15), 0))
    #     language = 'zh-CN'
    #     speech = gTTS(text=cast_text, lang=language, slow=False)
    #     speech.save("../audios/temperature.mp3")
    #     playaudio("../audios/temperature.mp3")

    # def play_audio(self):
    #     if os.path.exists("../audios/softrainambient.mp3"):
    #         playaudio("../audios/softrainambient.mp3")
    #     else:
    #         print("path did not exist")

    def __init__(self, name):
        self.name = name

        # self.machine = Machine(model=self, states=StateMachine.states, transitions=StateMachine.transitions, initial='standby')
        
        self.machine = Machine(model=self, states=StateMachine.states, initial='IDLE', ignore_invalid_triggers=True)

        self.machine.add_transition('start_mapping', 'IDLE', 'MAPPING', after='exec_mapping')
    
        # self.machine.add_transition('take_a_break', '*', 'standby', after='http_send_standby')
        # self.machine.add_transition('stop_play', 'music', 'standby', after='http_send_standby')
        # self.machine.add_transition('good_morning', 'sleep', 'weather', conditions='sunny_day', after='http_send_sun')