import math
import random
import time
from datetime import datetime

from gymnasium.envs.registration import register
import gymnasium
from gymnasium import spaces
import numpy as np
import socket
import requests
import urllib3
import json
from threading import Thread
import pandas as pd

register(
    id="mapmatching-v0",
    entry_point="mmEnv:MMEnv",
)

def sendRequest(index):
    if index is None:
        index = random.randint(0, 2699)
    print("index:",index)
    with open('../dataset/dataset/Shanghai/Shanghai/gpxfile/{}.gpx'.format(index), 'rb') as f:
        r = requests.post(url, data=f, headers={'Content-Type': 'application/gpx+xml'})
        j = json.loads(r.text)
        return index,j

class MyThread(Thread):
    def __init__(self, func, arg):
        Thread.__init__(self)
        self.func = func
        self.result = None
        self.arg = arg

    def run(self):
        self.result = self.func(self.arg)

    def getResult(self):
        return self.result


def allowed_gai_family():
    return socket.AF_INET


urllib3.util.connection.allowed_gai_family = allowed_gai_family
url = 'http://localhost:8989/match?vehicle=car&type=json&gps_accuracy=20&keypoint_num=999&kp_dis_threshold=50&kp_connum_threshold=3&algorithm=5'


class MMEnv(gymnasium.Env):
    metadata = {}

    def __init__(self, render_mode=None):
        # Observations are dictionaries with the agent's and the target's location.
        self.action_space = gymnasium.spaces.Discrete(2)
        self.observation_space = gymnasium.spaces.Box(
            -np.inf, np.inf, shape=(3,), dtype=np.float64
        )
        self.trace_idx = None
        self.serverSocket = None
        self.clientSocket = None
        self.task = None
        self.lasttime = 0
        self.ts = 0
        self.starttime = 0
        self.logCache = pd.DataFrame(columns=["id", "ori_distance", "distance","difference","time"])
        self.ground_truth = pd.read_csv('ground_truth.csv',header=None)
        self.buildServer()

    def buildServer(self):
        self.serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = "localhost"
        port = 7878
        self.serverSocket.bind((host, port))
        self.serverSocket.listen(1)

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        if options and 'trace_idx' in options:
            self.trace_idx = options['trace_idx']
        self.task = MyThread(sendRequest, self.trace_idx)
        self.task.start()
        self.clientSocket, addr = self.serverSocket.accept()
        msg = self.clientSocket.recv(1024)
        observation = json.loads(msg.decode("utf-8"))
        info = {}
        self.starttime = time.time()
        return np.array([observation['state']]), info

    def step(self, action):
        self.ts += 1
        # if self.ts % 50 == 0:
        #     nowt = time.time()
        #     print("ts/s",50/(nowt-self.lasttime))
        #     self.lasttime = nowt
        self.clientSocket.send((str(action)+"\n").encode('utf-8'))
        msg = self.clientSocket.recv(1024)
        result = json.loads(msg.decode("utf-8"))
        if result['done']:
            print("done")
            self.task.join()
            fileIdx,jsonResult = self.task.getResult()
            # ori = jsonResult['map_matching']['original_distance']
            ori = self.ground_truth.iloc[fileIdx,1]
            dis = jsonResult['map_matching']['distance']
            err = abs(ori - dis) / ori
            new_row = pd.Series([str(fileIdx), ori, dis, err,time.time()-self.starttime], index=self.logCache.columns)
            self.logCache.loc[len(self.logCache)] = new_row.values
            #self.logCache._append(new_row, ignore_index=True)
            result['reward'] += - 20 * math.log(err)
            result['err'] = err
        info = {}
        return np.array([result['state']]), result['reward'], result['done'], False, info

    def save(self):
        print("save")
        current_time = datetime.now()
        self.logCache.to_csv("rlLog{:02d}-{:02d}-{:02d}.csv".format(current_time.day, current_time.hour, current_time.minute),index=False)

