import requests
import json
import pandas as pd
import time
import socket
import urllib3


def allowed_gai_family():
    return socket.AF_INET


urllib3.util.connection.allowed_gai_family = allowed_gai_family

result = pd.DataFrame(columns=["id", "ori_distance", "distance", "difference", "time"])
ground_truth = pd.read_csv('ground_truth.csv',header=None)
total = 2700
url = 'http://localhost:8989/match?vehicle=car&type=json&gps_accuracy=20&keypoint_num=999&kp_dis_threshold=50&kp_connum_threshold=3&algorithm=3'
oldtime = time.time()
for i in range(0, total):
    with open('../dataset/dataset/Shanghai/Shanghai/gpxfile/{}.gpx'.format(i), 'rb') as f:
        t1 = time.time()
        r = requests.post(url, data=f, headers={'Content-Type': 'application/gpx+xml'})
        t2 = time.time()
        if r.status_code == 200:
            j = json.loads(r.text)
            ori = ground_truth.iloc[i,1]
            dis = j['map_matching']['distance']
            acc = abs(ori - dis) / ori
            dt = t2 - t1
            new_row = pd.Series([str(i), ori, dis, acc, dt], index=result.columns)
            result = result._append(new_row, ignore_index=True)
        else:
            new_row = pd.Series([str(i), r.status_code, 0, 0, 0], index=result.columns)
            result = result._append(new_row, ignore_index=True)
    if i % 10 == 0:
        print("{:.2f}%".format(i * 100 / total))
result.to_csv("match_result/rl_acc20_cuthead.csv", index=False)
