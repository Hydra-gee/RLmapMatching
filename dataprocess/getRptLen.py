import xml.etree.ElementTree as ET
import math
import csv

def parse_xml(file_name):
    # 解析XML文件
    tree = ET.parse(file_name)
    root = tree.getroot()

    # 定义命名空间
    namespace = '{http://www.topografix.com/GPX/1/1}'

    # 创建一个列表来存储所有的rtept对象
    rtept_list = []

    # 遍历gpx标签下的rte标签，再遍历rte标签下的rtept标签
    for rte in root.findall(namespace + 'rte'):
        for rtept in rte.findall(namespace + 'rtept'):
            # 获取每个rtept标签的lat和lon属性
            lat = float(rtept.get('lat'))
            lon = float(rtept.get('lon'))

            # 创建一个字典来存储lat和lon值
            rtept_obj = {'lat': lat, 'lon': lon}

            # 将rtept对象添加到列表中
            rtept_list.append(rtept_obj)

    return rtept_list

def gpx_length(gpx_list):
    if not gpx_list:
        return 0
    else:
        gpx_length = 0
        prev_entry = gpx_list[0]
        for entry in gpx_list[1:]:
            gpx_length += calc_dist(prev_entry['lat'], prev_entry['lon'], entry['lat'], entry['lon'])
            prev_entry = entry
        return gpx_length

def calc_dist(from_lat, from_lon, to_lat, to_lon):
    normed_dist = calc_normalized_dist(from_lat, from_lon, to_lat, to_lon)
    return 1.2742E7 * math.asin(math.sqrt(normed_dist))

def calc_normalized_dist(from_lat, from_lon, to_lat, to_lon):
    sin_delta_lat = math.sin(math.radians(to_lat - from_lat) / 2.0)
    sin_delta_lon = math.sin(math.radians(to_lon - from_lon) / 2.0)
    return sin_delta_lat * sin_delta_lat + sin_delta_lon * sin_delta_lon * math.cos(math.radians(from_lat)) * math.cos(
        math.radians(to_lat))


output_file = "ground_truth.csv"  # 输出的CSV文件的名称

# 创建或写入CSV文件
with open(output_file, 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    for idx in range(0,2700):
        file_path = '../dataset/dataset/Shanghai/Shanghai/gpxfile/{}.gpx'.format(idx)
        length = gpx_length(parse_xml(file_path))  # 获取统计数据
        writer.writerow([idx, length])  # 将序号和统计数据写入CSV文件