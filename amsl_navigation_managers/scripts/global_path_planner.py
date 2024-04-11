#!/usr/bin/python3

import yaml
import math
import make_prompt
import rospy
from std_msgs.msg import Int64MultiArray


class Node:
    def __init__(self, parent=None, position=None):
        self.parent = parent  # 親ノードの設定
        self.position = position  # (row, column)のタプル ※row：行、column：列

        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        # node 同士の比較演算子(==)を使用できるように
        return self.position == other.position


def get_data(file):
    with open(file, "r") as f:
        map_data = yaml.safe_load(f)

    n_list = []
    e_list = []
    for node in map_data["NODE"]:
        n_list.append(node)

    for edge in map_data["EDGE"]:
        e_list.append(edge)

    return n_list, e_list


def get_position(node):
    x = node["point"]["x"]
    y = node["point"]["y"]
    position = (x, y)
    return position


def get_id(position):
    for node in node_list:
        if position[0] == node["point"]["x"]:
            return node["id"]


# ゴールまでの最短経路のリストを返す関数
def aster(start, end):
    # スタート、エンド（ゴール）ノードの初期化
    start_node = Node(None, start)  # 親ノードは無し
    start_node.g = start_node.h = start_node.f = 0
    end_node = Node(None, end)
    end_node.g = end_node.h = end_node.f = 0

    # オープンリストとクローズリストの両方を初期化する
    open_list = []  # 経路候補を入れとくリスト
    closed_list = []  # 計算終わった用済みリスト

    # 経路候補にスタートノードを追加して計算スタート
    open_list.append(start_node)

    # endを見つけるまでループする
    while len(open_list) > 0:
        # 現在のノードを取得する
        current_node = open_list[0]
        current_index = 0

        for index, item in enumerate(open_list):
            # オープンリストの中でF値が一番小さいノードを選ぶ
            if item.f < current_node.f:
                current_node = item
                current_index = index

        # 一番小さいF値のノードをオープンリストから削除して、クローズリストに追加
        open_list.pop(current_index)
        closed_list.append(current_node)

        # ゴールに到達してれば経路(Path)を表示して終了
        if current_node == end_node:
            path = []
            current = current_node
            while current is not None:
                path.append(current.position)
                current = current.parent
            return path[::-1]

        # ゴールに到達していなければ隣接するノードをリスト化する
        next_node_list = []
        for edge in edge_list:
            if current_node.position == get_position(node_list[edge["node_id"][1]]):
                next_node_list.append(get_position(node_list[edge["node_id"][0]]))
            if current_node.position == get_position(node_list[edge["node_id"][0]]):
                next_node_list.append(get_position(node_list[edge["node_id"][1]]))

        # 子ノードを生成
        children = []
        for next_node in next_node_list:

            # ノードの位置を得る
            node_position = (next_node[0], next_node[1])

            # 移動できる位置のノードのみを生成
            new_node = Node(current_node, node_position)

            # 子リストに追加
            children.append(new_node)

        # 各ノードでG, H, Fを計算
        for child in children:

            # 子がクローズドリスト内にある場合戻る
            if len([closed_child for closed_child in closed_list if closed_child == child]) > 0:
                continue

            # G は親ノード + 1
            child.g = math.hypot(new_node.position[0], new_node.position[1])
            # H は （現在位置 - エンド位置)の2乗
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + (
                (child.position[1] - end_node.position[1]) ** 2
            )
            # F = G + H
            child.f = child.g + child.h

            # 子がすでにオープンリスト内にある場合戻る
            if (
                len(
                    [
                        open_node
                        for open_node in open_list
                        if child.position == open_node.position and child.g > open_node.g
                    ]
                )
                > 0
            ):
                continue

            # 子ノードをオープンリストに追加
            open_list.append(child)


# if __name__ == "__main__":
# rospy.init_node("global_path_planner", anonymous=True)
# r = rospy.Rate(10)

filename = "/home/amsl/catkin_ws/src/amsl_navigation_managers/amsl_navigation_managers/sample/map/ikuta_graph.yaml"
node_list, edge_list = get_data(filename)

goal_id = int(make_prompt.answer)

start_node = get_position(node_list[0])
for node in node_list:
    if node["id"] == goal_id:
        end_node = get_position(node)

path = aster(start_node, end_node)

id_list = []
for position in path:
    id_list.append(get_id(position))

# global_path_pub = rospy.Publisher("/path", Int64MultiArray, queue_size=10)
# id_list_forPublish = Int64MultiArray(data=id_list)

# while not rospy.is_shutdown():
# global_path_pub.publish(id_list_forPublish)
# r.sleep()
