import base64
import cv2
import json
import math
import numpy as np
import os
from pathlib import Path as FilesystemPath
import rclpy
import shutil
import time
import yaml
from ament_index_python.packages import get_package_prefix
from aid_robot_msgs.srv import MapImage
from aid_robot_msgs.srv import MapLinkedDataList
from aid_robot_msgs.srv import MapList
from aid_robot_msgs.srv import MapOperationAdd
from aid_robot_msgs.srv import OperationAdd
from aid_robot_msgs.srv import OperationDelete
from aid_robot_msgs.srv import OperationGet
from aid_robot_msgs.srv import OperationUpdate
from aid_robot_msgs.srv import ForbiddenGet
from aid_robot_msgs.srv import GetCurrentForbidden
from aid_robot_msgs.srv import SetCurrentMap, GetCurrentMap
from aid_robot_msgs.srv import GetCurrentForbidden
from aid_robot_msgs.msg import StartToEndPoint as ForbiddenLine
from aid_robot_msgs.srv import GetDockPose, SetDockPose
from aid_robot_msgs.msg import AidPoses,AidPose
from aid_robot_msgs.srv import PointGet
from aid_robot_msgs.srv import ForbiddenSet

from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, MapMetaData
from rclpy.node import Node

from .db import SQLite


class MapManagerNode(Node):
    def __init__(self):
        super().__init__("map_manager_server")
        db_file_path = os.path.expanduser("~")+"/maps/db.sqlite"
        sql_file_dir = os.path.expanduser("~")+"/maps"
        
        if os.path.exists(sql_file_dir) is False:
            os.makedirs(sql_file_dir, exist_ok=True)
            print(f"create dir {sql_file_dir}")

        self.conn = SQLite(db_file_path)
        # self.declare_parameter("map_manager_server")
        # self.map_path = self.get_parameter("map_path").get_parameter_value().string_array_value

        # 地图操作
        self.create_service(MapOperationAdd, "add_map", self.add_map_callback)
        self.create_service(OperationDelete, "delete_map", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_map", self.update_data_callback)
        self.create_service(MapList, "get_map_list", self.get_map_list_callback)
        self.create_service(MapImage, "get_map_image", self.get_map_image_callback)
        self.create_service(SetCurrentMap, "set_current_map_id", self.set_current_map_callback)
        self.create_service(GetCurrentMap, "get_current_map_id", self.get_current_map_callback)

        # 位置点操作
        self.create_service(OperationGet, "get_waypoint", self.get_data_callback)
        self.create_service(OperationAdd, "add_waypoint", self.add_data_callback)
        self.create_service(OperationDelete, "delete_waypoint", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_waypoint", self.update_data_callback)
        self.create_service(MapLinkedDataList, "get_map_waypoint_list", self.get_data_list_callback)

        # 单点
        self.create_service(PointGet, "get_point", self.get_point_callback)
        self.create_service(OperationAdd, "add_point", self.add_data_callback)
        self.create_service(OperationDelete, "delete_point", self.delete_data_callback)
        self.create_service(OperationUpdate, "update_point", self.update_point_callback)
        self.create_service(MapLinkedDataList, "get_map_point_list", self.get_data_list_callback)

        # 禁行线操作
        self.create_service(ForbiddenGet, "get_forbidden", self.get_forbidden_callback)
        self.create_service(ForbiddenSet, "set_forbidden", self.set_forbidden_callback)
        self.create_service(OperationDelete, "delete_forbidden", self.delete_data_callback)
        self.create_service(GetCurrentForbidden, "get_current_forbidden", self.get_current_forbidden)

        self.add_service = self.create_service(SetDockPose, 'set_dock_pose', self.set_dock_pose_callback)
        self.get_service = self.create_service(GetDockPose, 'get_dock_pose', self.get_dock_pose_callback)

    def _remove_map_files(self, file_path: str):
        """Remove one registered map without invoking a shell or following arbitrary paths."""
        alias = FilesystemPath(file_path).expanduser()
        legacy_root = FilesystemPath.home() / "maps"
        lightning_root = FilesystemPath("/opt/G1/maps")

        if not alias.is_absolute() or not alias.is_relative_to(legacy_root):
            raise ValueError(f"refusing to delete map path outside {legacy_root}: {alias}")

        target = alias.resolve(strict=False) if alias.is_symlink() else None
        if alias.is_symlink():
            alias.unlink()
        elif alias.exists():
            shutil.rmtree(alias)

        # Lightning compatibility entries are symlinks such as
        # ~/maps/<id> -> /opt/G1/maps/<id>/lightning. Remove that map only.
        if target is not None and target.is_relative_to(lightning_root):
            relative = target.relative_to(lightning_root)
            if len(relative.parts) == 2 and relative.parts[1] == "lightning":
                map_root = lightning_root / relative.parts[0]
                if map_root.exists():
                    shutil.rmtree(map_root)

    @staticmethod
    def _map_yaml_path(file_path: str) -> FilesystemPath:
        map_directory = FilesystemPath(file_path).expanduser()
        root_yaml = map_directory / "map.yaml"
        if root_yaml.is_file():
            return root_yaml
        return map_directory / "lightning" / "map.yaml"

    def is_map_id_exist(self, map_id: int) -> bool:
        """
        Check if a map_id exists in the map table
        """
        sql_check_map_id = "SELECT 1 FROM map WHERE id = ?"
        result = self.conn.execSQL(sql_check_map_id, (map_id,))
        return len(result) > 0

    def set_forbidden_callback(self, request, response):
        """
        Handle service requests to add forbidden lines
        """

        if not self.is_map_id_exist(request.map_id):
            response.success = False
            response.message = f"Map ID {request.map_id} does not exist."
            self.get_logger().error(response.message)
            return response

        try:
            # Normalize incoming lines to a JSON-serializable list
            if hasattr(request, 'lines') and isinstance(request.lines, list) and len(request.lines) > 0:
                first = request.lines[0]
                if hasattr(first, 'start') and hasattr(first, 'end'):
                    point_list = self.__forbiddenlines_to_point_list(request.lines)
                else:
                    point_list = request.lines
            else:
                point_list = []

            # Enforce one forbidden entry per map_id: update if exists, otherwise insert
            sql_check = "select id from forbidden where map_id=?"
            existing = self.conn.execSQL(sql_check, (request.map_id,))
            now = int(time.time())
            if len(existing) > 0:
                sql_update = "update forbidden set frame_id=?, point_list=?, edit_timestamp=? where map_id=?"
                self.conn.execSQL(sql_update, (request.frame_id, json.dumps(point_list), now, request.map_id))
            else:
                sql_insert = "insert into forbidden(map_id, frame_id, point_list, create_timestamp, edit_timestamp, creator_id) values(?, ?, ?, ?, ?, ?)"
                self.conn.execSQL(sql_insert, (request.map_id, request.frame_id, json.dumps(point_list), now, now, 0))
            response.success = True
            response.message = f"Successfully added forbidden lines: {request.map_id}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to add forbidden lines: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def set_dock_pose_callback(self, request, response):
        """
        Handle service requests to add dock pose
        """
        try:
            sql_set_dock_pose = "INSERT OR REPLACE INTO dock_poses (map_id, position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w) VALUES (?, ?, ?, ?, ?, ?, ?, ?)"
            self.conn.execSQL(sql_set_dock_pose, (
                request.map_id, request.point.position.x, request.point.position.y, request.point.position.z,
                request.point.orientation.x, request.point.orientation.y,
                request.point.orientation.z, request.point.orientation.w
            ))
            response.success = True
            response.message = f"Successfully added dock pose: {request.map_id}"
            self.get_logger().info(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to add dock pose: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def get_dock_pose_callback(self, request, response):
        """
        Handle service requests to get a dock pose
        """
        try:
            sql_get_dock_pose = "SELECT position_x, position_y, position_z, orientation_x, orientation_y, orientation_z, orientation_w FROM dock_poses WHERE map_id = ?"
            result = self.conn.execSQL(sql_get_dock_pose, (request.map_id,))
            if result:
                point_data = result[0]
                response.point = Pose()
                response.point.position.x = point_data['position_x']
                response.point.position.y = point_data['position_y']
                response.point.position.z = point_data['position_z']
                response.point.orientation.x = point_data['orientation_x']
                response.point.orientation.y = point_data['orientation_y']
                response.point.orientation.z = point_data['orientation_z']
                response.point.orientation.w = point_data['orientation_w']

                response.success = True
                response.message = f"Found dock pose: {request.map_id}"
                self.get_logger().info(response.message)
            else:
                response.success = False
                response.message = f"No dock pose found with map_id: {request.map_id}"
                self.get_logger().warning(response.message)
        except Exception as e:
            response.success = False
            response.message = f"Failed to get dock pose: {str(e)}"
            self.get_logger().error(response.message)

        return response

    def set_current_map_callback(self, request, response):
        # 设置为当前地图接口
        try:
            #print("set_current_map in") # debug print
            sql_clear_current_map = "delete from current_map"
            self.conn.execSQL(sql_clear_current_map)
            #print("delete current") # debug print

            map_id = request.id

            sql_get_map_file = "select file_path, name from map where id=?"
            result = self.conn.execSQL(sql_get_map_file, (map_id,))
            #print("get map file path") # debug print

            sql_insert_current_map = "insert into current_map(id, file_path, name) values(?, ?, ?)"
            self.conn.execSQL(sql_insert_current_map, (map_id, result[0]["file_path"], result[0]["name"]))
            response.success = True
            #print("set_current_map ok") # debug print
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
        return response

    def get_current_map_callback(self, request, response):
        # 请求当前地图接口
        try:
            #print("get_current_map in") # debug print
            sql_get_current = "select id, file_path, name from current_map"
            result = self.conn.execSQL(sql_get_current)
        except Exception as e:
            #print("get_current_map error") # debug print
            response.success = False
            response.map_id = 0
            response.map_file = ""
            response.map_name = ""
        else:
            response.success = len(result) > 0
            if response.success:
                #print("get_current_map ok") # debug print
                _ = result[0]
                response.map_id = _.get("id", 0)
                response.map_file = _.get("file_path", "")
                response.map_name = _.get("name", "")
            else:
                #print("no current map") # debug print
                response.map_id = 0
                response.map_file = ""
                response.map_name = ""
        return response

    def add_map_callback(self, request, response):
        print("add_map in") # debug print
        try:
            map_name = request.map_name.strip()
            file_path = os.path.expanduser("~") + request.map_file.strip()
            creator_id = 0
            sql_check_map_name = "select 1 from map where name=? and creator_id=?"
            result = self.conn.execSQL(sql_check_map_name, (map_name, creator_id))
            if len(result) > 0:
                print("add_map failed") # debug print
                response.success = False
                response.message = "map name duplicated"
            else:
                # 判断新增的地图文件是否存在
                map_yaml_file = self._map_yaml_path(file_path)
                if map_yaml_file.is_file():
                    sql_add_map = "insert into map(name, file_path, create_timestamp, edit_timestamp, creator_id) values(?, ?, ?, ?, ?)"
                    now = int(time.time())
                    self.conn.execSQL(sql_add_map, (map_name, file_path, now, now, creator_id))
                    response.success = True
                    response.message = "ok"
                    print("add_map ok") # debug print
                else:
                    print("add_map not exists file " + str(map_yaml_file)) # debug print
                    response.success = False
                    response.message = "map file not exists " + str(map_yaml_file)
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
            #print("add_map error") # debug print
        return response

    def get_map_image_callback(self, request, response):
        #print("get_map_image in") # debug print
        # 默认状态 / 没找到地图记录 / 查询出现问题，都是以下值作为返回值
        response.success = False
        response.map_file = ""
        # 通过地图id，查找地图文件路径
        map_id = (request.id,)
        try:
            sql_query_map_file = "select file_path from map where id=?"
            result = self.conn.execSQL(sql_query_map_file, map_id)
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)
            #print("get_map_image error") # debug print
        else:
            print(f"select map:{result}, {len(result)}")
            if len(result) > 0:
                print("get_map_image in2") # debug print
                _ = result[0]
                map_yaml_file = self._map_yaml_path(_["file_path"])
                response.success = map_yaml_file.is_file()
                response.map_file = str(map_yaml_file)
                if response.success:
                    # 打开 filepath，并转成 nav_msgs/OccupancyGrid 类型
                    self.__map_pgm_to_grid(str(map_yaml_file), response.map)
                    #print("get_map_image ok") # debug print
            print("get_map_image out") # debug print
        return response

    def get_map_list_callback(self, request, response):
        #print("get_map_list in") # debug print
        # 查询地图列表
        try:
            sql_query_map_list = "select id, name, create_timestamp from map where creator_id=0"
            result = self.conn.execSQL(sql_query_map_list)

            response.success = True
            if len(result) > 0:
                response.map_list = json.dumps(result)
                #print("get_map_list ok") # debug print

            else:
                response.map_list = json.dumps([])
                #print("get_map_list no map") # debug print
        except Exception as e:
            #print("get_map_list error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.map_list = json.dumps([])

        return response

    def get_data_callback(self, request, response):
        #print("get_data in") # debug print
        data_id = (request.id,)
        sql_get_data = f"select id, frame_id, point_list from {request.data_type} where id=?"
        try:
            result = self.conn.execSQL(sql_get_data, data_id)
            response.success = len(result) > 0
            if response.success:
                #print("get_data ok") # debug print
                _ = result[0]
                response.message = self.__point_list_to_path(_["frame_id"], json.loads(_["point_list"]))

            else:
                #print("get_data no data") # debug print
                response.message = None  # "waypoint not found"
        except Exception as e:
            #print("get_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = None

        return response

    def add_data_callback(self, request, response):
        # print("add_data in") # debug print
        map_id = request.map_id
        try:
            sql_add_data = f"insert into {request.data_type}(map_id, point_list, frame_id, create_timestamp, edit_timestamp, creator_id) values (?, ?, ?, ?, ?, ?)"
            now = int(time.time())
            point_list = request.data
            print(f"request.data {request.data} type {type(request.data)}")
            print(f"request.frame_id {request.frame_id} type {type(request.frame_id)}")
            self.conn.execSQL(sql_add_data, (map_id, point_list, request.frame_id, now, now, 0))
            # print(4)
            response.success = True
            # print("add_data ok") # debug print
            response.message = "ok"
        except Exception as e:
            # print("add_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
        return response

    def delete_data_callback(self, request, response):
        #print("delete_data in") # debug print
        data_id = (request.id,)
        try:
            if request.data_type == "map":
                #print("delete_data in map") # debug print
                # 如果是删除 map，则同步删除map对应的其他数据
                sql_get_map_file = "select file_path from map where id=?"
                result = self.conn.execSQL(sql_get_map_file, data_id)
                if len(result):
                    # 删除地图对应的定位点记录
                    sql_delete_waypoint = "delete from waypoint where map_id=?"
                    self.conn.execSQL(sql_delete_waypoint, data_id)

                    sql_delete_point = "delete from waypoint_node where map_id=?"
                    self.conn.execSQL(sql_delete_point, data_id)
                    
                    #print("delete_data 1") # debug print
                    # 删除地图对应的禁行线记录
                    sql_delete_forbidden = "delete from forbidden where map_id=?"
                    self.conn.execSQL(sql_delete_forbidden, data_id)
                    #print("delete_data 2") # debug print

                    sql_delete_current_map = "delete from current_map where id=?"
                    self.conn.execSQL(sql_delete_current_map, data_id)
                    #print("delete_data 3") # debug print

                    # 删除文件系统里对应地图文件。路径必须位于受控地图目录，
                    # 且不通过 shell 拼接，避免误删与命令注入。
                    self._remove_map_files(result[0]['file_path'])

            sql_delete_data = f"delete from {request.data_type} where id=?"
            self.conn.execSQL(sql_delete_data, data_id)
            response.success = True
            response.message = "ok"
            #print("delete_data ok") # debug print
        except Exception as e:
            #print("delete_data error") # debug print
            print(e)
            response.success = False
            response.message = "error"
        return response

    def update_data_callback(self, request, response):
        #print("update_data in") # debug print
        try:
            sql_get_data = f"select 1 from {request.data_type} where id=?"
            result = self.conn.execSQL(sql_get_data, (request.id,))
            response.success = len(result) > 0
            if response.success:
                data = json.loads(request.data)
                sql_keys = ["edit_timestamp=?"]
                sql_values = [int(time.time())]
                print(f"data {data}")
                for k, v in data.items():
                    sql_keys.append(f"{k}=?")
                    sql_values.append(v)
                    print(f"k {k}, v {v}, type(v) {type(v)}")
                sql_values.append(request.id)  # id 这个一定要放在更新值列表之后因为是最后作为 where 子句的条件存在
                print(f"sql_values {sql_values}")
                update_data_str = ",".join(sql_keys)  # 将字段更新列表拼接成字符串
                sql_update_data = f"update {request.data_type} set {update_data_str} where id=?"
                print(f"sql_update_data {sql_update_data}")

                self.conn.execSQL(sql_update_data, sql_values)
                response.message = "ok"
                #print("update_data ok") # debug print
            else:
                #print("update_data no data") # debug print
                response.message = "data not found"
        except Exception as e:
            #print("update_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
        return response
    def get_data_list_callback(self, request, response):
        #print("get_data_list in") # debug print
        map_id = (request.map_id, 0)
        try:
            sql_get_data_list = f"select id, frame_id, point_list from {request.data_type} where map_id=? and creator_id=?"
            result = self.conn.execSQL(sql_get_data_list, map_id)
            response.success = True
            if len(result) > 0:
                # print("get_data_list ok") # debug print
                response.message = json.dumps(result)
            else:
                #print("get_data_list no data") # debug print
                response.message = json.dumps([])
        except Exception as e:
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
            #print("get_data_list error") # debug print
        return response

    def get_forbidden_callback(self, request, response):
        data_id = (request.map_id,)
        sql_get_data = f"select id, frame_id, point_list from forbidden where map_id=?"
        print(f"get_forbidden_callback cmd: {sql_get_data}")
        try:
            result = self.conn.execSQL(sql_get_data, data_id)
            response.success = True
            if len(result) > 0:
                print(f"get_forbidden_callback ok {result} type of result {type(result)}")
                for forbidden_item in result:
                    # 防止地图的禁止线数据又错误的空字符串的情况，所以增加一个检查，避免在转换json对象时出错导致失败
                    if len(forbidden_item['point_list']) == 0:
                        continue
                    pl = json.loads(forbidden_item['point_list'])
                    print(f"pl is {pl}")

                    if isinstance(pl, list) and len(pl):
                        response.lines = self.__point_list_to_forbiddenlines(pl)
                        response.message = "success"
                        print("get_forbidden_callback done")
                        break
            else:
                print("get_forbidden_callback no data") # debug print
                response.message = "get_forbidden_callback no data"  # "forbidden not found"
        except Exception as e:
            print("get_forbidden_callback error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error" + str(e)

        return response

    def get_current_forbidden(self, request, response):
        try:
            sql_get_current_map = "select id from current_map"
            result = self.conn.execSQL(sql_get_current_map)
            if len(result) > 0:
                map_id = (result[0].get("id", 0), )
                sql_get_current_forbidden = "select id, frame_id, point_list from forbidden where map_id=?"
                print(f"get_current_forbidden_callback cmd: {sql_get_current_forbidden}, map_id={map_id}")

                result = self.conn.execSQL(sql_get_current_forbidden, map_id)
                response.success = len(result) > 0
                if response.success:
                    print(f"get_current_forbidden_callback ok {result} type of result {type(result)}")
                    for forbidden_item in result:
                        # 防止地图的禁止线数据又错误的空字符串的情况，所以增加一个检查，避免在转换json对象时出错导致失败
                        if len(forbidden_item['point_list']) == 0:
                            continue
                        pl = json.loads(forbidden_item['point_list'])
                        print(f"pl is {pl}")

                        if isinstance(pl, list) and len(pl):
                            response.message = self.__point_list_to_forbiddenlines(pl)
                            print("get_forbidden_callback done")
                            break
                else:
                    print("get_current_forbidden_callback no data") # debug print
                    response.message = []  # "forbidden not found"
            else:
                raise Exception("current map not set")

        except Exception as e:
            print("get_current_forbidden error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)
            response.success = False
            response.message = []

        return response

    def get_point_callback(self, request, response):
        #print("get_point in") # debug print
        data_id = (request.id,)
        sql_get_data = f"select id, frame_id, point_list from {request.data_type} where id=?"
        try:
            result = self.conn.execSQL(sql_get_data, data_id)
            response.success = len(result) > 0
            if response.success:
                # print("get_point ok") # debug print
                _ = result[0]
                msg = AidPose()
                msg.header.frame_id = _["frame_id"]
                msg.header.stamp = self.get_clock().now().to_msg()
                point_list = json.loads(_["point_list"])
                position = point_list["position"]
                msg.pose.position.x = float(position["x"])
                msg.pose.position.y = float(position["y"])
                msg.pose.position.z = float(position["z"])
                orientation = point_list["orientation"]
                msg.pose.orientation.x = float(orientation["x"])
                msg.pose.orientation.y = float(orientation["y"])
                msg.pose.orientation.z = float(orientation["z"])
                msg.pose.orientation.w = float(orientation["w"])
                msg.name = str(point_list["name"])
                response.message = msg
            else:
                #print("get_point no point") # debug print
                response.message = AidPose()  # "waypoint not found"
        except Exception as e:
            #print("get_point error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = AidPose()

        return response
    
    def update_point_callback(self, request, response):
        #print("update_data in") # debug print
        try:
            sql_get_data = f"select 1 from {request.data_type} where id=?"
            result = self.conn.execSQL(sql_get_data, (request.id,))
            response.success = len(result) > 0
            if response.success:
                data = json.loads(request.data)
                sql_keys = ["edit_timestamp=?,point_list=?"]
                sql_values = [int(time.time())]
                sql_values.append(request.data)
                sql_values.append(request.id)  # id 这个一定要放在更新值列表之后因为是最后作为 where 子句的条件存在
                print(f"sql_values {sql_values}")
                update_data_str = ",".join(sql_keys)  # 将字段更新列表拼接成字符串
                sql_update_data = f"update {request.data_type} set {update_data_str} where id=?"
                print(f"sql_update_data {sql_update_data}")

                self.conn.execSQL(sql_update_data, sql_values)
                response.message = "ok"
                #print("update_data ok") # debug print
            else:
                #print("update_data no data") # debug print
                response.message = "data not found"
        except Exception as e:
            #print("update_data error") # debug print
            # todo: 记录详细错误信息到日志
            print(e)

            response.success = False
            response.message = "error"
        return response

    def run(self):
        try:
            rclpy.spin(self)
        except KeyboardInterrupt:
            pass
        finally:
            rclpy.shutdown()

    def __map_pgm_to_grid(self, map_file_path: str, msg: OccupancyGrid):
        with open(map_file_path, "r", encoding="utf-8") as f:
            map_yaml_data = yaml.load(f, Loader=yaml.FullLoader)

        print(f"map_yaml_data\n{map_yaml_data}")

        map_img_file_path = map_yaml_data.get("image", "")
        if not os.path.isabs(map_img_file_path):
            map_img_file_path = os.path.join(os.path.dirname(map_file_path), map_img_file_path)
        print(f"abs map_img_file_path: {map_img_file_path}")
        # 读取pgm文件
        np_array2d = cv2.imread(map_img_file_path, cv2.IMREAD_UNCHANGED)
        if np_array2d is None:
            raise FileNotFoundError(f"cannot read map image: {map_img_file_path}")

        # nav_msgs map_server normally saves an 8-bit grayscale PGM. Keep
        # compatibility with RGB/BGRA inputs used by older maps as well.
        if np_array2d.ndim == 2:
            img = cv2.cvtColor(np_array2d, cv2.COLOR_GRAY2BGR)
        elif np_array2d.shape[2] == 4:
            img = cv2.cvtColor(np_array2d, cv2.COLOR_BGRA2BGR)
        else:
            img = np_array2d.copy()
        

        img[np.where((img == [205, 205, 205]).all(axis=2))] = [170, 108, 82]
        img[np.where((img == [254, 254, 254]).all(axis=2))] = [200, 145, 127]

        # Convert image to the JPEG format declared by the web client.
        retval, buffer = cv2.imencode(".jpg", img)
        if not retval:
            raise RuntimeError("JPEG encoding failed")
        jpeg_data = buffer.tobytes()

        # Convert JPEG data to base64 format
        base64_data = base64.b64encode(jpeg_data)

        # Convert base64_data to sequence of int8
        int_array = np.frombuffer(base64_data, dtype=np.uint8)

        # Create new OccupancyGrid message
        msg.header.frame_id = map_yaml_data.get("frame_id", "map")
        msg.header.stamp = self.get_clock().now().to_msg()
        info = MapMetaData()
        info.map_load_time = msg.header.stamp
        info.resolution = map_yaml_data.get("resolution", 0.0)
        if len(img.shape) == 3:
            height, width, channels = img.shape
        else:
            height, width = img.shape
        info.width = width
        info.height = height
        origin = Pose()
        orientation = Quaternion()
        orientation.w = float(math.cos(map_yaml_data.get("origin")[2] / 2))
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = float(math.sin(map_yaml_data.get("origin")[2] / 2))
        origin.orientation = orientation
        position = Point()
        position.x = float(map_yaml_data.get("origin")[0])
        position.y = float(map_yaml_data.get("origin")[1])
        # position.z = map_yaml_data.get("origin")[2] # 暂时写死为0.0
        position.z = 0.0
        origin.position = position
        info.origin = origin
        msg.info = info
        # Convert int8 numpy array to Python list
        msg.data = int_array.tolist()
        return msg

    def __point_list_to_path(self, frame_id: str, point_list: list):
        msg = Path()
        msg.header.frame_id = frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        for point in point_list:
            pose = PoseStamped()
            position = point["position"]
            pose.pose.position.x = float(position["x"])
            pose.pose.position.y = float(position["y"])
            pose.pose.position.z = float(position["z"])
            orientation = point["orientation"]
            pose.pose.orientation.x = float(orientation["x"])
            pose.pose.orientation.y = float(orientation["y"])
            pose.pose.orientation.z = float(orientation["z"])
            pose.pose.orientation.w = float(orientation["w"])
            msg.poses.append(pose)
        return msg

    def __path_to_point_list(self, path: Path):
        point_list = []
        for pose in path.poses:
            point = {
                "position": {
                    "x": pose.pose.position.x,
                    "y": pose.pose.position.y,
                    "z": pose.pose.position.z
                },
                "orientation": {
                    "x": pose.pose.orientation.x,
                    "y": pose.pose.orientation.y,
                    "z": pose.pose.orientation.z,
                    "w": pose.pose.orientation.w
                }
            }
            point_list.append(point)
        return point_list

    def __point_list_to_forbiddenlines(self, point_list: list):
        msg = []
        for point_pair in point_list:
            forbiddenline = ForbiddenLine()
            start = point_pair["start"]
            end = point_pair["end"]
            forbiddenline.start.x = float(start['x'])
            forbiddenline.start.y = float(start['y'])
            forbiddenline.start.z = float(start['z'])
            forbiddenline.end.x = float(end['x'])
            forbiddenline.end.y = float(end['y'])
            forbiddenline.end.z = float(end['z'])
            msg.append(forbiddenline)
        return msg

    def __forbiddenlines_to_point_list(self, forbiddenlines: list):
        point_list = []
        for forbiddenline in forbiddenlines:
            item = {
                "start": {"x": 0.0, "y":0.0, "z":0.0},
                "end": {"x": 0.0, "y":0.0, "z":0.0}
            }
            item["start"]["x"] = float(forbiddenline.start.x)
            item["start"]["y"] = float(forbiddenline.start.y)
            item["start"]["z"] = float(forbiddenline.start.z)
            item["end"]["x"] = float(forbiddenline.end.x)
            item["end"]["y"] = float(forbiddenline.end.y)
            item["end"]["z"] = float(forbiddenline.end.z)
            point_list.append(item)
        return point_list


def main():
    rclpy.init()
    node = MapManagerNode()
    node.run()


if __name__ == "__main__":
    main()
