const ros = new ROSLIB.Ros();
const rosIP = 'localhost';
const getRosURL = () => {
  const storageIP = localStorage.getItem('rosIP');
  const ip = storageIP || rosIP;
  return `ws://${ip}:9090`
}
// const rosURL = 'ws://localhost:9090';
// const rosURL = 'ws://192.168.120.201:9090'

/* ros 的 connect 连接逻辑移动到 headArea.vue 组件中进行 */

/** 模式切换 */
const robotMode = new ROSLIB.Service({
  ros: ros,
  name: '/mode_set',
  serviceType: 'aid_robot_msgs/srv/StatusChange'
});

/** 指令控制 */
const controlRobot = new ROSLIB.Topic({
  ros: ros,
  name: '/cmd_vel_remote_ctrl',
  messageType: 'geometry_msgs/msg/Twist'
});

/** 建图订阅 */
const robotMap = new ROSLIB.Topic({
  ros: ros,
  name: '/map_base64',
  messageType: 'nav_msgs/msg/OccupancyGrid'
});
/** 建图结束 */
const finishMap = new ROSLIB.Service({
  ros: ros,
  name: '/finish_trajectory',
  serviceType: 'cartographer_ros_msgs/srv/FinishTrajectory'
});
/** 保存建图文件 */
const saveMapFile = new ROSLIB.Service({
  ros: ros,
  name: '/write_state',
  serviceType: 'cartographer_ros_msgs/srv/WriteState'
});
/** 建图集成保存 */
const saveMap = new ROSLIB.Service({
  ros: ros,
  name: '/aid_save_map',
  serviceType: 'aid_robot_msgs/srv/MapOperation'
});
/** 保存建图db */
const saveMapDb = new ROSLIB.Service({
  ros: ros,
  name: '/add_map',
  serviceType: 'aid_robot_msgs/srv/MapOperationAdd'
});
/** 获取地图列表 */
const getMapList = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_list',
  serviceType: 'aid_robot_msgs/srv/MapList'
});
/** 删除地图 */
const deleteMap = new ROSLIB.Service({
  ros: ros,
  name: '/delete_map',
  serviceType: 'aid_robot_msgs/srv/OperationDelete'
});
/** 更新地图 */
const updateMap = new ROSLIB.Service({
  ros: ros,
  name: '/update_map',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/** 获取地图数据 */
const getMapImage = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_image',
  serviceType: 'aid_robot_msgs/srv/MapImage'
});
/** 设置当前地图为使用中的地图 */
const setCurrentMapId = new ROSLIB.Service({
  ros: ros,
  name: '/set_current_map_id',
  serviceType: 'aid_robot_msgs/srv/SetCurrentMap'
});
/** 获取当前使用中的地图id */
const getCurrentMapId = new ROSLIB.Service({
  ros: ros,
  name: '/get_current_map_id',
  serviceType: 'aid_robot_msgs/srv/GetCurrentMap'
});
/** 机器人在地图中位置*/
const robotPosition = new ROSLIB.Topic({
  ros: ros,
  name: '/base_link_pose',
  messageType: 'geometry_msgs/msg/PoseStamped',
  throttle_rate: 500,  // 限速，单位为毫秒，表示每500ms接收一次消息
  queue_length: 1      // 队列只保留最新一条，丢弃积压消息
});
/** 重定位*/
const PoseStamped = new ROSLIB.Topic({
  ros: ros,
  name: '/goal_pose_',
  messageType: 'geometry_msgs/msg/PoseStamped'
});

/** 激光扫描 */
const RobotScan = new ROSLIB.Topic({
  ros: ros,
  name: '/scan',
  messageType: 'sensor_msgs/msg/LaserScan'
});
/** TF 动态变换 */
const RobotTF = new ROSLIB.Topic({
  ros: ros,
  name: '/tf',
  messageType: 'tf2_msgs/msg/TFMessage'
});
/** TF 静态变换 */
const RobotTFStatic = new ROSLIB.Topic({
  ros: ros,
  name: '/tf_static',
  messageType: 'tf2_msgs/msg/TFMessage'
});
/** 重定位*/
const InitialPose = new ROSLIB.Topic({
  ros: ros,
  name: '/initialpose',
  messageType: 'geometry_msgs/msg/PoseWithCovarianceStamped'
});
/*导航规划路径*/
const NavigationPlan = new ROSLIB.Topic({
  ros: ros,
  name: '/plan',
  messageType: 'nav_msgs/msg/Path'
})
/** 获取巡逻点列表 */
const getMapLinkedDataList = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_waypoint_list',
  serviceType: 'aid_robot_msgs/srv/MapLinkedDataList'
});
/** 新增巡逻 */
const OperationAdd = new ROSLIB.Service({
  ros: ros,
  name: '/add_waypoint',
  serviceType: 'aid_robot_msgs/srv/OperationAdd'
});
/** 更新巡逻点 */
const OperationUpdate = new ROSLIB.Service({
  ros: ros,
  name: '/update_waypoint',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/** 清除巡逻路线 */
const OperationDelete = new ROSLIB.Service({
  ros: ros,
  name: '/delete_waypoint',
  serviceType: 'aid_robot_msgs/srv/OperationDelete'
});
/** 获取禁行线 */
const ForbiddenGet = new ROSLIB.Service({
  ros: ros,
  name: '/get_forbidden',
  serviceType: 'aid_robot_msgs/srv/ForbiddenGet'
});
/** 设置禁行线 */
const ForbiddenSet = new ROSLIB.Service({
  ros: ros,
  name: '/set_forbidden',
  serviceType: 'aid_robot_msgs/srv/ForbiddenSet'
});
/** 东方禁行线 */
const DrawPicture = new ROSLIB.Service({
  ros: ros,
  name: '/aid_draw_forbidden_line',
  serviceType: 'aid_robot_msgs/srv/DrawPicture'
});

/** 发送位置点*/
const TalkerPoint = new ROSLIB.Topic({
  ros: ros,
  name: '/patrol_path',
  messageType: 'nav_msgs/msg/Path'
});
/** 停止 */
const stopPatrol = new ROSLIB.Topic({
  ros: ros,
  name: '/stop_patrol',
  messageType: 'std_msgs/Empty'
});
/** 运动状态控制 */
const patrolState = new ROSLIB.Service({
  ros: ros,
  name: '/patrol_control',
  serviceType: 'aid_robot_msgs/srv/PatrolControl'
});
/*新增定点导航点位*/
const NavigationPointAdd = new ROSLIB.Service({
  ros: ros,
  name: '/add_point',
  serviceType: 'aid_robot_msgs/srv/OperationAdd'
});
/*修改定点导航点位*/
const NavigationPointUpdate = new ROSLIB.Service({
  ros: ros,
  name: '/update_point',
  serviceType: 'aid_robot_msgs/srv/OperationUpdate'
});
/*删除定点导航点位*/
const NavigationPointDelete = new ROSLIB.Service({
  ros: ros,
  name: '/delete_point',
  serviceType: 'aid_robot_msgs/srv/OperationDelete'
});
/*获取定点导航点位列表*/
const NavigationPointsGet = new ROSLIB.Service({
  ros: ros,
  name: '/get_map_point_list',
  serviceType: 'aid_robot_msgs/srv/MapLinkedDataList'
});
/*开始定点导航*/
const StartNavigation = new ROSLIB.Topic({
  ros: ros,
  name: '/nav_to_pose',
  messageType: 'geometry_msgs/msg/PoseStamped'
})
/*机器人任务状态*/
const RobotTaskStatus = new ROSLIB.Topic({
  ros: ros,
  name: '/task_status',
  messageType: 'aid_robot_msgs/msg/AidTaskStatus'
})



/** 页面初始化摄像头 */
const startCamera = new ROSLIB.Service({
  ros: ros,
  name: '/cam_start',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 页面停止摄像头 */
const stopCamera = new ROSLIB.Service({
  ros: ros,
  name: '/cam_stop',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 获取人物识别状态 */
const getFollowStatus = new ROSLIB.Topic({
  ros: ros,
  name: '/follow_status',
  messageType: 'std_msgs/msg/Bool'
})
/** 后置相机图像 */
const rearCameraImageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/rear_camera/image_raw/compressed',
  messageType: 'sensor_msgs/msg/CompressedImage',
  throttle_rate: 100, // 10fps
  queue_length: 1,
  compression: 'cbor'
});
/** 标签检测图像 */
const tagDetectionsImageTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/tag_detections_image/compressed',
  messageType: 'sensor_msgs/msg/CompressedImage',
  throttle_rate: 100, // 10fps
  queue_length: 1,
  compression: 'cbor'
});
/** 特征跟随-开始跟随 */
const startFollow = new ROSLIB.Service({
  ros: ros,
  name: '/follow_start',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 特征跟随-停止跟随 */
const stopFollow = new ROSLIB.Service({
  ros: ros,
  name: '/follow_stop',
  serviceType: 'aid_robot_msgs/srv/AICmd'
});
/** 获取ip */
const GetStrings = new ROSLIB.Service({
  ros: ros,
  name: '/get_ip_addresses',
  serviceType: 'aid_robot_msgs/srv/GetString'
});
/** 获取电量信息 */
const BatteryState = new ROSLIB.Topic({
  ros: ros,
  name: '/battery_state',
  serviceType: 'sensor_msgs/msg/BatteryState'
});
/** 编辑地图 */
const MapEditor = new ROSLIB.Service({
  ros: ros,
  name: '/map_editor',
  serviceType: 'aid_robot_msgs/srv/DrawPicture'
});

/** rgbd下标定服务 */
const DownCalibService = new ROSLIB.Service({
  ros: ros,
  name: '/calibrate_down_rgbd',
  serviceType: 'std_srvs/srv/Trigger'
});
/** rgbd上标定服务 */
const UpCalibService = new ROSLIB.Service({
  ros: ros,
  name: '/calibrate_up_rgbd',
  serviceType: 'std_srvs/srv/Trigger'
});
/* 上标定的状态 */
const rgbdCalibStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/rgbd_calib_status",
  messageType: "std_msgs/UInt8",
})


/**回充接口*/
const dockService = new ROSLIB.Service({
  ros: ros,
  name: "/cmd_dock",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**设置充电桩ID*/
const setPileIdService = new ROSLIB.Service({
  ros: ros,
  name: "/set_pile_id",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**设置充电桩通道*/
const setPileChannelService = new ROSLIB.Service({
  ros: ros,
  name: "/set_pile_channel",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**查询充电桩ID*/
const getPileIdService = new ROSLIB.Service({
  ros: ros,
  name: "/get_pile_id",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**获取充电桩位置*/
const getDockPoseService = new ROSLIB.Service({
  ros:ros,
  name: "/get_dock_pose",
  serviceType: "aid_robot_msgs/srv/GetDockPose",
});
// 充电状态监听
const dockStateTopic = new ROSLIB.Topic({
  ros:ros,
  name: "/dock_state",
  messageType: "std_msgs/msg/String",
  throttle_rate: 1000,
  queue_length: 1
});
/**充电结果订阅*/
const dockResultTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/dock_result",
  messageType: "std_msgs/msg/String",
})

/**获取嵌入版本(mcu/electrical)*/
const getEmbeddedVersionService = new ROSLIB.Service({
  ros:ros,
  name: "/get_embedded_version",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**获取导航版本*/
const getReleaseVersionService = new ROSLIB.Service({
  ros:ros,
  name: "/get_release_info",
  serviceType: "std_srvs/srv/Trigger",
});
/**标签检测开关*/
const enableDetectionService = new ROSLIB.Service({
  ros: ros,
  name: "/enable_detection",
  serviceType: "std_srvs/srv/SetBool",
});
/**老化测试-开始机械臂测试*/
const startArmTest = new ROSLIB.Service({
  ros: ros,
  name: "/arm_test/start_system",
  serviceType: "aid_robot_msgs/srv/AICmd",
});
/**老化测试-停止机械臂测试*/
const stopArmTest = new ROSLIB.Service({
  ros: ros,
  name: "/arm_test/stop_system",
  serviceType: "aid_robot_msgs/srv/AICmd",
});
/**老化测试-继续老化*/
const resumeAgingService = new ROSLIB.Service({
  ros: ros,
  name: "/resume_aging",
  serviceType: "std_srvs/srv/Trigger",
});
/**老化测试-停止老化*/
const stopAgingService = new ROSLIB.Service({
  ros: ros,
  name: "/stop_aging",
  serviceType: "std_srvs/srv/Trigger",
});
/**老化测试-状态订阅*/
const agingControlStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/aging_control_status",
  messageType: "aid_robot_msgs/msg/AidTaskStatus",
  compression: "cbor",
});
/**雷达标定-开始标定*/
const startRadarCalib = new ROSLIB.Service({
  ros: ros,
  name: "/start_calibration_odom_laser",
  serviceType: "aid_robot_msgs/srv/StartCalibration",
});
/**雷达标定-结束标定*/
const endRadarCalib = new ROSLIB.Service({
  ros: ros,
  name: "/end_calibration_odom_laser",
  serviceType: "aid_robot_msgs/srv/EndCalibration",
});
/** 设置电机模式 */
const setMotorMode = new ROSLIB.Service({
  ros: ros,
  name: "/set_motor_mode",
  serviceType: "aid_robot_msgs/srv/SetString",
});
/**雷达标定-状态订阅*/
const radarCalibStatusTopic = new ROSLIB.Topic({
  ros: ros,
  name: "/odom_laser_calibration_status",
  messageType: "aid_robot_msgs/msg/AidTaskStatus",
  compression: "cbor",
});
/**悬崖检测标定（点击一次标定一次，返回 true/false）*/
const cliffCalibService = new ROSLIB.Service({
  ros: ros,
  name: "/calibrate_cliff",
  serviceType: "std_srvs/srv/Trigger",
});
