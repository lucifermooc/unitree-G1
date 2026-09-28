import Vue from 'vue'
import Vuex from 'vuex'
Vue.use(Vuex)
const store = new Vuex.Store({
  state: {
    mcode: '', //操作选择
    rubber_data1: false, //不规则区域
    rubber_data2: false, //选择橡皮檫
    rubber_size: 20, //橡皮檫大小
    stop_point: 0, //禁行区计数点
    head_h: 150, //工具区域
    zero: 0, //不规则区域点计数

    tool: '',//选取的工具
    x_can: null, //canvas
    _map_name: '', //地图名字
    map_width: null,//发送给后台将获取的地图宽
    m_width: 992,//地图IMG宽
    m_height: 992,//地图IMG高
    m_resolution: 0.05, //地图分辨率
    m_position:{},//地图原点
    m_resolution2:0,//地图物理分辨率
    charge_po_back:null,//后台传来充电桩坐标
    set_state:null,//地图操作成功与否
    init_img_data: true,//加载地图是否初始化
    right_top_save:false,//是否为左上保存
    save_map:false,//保存地图
    txt:'等待中...',
    loading_build: false, //构建地图等待
    loading_dev:false,//编辑地图等待
    tool_active:0,//工具栏选项
    prepro_val:0,//孤立点大小值
    charge_po:[],//充电桩位置
    robotPoint:{x:0,y:0},
    robotOrientation:{x:0,y:0,z:0,w:1},
    robotYaw: 0,
    mapSrc: '',
    help: false,//帮助栏开关
    map_img_w:0,//屏幕地图宽
    set_msg:null,//接收錯誤消息
    patrol_chang_data:[],
    // 巡逻和导航的 actionStatus 都不要手动改变，而是从 changeRobotTaskStatus 中接口获取状态
    actionStatus:'',//前端操作状态point/patrolStart(巡逻中)/patrolPause(暂停巡逻)/navigateStart(导航中)/navigatePause(暂停导航)/remote_control/localization/newMap/
    linearCurveArr:[],//禁行线点位
    linearCurveArrP:[],//禁行线点位
    eraserArr:[],//橡皮擦点位
    eraserArrP:[],//橡皮擦点位
    navigationMapPoints: [],//定点导航的点位(地图所用点位) {x,y,name,id,deg,orientation}
    navigationImgPoints:[],//定点导航的点位(UI展示所用点位)
    mapData:{
      src: "",
      width: 1930,
      height: 3909,
      resolution:0.05000000074505806,
      positionX: 0,
      positionY: 0,
    },//地图数据
    showMsg:false,//控制msg提示框显示
    hasSave:true,//顶部路由=》页面保存提示控制
    nowMap:{id:"",name:''},
    patrol_arr_area:[],//图像坐标巡逻点
    patrol_arr: [],//地图坐标巡逻点
    IP:'',//ros链接ip
    percentage: undefined,

    // 基于 charge 的充电状态（带防抖），charge > 0.1 视为充电
    batteryChargeState: {
      isCharging: false,
      initialized: false,
      candidate: null,
      count: 0,
    },
  },
  getters: {
    mcode: state => state.mcode,
    // 巡逻和导航中需要先cancel才能执行下一个操作的状态
    actionNeedCancel: state => ['patrolStart', 'patrolPause', 'navigateStart', 'navigatePause'].includes(state.actionStatus),
    // 是否是巡逻的状态
    isPatrolAction: state => ['patrolStart', 'patrolPause'].includes(state.actionStatus),
    // 是否是导航的状态
    isNavigateAction: state => ['navigateStart', 'navigatePause'].includes(state.actionStatus),
  },
  mutations: {
    chang_data(state, n) {
      state.mcode = n;
    },
    rubber_chang_data1(state, e) {//构建修改值得方法
      e != undefined ? state.rubber_data1 = e : state.rubber_data1 = !state.rubber_data1;
    },
    resetNavigationMapPoints(state) {
      state.navigationMapPoints = [];
      state.navigationImgPoints = [];
    },
    resetLinearCurve(state) {
      state.linearCurveArr = [];
      state.linearCurveArrP = [];
    },
    changeRobotTaskStatus(state, payload) {
      // status int32 任务状态：0 - idle（空闲）
      //  1 - working（执行中）
      //  2 - success（成功完成）
      //  3 - failed（失败）
      //  4 - suspend（暂停）
      //  5 - cancel（取消）
      // task_type int32 任务类型： 0 - 单点导航
      // 1 - 巡逻

      // `{task_type}-{status}` 对应的 actionStatus
      const status = {
        '0-1': 'navigateStart',
        '0-4': 'navigatePause',
        '1-1': 'patrolStart',
        '1-4': 'patrolPause',
      }
      state.actionStatus = status[`${payload.task_type}-${payload.status}`] || ''
    },

    updateRobotPose(state, payload) {
      const pose = payload
      const position = pose.position || {}
      const orientation = pose.orientation || {}

      state.robotPoint = {
        x: Number(position.x || 0),
        y: Number(position.y || 0)
      }
      state.robotOrientation = orientation
    },

    // 充电状态:只要charge大于0.1就是充电。初始状态时，就拿第一次判断大于0.1就是充电，否则就是没充电，后续有变动时就连续5次消息都是相同大于或小于等于才改变状态，如果5次内又有大于又有小于等于，就认为是数据跳动，就不管，只有连续的判定就认为改变状态成功
    updateBatteryCharge(state, charge) {
      const s = state.batteryChargeState
      const charging = charge > 0.1

      if (!s.initialized) {
        // 初始状态：第一次直接判定
        s.isCharging = charging
        s.initialized = true
        s.candidate = charging
        s.count = 1
        return
      }

      if (charging === s.candidate) {
        // 候选值等于当前已确认状态，无需计数
        if (charging === s.isCharging) {
          return
        }
        s.count++
        // 连续 5 次相同判定，确认状态变更
        if (s.count >= 5) {
          s.isCharging = charging
        }
      } else {
        // 数据跳动：与上一次候选值不同，重置计数
        s.candidate = charging
        s.count = 1
      }
    }
  }
})
export default store
