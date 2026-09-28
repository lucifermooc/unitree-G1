<template>
  <div class="newMapBox">
    <ShowMap :initData="initData" />
    <div class="right point">
      <div class="step1" v-if="step == 1">
        <div class="rText" v-if="!$store.state.patrol_arr.length">
          <p>选择位置点</p>
          <p>请依次在地图中点击想巡逻的位置点</p>
        </div>
        <div class="titleBox" v-else>
          <p>选择位置点:</p>
          <div class="mapNameB">
            <p
              v-for="(e, i) in $store.state.patrol_arr"
              :key="i"
              class="mapName"
              @click="onDelOne(i)"
            >
              ({{ e.x.toFixed(2) }},{{ e.y.toFixed(2) }})
            </p>
          </div>
        </div>
        <div
          class="goPoint"
          :class="{ goPointSure: $store.state.patrol_arr.length }"
          @click="onSave()"
        >
          保存路线
        </div>
      </div>
      <div class="step2" v-if="step == 2">
        <div>
          <div class="goPoint stop" @click="editPatrolFun()">编辑路线</div>
          <div class="goPoint delList" @click="onDelList()">清除路线</div>
        </div>
        <div class="goPoint stop" @click="onStop()" v-if="['patrolStart','patrolPause'].includes(actionStatus)">
          关闭巡逻
        </div>
        <div class="goPoint action" @click="onBegin()" v-else>
          开启巡逻
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/pointMap";
import { mapState, mapMutations } from "vuex";
import fullscreenLoading from "@/components/fullscreenLoading.js";

export default {
  components: {
    ShowMap
  },
  computed: {
    ...mapState(["actionStatus"])
  },
  data() {
    return {
      text: "选择位置",
      hasHistory: false,
      step: 1,
      initData: false,
      patrolId: 1,
    };
  },
  mounted() {
    console.log('---- patrol mounted ------', new Date().toLocaleString())
    let loading = fullscreenLoading();
    this.$store.state.hasSave = false;
    this.$store.state.patrol_arr = [];
    this.$store.state.patrol_arr_area = [];
    try {
      this.getPoint();
    } catch (error) {
      console.log("[ error ]-45", error);
    }
    // 状态机
    const type = new ROSLIB.ServiceRequest({
      action: "patrol"
    });
    robotMode.callService(
      type,
      result => {
        console.log("[ robotMode OK]-61", result);
        loading.close();
        if (result.message !== "ok") {
          this.$message("状态切换失败");
        }
      },
      result => {
        loading.close();
        this.$message("状态切换失败");
        console.log("[ robotMode ERR]-61", result);
      }
    );
  },
  methods: {
    editPatrolFun() {
      this.$store.state.tool = "patrol";
      this.step = 1;
    },
    addPoint(data) {
      const data_ = data.map(e => {
        return { x: e.x, y: e.y, z: 0 };
      });
      const msg2 = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id,
        data: JSON.stringify(data_),
        frame_id: "map",
        data_type: "waypoint"
      });
      OperationAdd.callService(
        msg2,
        result => {
          console.log("[  addPoint OK]-61", result);
          this.getPoint(); // 更新 patrolId和 hasHistory 的数据
        },
        result => {
          console.log("[  addPoint ERR]-61", result);
        }
      );
    },
    updatePoint(data) {
      const data_ = data.map(e => {
        return { x: e.x, y: e.y, z: 0 };
      });
      const msg2 = new ROSLIB.ServiceRequest({
        id: this.patrolId,
        data: JSON.stringify({
          map_id: this.$store.state.nowMap.id,
          frame_id: "map",
          point_list: JSON.stringify(data_)
        }),
        data_type: "waypoint"
      });
      OperationUpdate.callService(
        msg2,
        result => {
          console.log("[  updatePoint OK]-61", result);
        },
        result => {
          console.log("[  updatePoint ERR]-61", result);
        }
      );
    },
    getPoint() {
      const msg2 = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id,
        data_type: "waypoint"
      });
      getMapLinkedDataList.callService(
        msg2,
        result => {
          console.log("[  getPoint result]-61", result);
          if (result.success) {
            let msg = [];
            try {
              msg = JSON.parse(result.message);
              this.initData = true;
              this.patrolId = msg[0].id;
              this.$store.state.patrol_arr = JSON.parse(msg[0].point_list);
              this.text = "开始巡逻";
              this.hasHistory = true;
              this.step = 2;
            } catch (error) {
              this.$message("还未设置巡逻点");
            }
            console.log(this.$store.state.patrol_arr);
          } else {
            this.$message("获取巡逻点失败");
          }
        },
        result => {
          console.log("[  getPoint ERR]-61", result);
          this.$message("获取巡逻点失败");
        }
      );
    },
    onDelOne(i) {
      this.$store.state.patrol_arr.splice(i, 1);
      this.$store.state.patrol_arr_area.splice(i, 1);
    },
    // 取消巡逻，保存空点位数据
    onDelList() {
      if(this.$store.getters.isPatrolAction) {
        const type = new ROSLIB.ServiceRequest({
          cmd: "cancel"
        });
        patrolState.callService(
            type,
            res => {
              console.log("[ patrol_control ok]-61", res);
            },
            res => {
              console.log("[ patrol_control ERR]-61", res);
            }
        );
      }
      const msg = new ROSLIB.ServiceRequest({
        id: this.patrolId,
        data_type: "waypoint"
      });
      OperationDelete.callService(
          msg,
          result => {
            console.log("[  deletePoint OK]-61", result);
            this.step = 1;
            this.patrolId = undefined;
            this.$store.state.patrol_arr = [];
            this.$store.state.patrol_arr_area = [];
            this.hasHistory = false;
            this.$message("已清除路线，请重新选择位置点");
          },
          result => {
            console.log("[  deletePoint ERR]-61", result);
          }
      )
    },
    onSave() {
      if (!this.$store.state.patrol_arr.length) {
        return false;
      }
      if (this.hasHistory) {
        this.updatePoint(this.$store.state.patrol_arr);
      } else {
        this.addPoint(this.$store.state.patrol_arr);
      }
      this.step = 2;
    },
    onBegin() {
      if(this.$store.getters.actionNeedCancel) {
        // 先取消上次的巡逻或导航，在开始本次的巡逻
        console.log('---- patrol click cancel ------', new Date().toLocaleString())
        const type = new ROSLIB.ServiceRequest({
          cmd: "cancel"
        });
        console.log('---- patrol click cancel send ------', new Date().toLocaleString())
        patrolState.callService(
          type,
          res => {
            this.startPatrol();
            console.log("[ patrol_control ok]-61", res);
          },
          res => {
            console.log("[ patrol_control ERR]-61", res);
          }
        );
      } else {
        this.startPatrol();
      }
    },
    startPatrol() {
      console.log('---- patrol click start ------', new Date().toLocaleString())
      const point = { poses: [] };
      let mapPoint = this.$store.state.patrol_arr;
      mapPoint.map(e => {
        point.poses.push({
          header: {
            stamp: {
              sec: 0,
              nanosec: 0
            },
            frame_id: "map"
          },
          pose: {
            position: {
              x: e.x,
              y: e.y,
              z: 0.0
            },
            orientation: {
              x: 0.0,
              y: 0.0,
              z: 0.0,
              w: 1.0
            }
          }
        });
      });
      const msg = new ROSLIB.Message(point);
      console.log('---- patrol click start send ------', new Date().toLocaleString())
      TalkerPoint.publish(msg);
    },
    onStop() {
      const tag = "[patrol stop]";
      const t0 = Date.now();
      const socketState = ros.socket ? ros.socket.readyState : null; // 0 CONNECTING 1 OPEN 2 CLOSING 3 CLOSED
      console.log(tag, "click", JSON.stringify({
        time: new Date().toLocaleString(),
        isConnected: ros.isConnected,
        socketState,
        rosURL: typeof getRosURL === "function" ? getRosURL() : ""
      }));

      // 临时钩子：确认消息真正写入 WebSocket（比 callService 返回更能证明“已发出”）
      const sock = ros.socket;
      let wsHooked = false;
      let originalSend = null;
      if (sock && typeof sock.send === "function") {
        wsHooked = true;
        originalSend = sock.send.bind(sock);
        sock.send = function(data) {
          try {
            const raw = typeof data === "string" ? data : "";
            if (raw.indexOf("/patrol_control") !== -1 && raw.indexOf("call_service") !== -1) {
              console.log(tag, "WS ACTUALLY SENT", Date.now() - t0, "ms", raw);
            }
          } catch (e) {
            /* ignore */
          }
          return originalSend(data);
        };
      }

      const restoreWsHook = () => {
        if (wsHooked && sock && originalSend) {
          sock.send = originalSend;
          wsHooked = false;
        }
      };

      const type = new ROSLIB.ServiceRequest({
        cmd: "cancel"
      });
      let responded = false;
      const timeoutMs = 60*1000;
      const timer = setTimeout(() => {
        if (responded) return;
        restoreWsHook();
        console.log(tag, "TIMEOUT no response", JSON.stringify({
          elapsed: Date.now() - t0,
          isConnected: ros.isConnected,
          socketState: ros.socket ? ros.socket.readyState : null,
          hint: !ros.isConnected
            ? "未连接：请求可能还在 roslib 队列里，根本没发出去"
            : "已连接：请求大概率已发出，但 /patrol_control 一直无回包"
        }));
      }, timeoutMs);

      console.log(tag, "callService invoke", JSON.stringify(type));
      patrolState.callService(
        type,
        res => {
          responded = true;
          clearTimeout(timer);
          restoreWsHook();
          console.log(tag, "response OK", Date.now() - t0, "ms", JSON.stringify(res));
        },
        err => {
          responded = true;
          clearTimeout(timer);
          restoreWsHook();
          console.log(tag, "response ERR", Date.now() - t0, "ms", JSON.stringify(err));
        }
      );
      // callService 是同步把消息交给底层；未连接时只会排队，不会立刻失败
      console.log(tag, "callService returned", JSON.stringify({
        isConnected: ros.isConnected,
        note: ros.isConnected
          ? "已连接，随后应出现 WS ACTUALLY SENT；若只有 TIMEOUT 则是无回包"
          : "未连接，不会马上发，等重连；若一直断线则只有 TIMEOUT"
      }));
    },
    onBegin2() {
      if (this.text === "选择位置") {
        this.$store.state.tool = "patrol";
        this.text = "开始巡逻";
      } else if (this.text === "开始巡逻") {
        if (this.$store.state.patrol_arr.length < 2) {
          return false;
        }
        if (this.hasHistory) {
          this.updatePoint(this.$store.state.patrol_arr);
        } else {
          this.addPoint(this.$store.state.patrol_arr);
        }
        const point = { poses: [] };
        this.$store.state.patrol_arr.map(e => {
          point.poses.push({
            header: {
              stamp: {
                sec: 0,
                nanosec: 0
              },
              frame_id: "map"
            },
            pose: {
              position: {
                x: e.x,
                y: e.y,
                z: 0.0
              },
              orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
              }
            }
          });
        });
        const msg = new ROSLIB.Message(point);
        TalkerPoint.publish(msg);
        this.text = "暂停巡逻";
      } else if (this.text === "暂停巡逻") {
        const type = new ROSLIB.ServiceRequest({
          cmd: "pause"
        });
        patrolState.callService(
          type,
          res => {
            console.log("[ patrol_control ok]-61", res);
          },
          res => {
            console.log("[ patrol_control ERR]-61", res);
          }
        );
        this.text = "恢复巡逻";
      } else if (this.text === "恢复巡逻") {
        const type = new ROSLIB.ServiceRequest({
          cmd: "resume"
        });
        patrolState.callService(
          type,
          res => {
            console.log("[ patrol_control ok]-61", res);
          },
          res => {
            console.log("[ patrol_control ERR]-61", res);
          }
        );
        this.text = "暂停巡逻";
      }
    }
  }
};
</script>

<style lang="less" scoped>
.newMapBox {
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
}

.right {
  width: 434px;
  height: calc(100% - 70px);
  background-color: #ccc;
  border-radius: 5px;
  background: linear-gradient(
    155deg,
    rgba(71, 84, 141, 0.64) 24%,
    rgba(71, 66, 124, 0.52) 98%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  flex-direction: column;
  align-items: center;
  line-height: 50px;
  margin-left: 30px;
  margin-top: 30px;
  justify-content: space-between;
  padding: 20px;
  box-sizing: border-box;

  .rText {
    display: flex;
    align-items: center;
    flex-direction: column;
    padding: 20px;
  }

  & > p {
    text-align: center;
    text-align: center;
    line-height: 50px;
    padding: 0px 40px;
  }

  & > div {
    display: flex;
    flex-direction: column;
    align-items: center;
    justify-content: space-between;
    height: 100%;
  }
}

.titleBox {
  display: flex;
  flex-direction: column;
  justify-content: space-between;

  .mapNameB {
    height: 640px;
    overflow: auto;
    width: 100%;
  }

  .mapName {
    width: 329px;
    height: 80px;
    border-radius: 10px;
    opacity: 1;
    background: #2f3758;
    backdrop-filter: blur(10px);
    display: flex;
    align-items: center;
    justify-content: space-around;
    margin-top: 20px;
    padding: 10px 0;
  }
}

.goPoint {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  background: #4f5478;
  opacity: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.goPointSure {
  background: #b04cf3;
}

.delList {
  background: #d94040;
  margin-top: 30px;
}

.action {
  width: 300px;
  height: 120px;
  background: linear-gradient(
    110deg,
    rgba(55, 89, 238, 0.64) 11%,
    rgba(30, 157, 244, 0.37) 89%
  );
}

.stop {
  width: 300px;
  height: 120px;
  background: linear-gradient(110deg, #596ab5 11%, rgba(66, 82, 146, 0.53) 89%);
}

.readyBtn {
  background: linear-gradient(
    110deg,
    rgba(55, 89, 238, 0.64) 11%,
    rgba(30, 157, 244, 0.37) 89%
  ) !important;
}
</style>
