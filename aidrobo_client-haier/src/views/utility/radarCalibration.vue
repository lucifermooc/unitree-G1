<template>
  <div class="aging-container">
    <div class="right">
      <div class="rText">
        <p class="title">雷达标定</p>
        <p class="description">{{ statusText }}</p>
      </div>

      <div class="btn-group">
        <div
          :class="['action', { disabled: isRunning || loading || !statusReceived }]"
          @click="handleStart"
        >
          开始标定
        </div>
        <div
          :class="['action', 'stop-btn', { disabled: !isRunning || loading || !statusReceived }]"
          @click="handleStop"
        >
          结束标定
        </div>
      </div>
    </div>
  </div>
</template>

<script>
/*
| status | 含义 |
| --- | --- |
| 0 | idle（空闲） |
| 1 | working（正在标定） |
| 2 | success（标定完成） |
| 3 | failed（标定失败） |
| 4 | suspend（暂停） |
| 5 | cancel（取消） |
* */
const RADAR_CALIB_STATUS_TEXT = {
  0: "点击「开始标定」启动雷达标定",
  1: "雷达标定运行中",
  2: "雷达标定完成",
  3: "雷达标定失败，请重新标定",
  4: "雷达标定已暂停",
  5: "雷达标定已取消"
};

export default {
  data() {
    return {
      calibrationStatus: 0,
      statusReceived: false,
      loading: false
    };
  },
  computed: {
    isRunning() {
      return this.calibrationStatus === 1;
    },
    statusText() {
      if (!this.statusReceived) return "正在获取雷达标定状态...";
      return RADAR_CALIB_STATUS_TEXT[this.calibrationStatus] || "雷达标定状态未知";
    }
  },
  mounted() {
    this.loading = false;
    radarCalibStatusTopic.subscribe(this.handleRadarCalibStatus);
  },
  beforeDestroy() {
    radarCalibStatusTopic.unsubscribe(this.handleRadarCalibStatus);
  },
  methods: {
    handleRadarCalibStatus(res) {
      const status = Number(res && res.status);
      if (Number.isNaN(status)) return;

      console.log("[radarCalib] status", res);
      this.statusReceived = true;
      this.calibrationStatus = status;
    },

    callServiceWithRetry(service, request) {
      return new Promise(resolve => {
        let retryCount = 0;
        const maxRetries = 5;

        const attempt = () => {
          console.log(`[radarCalib] call ${service.name}`, request);
          service.callService(
            request,
            result => {
              console.log(`[radarCalib] ${service.name} OK`, result);
              if (result && result.success === false) {
                retryCount++;
                if (retryCount >= maxRetries) {
                  resolve({
                    success: false,
                    message: result.message || "接口返回失败"
                  });
                } else {
                  attempt();
                }
                return;
              }
              resolve({
                success: true,
                message: result ? result.message : ""
              });
            },
            error => {
              console.log(`[radarCalib] ${service.name} ERR`, error);
              retryCount++;
              if (retryCount >= maxRetries) {
                resolve({
                  success: false,
                  message: (error && error.message) || error || "接口调用失败"
                });
              } else {
                attempt();
              }
            }
          );
        };

        attempt();
      });
    },

    showResult(result) {
      if (result.success) {
        this.$message.success(result.message || "操作成功");
      } else {
        this.$message.error(result.message || "操作失败");
      }
    },

    async handleStart() {
      if (this.isRunning || this.loading || !this.statusReceived) return;
      this.loading = true;

      // 第一步：/start_calibration_odom_laser
      const startReq = new ROSLIB.ServiceRequest({});
      const r1 = await this.callServiceWithRetry(startRadarCalib, startReq);
      if (!r1.success) {
        this.showResult(r1);
        this.loading = false;
        return;
      }

      // 第二步：/set_motor_mode → torque
      const modeReq = new ROSLIB.ServiceRequest({ data: "torque" });
      const r2 = await this.callServiceWithRetry(setMotorMode, modeReq);
      if (!r2.success) {
        this.showResult(r2);
        this.loading = false;
        return;
      }

      this.loading = false;
    },

    async handleStop() {
      if (!this.isRunning || this.loading || !this.statusReceived) return;
      this.loading = true;

      // 第一步：/end_calibration_odom_laser
      const endReq = new ROSLIB.ServiceRequest({});
      const r1 = await this.callServiceWithRetry(endRadarCalib, endReq);
      if (!r1.success) {
        this.showResult(r1);
        this.loading = false;
        return;
      }

      // 第二步：/set_motor_mode → velocity
      const modeReq = new ROSLIB.ServiceRequest({ data: "velocity" });
      const r2 = await this.callServiceWithRetry(setMotorMode, modeReq);
      if (!r2.success) {
        this.showResult(r2);
        this.loading = false;
        return;
      }

      this.loading = false;
      this.showResult(r1);
    }
  }
};
</script>

<style lang="less" scoped>
.aging-container {
  box-sizing: border-box;
  display: flex;
  justify-content: center;
  align-items: center;
  color: #fff;
  width: 100%;
  height: 100%;
}

.right {
  width: 600px;
  min-height: 500px;
  border-radius: 20px;
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
  padding: 60px 40px;

  .rText {
    text-align: center;
    margin-bottom: 80px;

    .title {
      font-size: 40px;
      font-weight: bold;
      line-height: 50px;
    }

    .description {
      margin-top: 20px;
      font-size: 24px;
      color: #c1c1c1;
      line-height: 36px;
    }
  }

  .btn-group {
    display: flex;
    flex-direction: column;
    gap: 40px;
    align-items: center;
  }

  .action {
    width: 300px;
    height: 100px;
    border-radius: 10px;
    background: linear-gradient(
      110deg,
      rgba(55, 89, 238, 0.64) 11%,
      rgba(30, 157, 244, 0.37) 89%
    );
    opacity: 1;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 36px;
    box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
    letter-spacing: 2px;
    cursor: pointer;

    &.stop-btn {
      background: linear-gradient(
        110deg,
        rgba(238, 55, 55, 0.64) 11%,
        rgba(244, 30, 30, 0.37) 89%
      );
    }

    &.disabled {
      background: #4f5478 !important;
      cursor: not-allowed;
      opacity: 0.5;
    }
  }
}
</style>
