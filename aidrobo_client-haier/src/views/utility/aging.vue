<template>
  <div class="aging-container">
    <div class="right">
      <div class="rText">
        <p class="title">老化测试</p>
        <p class="description">{{ statusText }}</p>
      </div>

      <div class="btn-group">
        <div
          :class="['action', { disabled: isRunning || loading || !statusReceived }]"
          @click="handleStart"
        >
          开始
        </div>
        <div
          :class="['action', 'stop-btn', { disabled: !isRunning || loading || !statusReceived }]"
          @click="handleStop"
        >
          结束
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
| 1 | running（运行中） |
| 4 | charging（充电中） |
* */
const AGING_STATUS_TEXT = {
  0: "点击「开始」启动老化测试",
  1: "老化测试运行中",
  4: "老化测试充电中"
};

export default {
  data() {
    return {
      agingStatus: 0,
      statusReceived: false,
      loading: false
    };
  },
  computed: {
    // 运行中或充电中都算老化测试进行中，可结束
    isRunning() {
      return this.agingStatus === 1 || this.agingStatus === 4;
    },
    statusText() {
      if (this.loading) return "正在执行中...";
      if (!this.statusReceived) return "正在获取老化测试状态...";
      return AGING_STATUS_TEXT[this.agingStatus] || "老化测试状态未知";
    }
  },
  mounted() {
    // 每次进入页面重置为初始状态
    this.agingStatus = 0;
    this.statusReceived = false;
    this.loading = false;
    agingControlStatusTopic.subscribe(this.handleAgingStatus);
  },
  beforeDestroy() {
    agingControlStatusTopic.unsubscribe(this.handleAgingStatus);
  },
  methods: {
    handleAgingStatus(res) {
      const status = Number(res && res.status);
      if (Number.isNaN(status)) return;

      console.log("[aging] status", res);
      this.statusReceived = true;
      this.agingStatus = status;
    },

    /**
     * 带重试的服务调用（所有接口都检查 result.success）
     * @param {ROSLIB.Service} service - 服务对象
     * @param {object} request - ServiceRequest 参数
     * @returns {Promise<{success: boolean, message: string}>}
     */
    callServiceWithRetry(service, request) {
      return new Promise(resolve => {
        let retryCount = 0;
        const maxRetries = 5;

        const attempt = () => {
          console.log(`[aging] call ${service.name}`, request);
          service.callService(
            request,
            result => {
              console.log(`[aging] ${service.name} OK`, result);
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
              console.log(`[aging] ${service.name} ERR`, error);
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

      // 第一步：/arm_test/start_system
      const startReq = new ROSLIB.ServiceRequest({ cmd: "" });
      const r1 = await this.callServiceWithRetry(startArmTest, startReq);
      if (!r1.success) {
        this.showResult(r1);
        this.loading = false;
        return;
      }

      // 第二步：/resume_aging
      const resumeReq = new ROSLIB.ServiceRequest({});
      const r2 = await this.callServiceWithRetry(resumeAgingService, resumeReq);
      if (!r2.success) {
        this.showResult(r2);
        this.loading = false;
        return;
      }

      // 全部成功，运行状态由 /aging_control_status 订阅驱动
      this.loading = false;
    },

    async handleStop() {
      if (!this.isRunning || this.loading || !this.statusReceived) return;
      this.loading = true;

      // 第一步：/stop_aging
      const stopReq = new ROSLIB.ServiceRequest({});
      const r1 = await this.callServiceWithRetry(stopAgingService, stopReq);
      if (!r1.success) {
        this.showResult(r1);
        this.loading = false;
        return;
      }

      // 第二步：/arm_test/stop_system
      const stopArmReq = new ROSLIB.ServiceRequest({ cmd: "" });
      const r2 = await this.callServiceWithRetry(stopArmTest, stopArmReq);
      if (!r2.success) {
        this.showResult(r2);
        this.loading = false;
        return;
      }

      // 全部成功，运行状态由 /aging_control_status 订阅驱动
      this.loading = false;
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
