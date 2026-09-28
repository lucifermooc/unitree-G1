<template>
  <div class="cliff-container">
    <div class="right">
      <div class="rText">
        <p class="title">悬崖检测标定</p>
        <p class="hint">请将机器人放置在静止的平整地面</p>
        <p class="description">{{ statusText }}</p>
      </div>

      <div class="btn-group">
        <div
          :class="['action', { disabled: loading }]"
          @click="handleCalibrate"
        >
          {{ loading ? "标定中..." : "开始标定" }}
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {
      loading: false,
      lastResult: null // null | {success: bool, message: string}
    };
  },
  computed: {
    statusText() {
      if (this.loading) return "正在标定中...";
      if (this.lastResult) {
        return this.lastResult.success ? "标定成功" : "标定失败";
      }
      return "";
    }
  },
  mounted() {
    // 每次进入页面重置为初始状态
    this.loading = false;
    this.lastResult = null;
  },
  methods: {
    handleCalibrate() {
      if (this.loading) return;
      this.loading = true;
      this.lastResult = null;

      const req = new ROSLIB.ServiceRequest({});
      console.log("[cliffCalib] call /calibrate_cliff", req);
      cliffCalibService.callService(
        req,
        res => {
          console.log("[cliffCalib] OK", res);
          this.loading = false;
          this.lastResult = {
            success: !!res.success,
            message: res.message || ""
          };
          if (res.success) {
            this.$message.success("标定成功");
          } else {
            this.$message.error(this.lastResult.message || "标定失败");
          }
        },
        err => {
          console.log("[cliffCalib] ERR", err);
          this.loading = false;
          this.lastResult = {
            success: false,
            message: (err && err.message) || err || "接口调用失败"
          };
          this.$message.error(this.lastResult.message);
        }
      );
    }
  }
};
</script>

<style lang="less" scoped>
.cliff-container {
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

    .hint {
      margin-top: 20px;
      font-size: 28px;
      color: #fff;
      line-height: 40px;
    }

    .description {
      margin-top: 16px;
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

    &.disabled {
      background: #4f5478 !important;
      cursor: not-allowed;
      opacity: 0.5;
    }
  }
}
</style>
