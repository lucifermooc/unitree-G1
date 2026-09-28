<template>
  <div class="calibration-root">
    <div class="card-container">
      <!-- 上相机卡片 -->
      <div class="card">
        <div class="card-header">
          <h3>上相机（Up）</h3>
          <div :class="['status-badge', statusClass(upStatus)]">{{ upStatus }}</div>
        </div>

        <!--
        <div class="section-title">当前</div>
        <pre class="param-box" v-if="upParams">{{ upParams }}</pre>
        <pre class="param-box empty" v-else>等待 TF 数据...</pre>

        <div class="section-title">后端日志 / 最近消息</div>
        <div class="log-box">{{ upLastMsg || "无" }}</div>-->

        <div class="card-actions">
          <button
            class="calib-btn"
            :class="{ pressed: upPressed }"
            @mousedown.prevent="onUpMouseDown"
            @mouseup.prevent="onUpMouseUp"
            @touchstart.prevent="onUpMouseDown"
            @touchend.prevent="onUpMouseUp"
          >
            标定上相机
          </button>
        </div>
      </div>

      <!-- 下相机卡片 -->
      <div class="card">
        <div class="card-header">
          <h3>下相机（Down）</h3>
          <div :class="['status-badge', statusClass(downStatus)]">{{ downStatus }}</div>
        </div>

        <!--
        <div class="section-title">当前（TF）</div>
        <pre class="param-box" v-if="downParams">{{ downParams }}</pre>
        <pre class="param-box empty" v-else>等待 TF 数据...</pre>

        <div class="section-title">后端日志 / 最近消息</div>
        <div class="log-box">{{ downLastMsg || "无" }}</div>-->

        <div class="card-actions">
          <button
            class="calib-btn"
            :class="{ pressed: downPressed, disabled: !upCalibrated }"
            :disabled="!upCalibrated"
            @mousedown.prevent="onDownMouseDown"
            @mouseup.prevent="onDownMouseUp"
            @touchstart.prevent="onDownMouseDown"
            @touchend.prevent="onDownMouseUp"
          >
            标定下相机
          </button>
        </div>
      </div>
    </div>


    <div v-if="showModal" class="modal-backdrop" @click="closeModal">
      <div class="modal" @click.stop>
        <h4>{{ modalTitle }}</h4>
        <p>{{ modalMessage }}</p>
        <div class="modal-actions">
          <button @click="closeModal">关闭</button>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
export default {
  name: "CalibrationPage",
  data() {
    return {
      // 状态
      upStatus: "未标定",
      downStatus: "未标定",
      upCalibrated: false,

      // TF 显示
      upParams: null,
      downParams: null,

      // 后端消息（日志摘要）
      upLastMsg: "",
      downLastMsg: "",

      // 按钮按下效果
      upPressed: false,
      downPressed: false,

      // 弹窗
      showModal: false,
      modalTitle: "",
      modalMessage: ""
    };
  },
  computed: {
    // nothing special here
  },
  mounted() {
    this.$message('相机需要在静止状态下进行标定，且标定完成后需要重启程序');
    this.initSubscribe();
  },
  methods: {
    statusClass(status) {
      switch (status) {
        case "未标定": return "unset";
        case "正在标定": return "running";
        case "标定完成": return "success";
        case "标定失败": return "fail";
        default: return "";
      }
    },
    initSubscribe() {
      rgbdCalibStatusTopic.subscribe(res => {
        console.log('up status', res)
        const statusLabel = ['未标定', '正在标定', '标定完成', '标定失败', '标定失败'];
        this.upStatus = statusLabel[res.data]; // res.data: 0 ~ 4
      })
    },

    // ========== 上相机：按下/松开 ==========
    onUpMouseDown() {
      this.upPressed = true;
      // 按下时立刻调用服务
      this.callUpService();
    },
    onUpMouseUp() {
      this.upPressed = false;
    },

    // ========== 下相机：按下/松开 ==========
    onDownMouseDown() {
      if (!this.upCalibrated) {
        // 保护：不允许按下执行
        this.openModal("提示", "请先标定上相机");
        return;
      }
      this.downPressed = true;
      this.callDownService();
    },
    onDownMouseUp() {
      this.downPressed = false;
    },

    // 调用上相机服务
    callUpService() {
      this.upLastMsg = "";
      const req = new ROSLIB.ServiceRequest({});
      UpCalibService.callService(req, (res) => {
        // res: { success: bool, message: string }
        this.upLastMsg = res.message || "";
        if (res.success) {
          this.upCalibrated = true;
          // 如果后端在 message 中返回外参或说明，可以更新显示
          if (res.message) this.upParams = res.message;
        } else {
          // 后端 log 包含平面系数 => 弹窗提示清理杂物
          const msg = (res.message || "").toLowerCase();
          if (msg.includes("平面系数") || msg.includes("plane")) {
            this.openModal("请清理", "检测到平面系数异常：请清理相机前方杂物后重试。");
          }
        }
      }, (err) => {
        // 可选：一些 rosbridge 的错误回调
        console.error("Up service error:", err);
        this.upLastMsg = String(err || "service call error");
      });
    },

    // 调用下相机服务
    callDownService() {
      this.downStatus = "正在标定";
      this.downLastMsg = "";
      const req = new ROSLIB.ServiceRequest({});
      DownCalibService.callService(req, (res) => {
        this.downLastMsg = res.message || "";
        if (res.success) {
          this.downStatus = "标定完成";
          if (res.message) this.downParams = res.message;
        } else {
          // 如果后端日志显示 "done" 并失败 => 标定失败（按你需求）
          const msg = (res.message || "").toLowerCase();
          if (msg.includes("done")) {
            this.downStatus = "标定失败";
            this.openModal("标定失败", "下相机标定日志包含 'done'，标定被判定为失败。请检查配置或环境。");
          } else {
            this.downStatus = "标定失败";
            // 也可以展示后端返回信息
            this.openModal("标定失败", res.message || "下相机标定失败");
          }
        }
      }, (err) => {
        console.error("Down service error:", err);
        this.downLastMsg = String(err || "service call error");
        this.downStatus = "标定失败";
      });
    },

    // 打开 modal
    openModal(title, message) {
      this.modalTitle = title;
      this.modalMessage = message;
      this.showModal = true;
    },
    closeModal() {
      this.showModal = false;
    },
  }
};
</script>

<style scoped>
.calibration-root {
  padding: 28px;
  background: linear-gradient(155deg, rgba(71, 84, 141, 0.64) 24%, rgba(71, 66, 124, 0.52) 98%);
  min-height: 100%;
  box-sizing: border-box;
  line-height: 1.3em;
}
.page-title {
  text-align: center;
  margin-bottom: 20px;
  color: #fff; /* 白色标题 */
}

/* 卡片容器 */
.card-container {
  display: flex;
  justify-content: center;
  gap: 28px;
  align-items: flex-start;
  flex-wrap: wrap;
}

/* 单张卡片：浅蓝背景 */
.card {
  flex: 1;
  min-height: 420px;
  background: rgba(71, 105, 180, 0.75); /* 浅蓝半透明 */
  border-radius: 12px;
  box-shadow: 0 8px 24px rgba(18, 42, 66, 0.08);
  padding: 20px;
  display: flex;
  flex-direction: column;
  gap: 30px;
}

/* header */
.card-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.card-header h3 {
  margin: 0;
  color: #fff; /* 白色文字 */
}

/* 状态徽章 */
.status-badge {
  padding: 8px 12px;
  border-radius: 16px;
  color: white;
  font-size: 28px;
}
.status-badge.unset { background: #888; }
.status-badge.running { background: #f39c12; }
.status-badge.success { background: #16a34a; }
.status-badge.fail { background: #dc2626; }

/* 参数区域 */
.section-title {
  font-size: 28px;
  color: #fff; /* 白色文字 */
  margin-top: 6px;
}
.param-box {
  background: rgba(255, 255, 255, 0.1); /* 半透明浅蓝底色 */
  color: #fff;
  padding: 12px;
  border-radius: 8px;
  min-height: 120px;
  max-height: 190px;
  overflow: auto;
  font-family: "SFMono-Regular", Consolas, Monaco, monospace;
  font-size: 28px;
  white-space: pre-wrap;
}
.param-box.empty {
  color: #ccc;
  background: rgba(255,255,255,0.05);
  border: 1px dashed #fff;
}

/* 日志区域 */
.log-box {
  background: rgba(255,255,255,0.1);
  border: 1px solid rgba(255,255,255,0.2);
  padding: 10px;
  border-radius: 8px;
  min-height: 60px;
  max-height: 190px;
  overflow: auto;
  color: #fff;
  font-size: 28px;
}

/* 按钮 */
.card-actions {
  margin-top: auto;
  display: flex;
  justify-content: center;
}
.calib-btn {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  border: none;
  font-size: 35px;
  font-weight: 400;
  color: #fff;
  background: linear-gradient(110deg, rgba(55, 89, 238, 0.64) 11%, rgba(30, 157, 244, 0.37) 89%);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  cursor: pointer;
  transition: transform .08s, filter .12s;
}
.calib-btn.pressed {
  transform: translateY(1px);
  filter: brightness(.82);
}
.calib-btn.disabled {
  background: rgba(71, 84, 141, 0.4);
  cursor: not-allowed;
  color: #e0eaf5;
  box-shadow: none;
}

/* Modal */
.modal-backdrop {
  position: fixed;
  left: 0; right: 0; top: 0; bottom: 0;
  background: rgba(0,0,50,0.45); /* 深蓝半透明遮罩 */
  display: flex;
  justify-content: center;
  align-items: center;
  z-index: 9999;
}
.modal {
  background: rgba(71, 105, 180, 0.85); /* 浅蓝卡片 */
  padding: 18px;
  border-radius: 10px;
  width: 600px;
  box-shadow: 0 10px 30px rgba(2,6,23,0.15);
}
.modal h4 { margin: 0 0 8px 0; color: #fff; }
.modal-actions { text-align: right; margin-top: 14px; }
.modal-actions button {
  padding: 15px 50px;
  border: none;
  border-radius: 8px;
  background: #0ea5a8;
  color: #fff;
  cursor: pointer;
  font-size: 35px;
}


</style>
