<template>
  <div>
    <div class="header">
      <div style="width: 800px">
        {{ `充电桩状态：${isSetDockPose ? "已" : "未"}设置充电桩位置` }}
      </div>
      <div>状态：{{ dockStatusLabel }}</div>
    </div>
    <div class="btns-row">
      <div class="btns">
        <!-- 执行按钮 -->
        <el-button
          type="primary"
          v-for="item in dockAction.slice(0, 3)"
          class="action"
          :key="item.value"
          @click="setDock(item.value)"
          :loading="loadingDock.startsWith(item.value)"
          :disabled="handleDisable(item)"
        >
          {{ item.label }}
        </el-button>
        <!-- 取消按钮 -->
        <el-button
          type="primary"
          v-for="item in dockAction.slice(3)"
          class="action"
          :key="item.value"
          @click="cancelDock"
          :loading="cancelLoading"
          :disabled="
            !['undock', 'set_dock_pose-result', 'dock-result'].includes(
              loadingDock
            )
          "
        >
          {{ item.label }}
        </el-button>
      </div>
      <div class="divider"></div>
      <div class="pile-config-row">
        <el-button
          type="primary"
          class="action"
          @click="scanPileInfo"
          :loading="scanPairLoading"
          :disabled="manualPairLoading || savePileLoading"
        >
          扫码配对
        </el-button>
        <el-button
          type="primary"
          class="action"
          @click="manualPairByConfirm"
          :loading="manualPairLoading"
          :disabled="scanPairLoading || savePileLoading"
        >
          手动配对
        </el-button>
      </div>
    </div>
    <div class="camera-bar"></div>
    <div class="camera-grid">
      <div>
        <div class="camera-switch">
          <span class="camera-label">后置相机</span>
          <el-switch
            v-model="cameraEnabled"
            active-text="打开"
            inactive-text="关闭"
            @change="handleCameraToggle"
          />
        </div>
        <div v-if="cameraEnabled" class="camera-panel">
          <div class="camera-title">
            {{ `状态：${cameraStatusText}` }}
          </div>
          <canvas ref="rearCameraCanvas" class="camera-canvas"></canvas>
          <div v-if="cameraError" class="camera-error">{{ cameraError }}</div>
        </div>
      </div>
      <div>
        <div class="camera-switch">
          <span class="camera-label">标签检测图</span>
          <el-switch
            v-model="tagCameraEnabled"
            active-text="打开"
            inactive-text="关闭"
            @change="handleTagCameraToggle"
          />
        </div>
        <div v-if="tagCameraEnabled" class="camera-panel">
          <div class="camera-title">
            {{ `状态：${tagCameraStatusText}` }}
          </div>
          <canvas ref="tagCameraCanvas" class="camera-canvas"></canvas>
          <div v-if="tagCameraError" class="camera-error">
            {{ tagCameraError }}
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
import fullscreenLoading from "@/components/fullscreenLoading";

export default {
  data() {
    return {
      isSetDockPose: false, // 是否设置了充电桩位置
      dockStatus: "",
      dockAction: [
        { label: "设为充电桩位置", value: "set_dock_pose" },
        { label: "开始回充", value: "dock" },
        { label: "脱离充电桩", value: "undock" },
        { label: "取消", value: "cancel_dock" }
      ],

      //当前正在执行哪一个dockAction，分为 `${value}`和`${value}-result`两种。`-result`表示是该action等待result结果的过程中
      // 如'dock'表示正在执行'/cmd_dock'的service返回过程中;'dock-result'表示service已返回true，正在等待'dock_result'返回结果
      loadingDock: "",
      cancelLoading: false, // cancel按钮的loading
      scanPairLoading: false, // 扫码配对loading
      manualPairLoading: false,
      savePileLoading: false,
      cameraEnabled: false,
      cameraError: "",
      cameraFrameReady: false,
      cameraFrameWidth: 0,
      cameraFrameHeight: 0,
      cameraRenderToken: 0,
      cameraSubscribed: false,
      tagCameraEnabled: false,
      tagCameraError: "",
      tagCameraFrameReady: false,
      tagCameraFrameWidth: 0,
      tagCameraFrameHeight: 0,
      tagCameraRenderToken: 0,
      tagCameraSubscribed: false
    };
  },
  computed: {
    dockStatusLabel() {
      const labes = {
        undock: "未充电",
        goto_dock_pose: "正在前往充电桩",
        charging: "已充上电",
        error: "错误"
      };
      return labes[this.dockStatus];
    },
    cameraStatusText() {
      if (this.cameraError) {
        return this.cameraError;
      }
      if (this.cameraFrameReady) {
        return `已接收画面 ${this.cameraFrameWidth} x ${this.cameraFrameHeight}`;
      }
      return "等待相机画面...";
    },
    tagCameraStatusText() {
      if (this.tagCameraError) {
        return this.tagCameraError;
      }
      if (this.tagCameraFrameReady) {
        return `已接收画面 ${this.tagCameraFrameWidth} x ${this.tagCameraFrameHeight}`;
      }
      return "等待相机画面...";
    }
  },
  mounted() {
    let loading = fullscreenLoading();
    setTimeout(() => {
      // loading 1分钟未结束就强制关闭
      loading && loading.close();
    }, 60 * 1000);
    // 状态机
    const type = new ROSLIB.ServiceRequest({
      action: "patrol"
    });
    robotMode.callService(
      type,
      result => {
        console.log("[ robotMode OK]-61", result);
        if (result.message !== "ok") {
          this.$message({
            id: "charge-robot-mode-failed",
            message: "状态切换失败"
          });
        }
        loading.close();
      },
      result => {
        this.$message({
          id: "charge-robot-mode-err",
          message: "状态切换失败"
        });
        console.log("[ robotMode ERR]-61", result);
        loading.close();
      }
    );
    this.getDockPose();
    this.initSubscribe();
  },
  beforeDestroy() {
    this.stopRearCameraSubscription();
    this.stopTagCameraSubscription();
  },
  methods: {
    getDockPose() {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id
      });
      getDockPoseService.callService(
        msg,
        result => {
          console.log("[  getDockPoseService result]-61", result);
          this.isSetDockPose = result.success;
        },
        result => {
          console.log("[  getDockPoseService ERR]-61", result);
          this.isSetDockPose = false;
        }
      );
    },
    initSubscribe() {
      dockStateTopic.subscribe(res => {
        this.dockStatus = res.data;
      });
    },
    handleCameraToggle(enabled) {
      if (enabled) {
        this.startRearCameraSubscription();
        return;
      }
      this.stopRearCameraSubscription();
    },
    handleTagCameraToggle(enabled) {
      const msg = new ROSLIB.ServiceRequest({ data: enabled });
      enableDetectionService.callService(
        msg,
        result => {
          console.log(
            "[ enableDetectionService OK ] " + (enabled ? "open" : "close"),
            result
          );
        },
        result => {
          console.log(
            "[ enableDetectionService err ] " + (enabled ? "open" : "close"),
            result
          );
        }
      );
      if (enabled) {
        this.startTagCameraSubscription();
        return;
      }
      this.stopTagCameraSubscription();
    },
    startRearCameraSubscription() {
      if (this.cameraSubscribed) {
        return;
      }
      this.cameraError = "";
      this.cameraFrameReady = false;
      this.cameraRenderToken++;
      rearCameraImageTopic.subscribe(this.handleRearCameraMessage);
      this.cameraSubscribed = true;
    },
    startTagCameraSubscription() {
      if (this.tagCameraSubscribed) {
        return;
      }
      this.tagCameraError = "";
      this.tagCameraFrameReady = false;
      this.tagCameraRenderToken++;
      tagDetectionsImageTopic.subscribe(this.handleTagCameraMessage);
      this.tagCameraSubscribed = true;
    },
    stopRearCameraSubscription() {
      if (this.cameraSubscribed) {
        rearCameraImageTopic.unsubscribe();
        this.cameraSubscribed = false;
      }
      this.cameraRenderToken++;
      this.cameraFrameReady = false;
      this.cameraFrameWidth = 0;
      this.cameraFrameHeight = 0;
      this.cameraError = "";
      this.clearRearCameraCanvas();
    },
    stopTagCameraSubscription() {
      if (this.tagCameraSubscribed) {
        tagDetectionsImageTopic.unsubscribe();
        this.tagCameraSubscribed = false;
      }
      this.tagCameraRenderToken++;
      this.tagCameraFrameReady = false;
      this.tagCameraFrameWidth = 0;
      this.tagCameraFrameHeight = 0;
      this.tagCameraError = "";
      this.clearTagCameraCanvas();
    },
    clearRearCameraCanvas() {
      const canvas = this.$refs.rearCameraCanvas;
      if (!canvas) return;
      const context = canvas.getContext("2d");
      context.clearRect(0, 0, canvas.width, canvas.height);
    },
    clearTagCameraCanvas() {
      const canvas = this.$refs.tagCameraCanvas;
      if (!canvas) return;
      const context = canvas.getContext("2d");
      context.clearRect(0, 0, canvas.width, canvas.height);
    },
    handleRearCameraMessage(message) {
      try {
        this.handleCameraMessage(message, {
          canvasRef: "rearCameraCanvas",
          enabledKey: "cameraEnabled",
          errorKey: "cameraError",
          frameReadyKey: "cameraFrameReady",
          frameWidthKey: "cameraFrameWidth",
          frameHeightKey: "cameraFrameHeight",
          renderTokenKey: "cameraRenderToken"
        });
      } catch (error) {
        console.error("rear camera render error", error);
        this.cameraError = error.message || "相机画面渲染失败";
      }
    },
    handleTagCameraMessage(message) {
      try {
        this.handleCameraMessage(message, {
          canvasRef: "tagCameraCanvas",
          enabledKey: "tagCameraEnabled",
          errorKey: "tagCameraError",
          frameReadyKey: "tagCameraFrameReady",
          frameWidthKey: "tagCameraFrameWidth",
          frameHeightKey: "tagCameraFrameHeight",
          renderTokenKey: "tagCameraRenderToken"
        });
      } catch (error) {
        console.error("tag camera render error", error);
        this.tagCameraError = error.message || "标签检测图渲染失败";
      }
    },
    handleCameraMessage(message, options) {
      const canvas = this.$refs[options.canvasRef];
      if (!canvas || !this[options.enabledKey]) {
        return;
      }
      if (this.isCompressedImageMessage(message)) {
        this.renderCompressedImageFrame(message, canvas, options);
        return;
      }
      const frame = this.normalizeImageFrame(message, options.errorKey);
      if (!frame) {
        return;
      }
      const context = canvas.getContext("2d");
      canvas.width = frame.width;
      canvas.height = frame.height;
      context.putImageData(frame.imageData, 0, 0);
      this[options.frameWidthKey] = frame.width;
      this[options.frameHeightKey] = frame.height;
      this[options.frameReadyKey] = true;
      this[options.errorKey] = "";
    },
    isCompressedImageMessage(message) {
      return !!(
        message &&
        message.data &&
        typeof message.format === "string" &&
        !message.width &&
        !message.height
      );
    },
    renderCompressedImageFrame(message, canvas, options) {
      const bytes = this.normalizeImageDataArray(message.data);
      if (!bytes.length) {
        this[options.errorKey] = "压缩相机消息没有图像数据";
        return;
      }
      const mimeType = this.getCompressedImageMimeType(message.format);
      const blob = new Blob([bytes], { type: mimeType });
      const imageUrl = window.URL.createObjectURL(blob);
      const image = new Image();
      const renderToken = this[options.renderTokenKey];
      image.onload = () => {
        window.URL.revokeObjectURL(imageUrl);
        if (
          !this[options.enabledKey] ||
          renderToken !== this[options.renderTokenKey]
        ) {
          return;
        }
        const context = canvas.getContext("2d");
        const width = image.naturalWidth || image.width;
        const height = image.naturalHeight || image.height;
        canvas.width = width;
        canvas.height = height;
        context.clearRect(0, 0, width, height);
        context.drawImage(image, 0, 0, width, height);
        this[options.frameWidthKey] = width;
        this[options.frameHeightKey] = height;
        this[options.frameReadyKey] = true;
        this[options.errorKey] = "";
      };
      image.onerror = () => {
        window.URL.revokeObjectURL(imageUrl);
        if (renderToken !== this[options.renderTokenKey]) {
          return;
        }
        this[
          options.errorKey
        ] = `压缩相机画面解码失败，format: ${message.format || "unknown"}`;
      };
      image.src = imageUrl;
    },
    getCompressedImageMimeType(format) {
      const normalized = (format || "").toLowerCase();
      if (normalized.includes("png")) {
        return "image/png";
      }
      if (normalized.includes("webp")) {
        return "image/webp";
      }
      return "image/jpeg";
    },
    normalizeImageFrame(message, errorKey) {
      if (!message || !message.width || !message.height) {
        this[errorKey] = "相机消息缺少宽高信息";
        return null;
      }
      const encoding = (message.encoding || "rgb8").toLowerCase();
      const width = message.width;
      const height = message.height;
      const step = message.step || 0;
      const source = this.normalizeImageDataArray(message.data);
      if (!source.length) {
        this[errorKey] = "相机消息没有图像数据";
        return null;
      }

      let imageData;
      switch (encoding) {
        case "rgb8":
          imageData = this.buildRgbImageData(
            source,
            width,
            height,
            false,
            step
          );
          break;
        case "bgr8":
          imageData = this.buildRgbImageData(source, width, height, true, step);
          break;
        case "rgba8":
          imageData = this.buildRgbaImageData(
            source,
            width,
            height,
            false,
            step
          );
          break;
        case "bgra8":
          imageData = this.buildRgbaImageData(
            source,
            width,
            height,
            true,
            step
          );
          break;
        case "mono8":
        case "8uc1":
          imageData = this.buildMonoImageData(source, width, height, step);
          break;
        default:
          throw new Error(`暂不支持的图像编码：${message.encoding}`);
      }

      return {
        width,
        height,
        imageData
      };
    },
    normalizeImageDataArray(data) {
      if (!data) {
        return new Uint8Array();
      }
      if (data instanceof Uint8Array) {
        return data;
      }
      if (Array.isArray(data)) {
        return Uint8Array.from(data);
      }
      if (typeof data === "string") {
        const binary = window.atob(data);
        const bytes = new Uint8Array(binary.length);
        for (let index = 0; index < binary.length; index++) {
          bytes[index] = binary.charCodeAt(index);
        }
        return bytes;
      }
      if (data.buffer && data.buffer instanceof ArrayBuffer) {
        return new Uint8Array(data.buffer);
      }
      return new Uint8Array();
    },
    buildRgbImageData(source, width, height, reverseChannel, step) {
      const imageData = new ImageData(width, height);
      const target = imageData.data;
      const rowStride = step || width * 3;
      for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
          const sourceIndex = row * rowStride + column * 3;
          const targetIndex = (row * width + column) * 4;
          const red = source[sourceIndex] || 0;
          const green = source[sourceIndex + 1] || 0;
          const blue = source[sourceIndex + 2] || 0;
          target[targetIndex] = reverseChannel ? blue : red;
          target[targetIndex + 1] = green;
          target[targetIndex + 2] = reverseChannel ? red : blue;
          target[targetIndex + 3] = 255;
        }
      }
      return imageData;
    },
    buildRgbaImageData(source, width, height, reverseChannel, step) {
      const imageData = new ImageData(width, height);
      const target = imageData.data;
      const rowStride = step || width * 4;
      for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
          const sourceIndex = row * rowStride + column * 4;
          const targetIndex = (row * width + column) * 4;
          const first = source[sourceIndex] || 0;
          const second = source[sourceIndex + 1] || 0;
          const third = source[sourceIndex + 2] || 0;
          target[targetIndex] = reverseChannel ? third : first;
          target[targetIndex + 1] = second;
          target[targetIndex + 2] = reverseChannel ? first : third;
          target[targetIndex + 3] =
            source[sourceIndex + 3] === undefined
              ? 255
              : source[sourceIndex + 3];
        }
      }
      return imageData;
    },
    buildMonoImageData(source, width, height, step) {
      const imageData = new ImageData(width, height);
      const target = imageData.data;
      const rowStride = step || width;
      for (let row = 0; row < height; row++) {
        for (let column = 0; column < width; column++) {
          const value = source[row * rowStride + column] || 0;
          const targetIndex = (row * width + column) * 4;
          target[targetIndex] = value;
          target[targetIndex + 1] = value;
          target[targetIndex + 2] = value;
          target[targetIndex + 3] = 255;
        }
      }
      return imageData;
    },
    handleDisable(item) {
      const actionDisable =
        !!this.loadingDock && !this.loadingDock.startsWith(item.value);
      let typeDisable = false;
      // 正在充电(靠桩)时，开始回充和设置充电桩位置按钮不让点击
      if (
        ["set_dock_pose", "dock"].includes(item.value) &&
        ["goto_dock_pose", "charging"].includes(this.dockStatus)
      )
        typeDisable = true;
      // 未设置位置时，不让充电
      if (["dock"].includes(item.value) && !this.isSetDockPose)
        typeDisable = true;

      return actionDisable || typeDisable;
    },
    cancelDock() {
      if (this.cancelLoading) return;
      this.cancelLoading = true;
      const msg = new ROSLIB.ServiceRequest({
        data: "cancel_dock"
      });
      dockService.callService(
        msg,
        result => {
          console.log("[  dockService result]-61", "cancel_dock", result);
          this.loadingDock = "";
          this.cancelLoading = false;
          if (!result.success) {
            return;
          }
        },
        result => {
          console.log("[  dockService ERR]-61", "cancel_dock", result);
          this.loadingDock = "";
          this.cancelLoading = false;
        }
      );
    },
    setDock(action) {
      console.log("setDock----:", action);
      const doAction = this.loadingDock || action;
      const dockLabel = this.dockAction.find(item =>
        doAction.startsWith(item.value)
      ).label;
      if (this.loadingDock) {
        this.$message({
          id: "charge-dock-running",
          message: `${dockLabel} 执行中，请稍后再试`
        });
        return;
      }
      this.loadingDock = action;
      const msg = new ROSLIB.ServiceRequest({
        data: action
      });
      dockService.callService(
        msg,
        result => {
          console.log("[  dockService result]-61", action, result);
          this.loadingDock = "";
          if (!result.success) {
            this.$message({
              id: "charge-dock-service-result-1" + result.message,
              message: result.message
            });
            return;
          }
          action === "undock"
            ? this.$message({
                id: "charge-dock-service-result-2" + dockLabel,
                message: `已${dockLabel}`
              })
            : this.waitResult(action);
        },
        result => {
          console.log("[  dockService ERR]-61", action, result);
          this.loadingDock = "";
          this.$message({
            id: "charge-dock-service-result-3" + dockLabel,
            message: `${dockLabel}失败`
          });
        }
      );
    },
    waitResult(action) {
      console.log("wait Result Topic:", action);
      this.loadingDock = action + "-result";
      // dock 和 set_dock_pose 的结果需要冲订阅中获取
      dockResultTopic.subscribe(res => {
        console.log("---dock Result res---", action, res);
        this.loadingDock = "";
        const labels = {
          dock_succeeded: "回充成功",
          dock_failed: "靠桩失败",
          detect_dock_failed: "未检测到充电桩",
          set_dock_pose_succeeded: "设置充电桩位置成功",
          set_dock_pose_failed: "设置充电桩位置失败"
        };
        this.$message({
          id: "charge-dock-result" + labels[res.data],
          message: labels[res.data]
        });
        if (action === "set_dock_pose") {
          // 更新充电桩位置信息
          this.getDockPose();
        }
        dockResultTopic.unsubscribe(res =>
          console.log("dockResultTopic unsubscribe:", res)
        );
      });
    },
    async scanPileInfo() {
      if (this.scanPairLoading) return;
      this.scanPairLoading = true;
      try {
        let rawText;
        try {
          rawText = await this.getScanCode();
        } catch (e) {
          // 扫码返回不报错
          console.error(e);
          return;
        }
        console.log("----rawText----:", rawText);
        const parsed = this.parsePileInfo(rawText);
        console.log("----parsed----:", parsed, JSON.stringify(parsed));
        if (!parsed) {
          throw new Error("二维码内容格式不正确，无法解析ID和通道");
        }
        await this.runPileConfigProcess(parsed);
        const formattedId = String(parsed.id).padStart(2, "0");
        const formattedChannel = String(parsed.channel).padStart(3, "0");
        this.$message.success(
          `扫码配对成功：SN:${parsed.sn ||
            ""}ID:${formattedId}CH:${formattedChannel}`
        );
      } catch (e) {
        this.$message.error(e.message || "扫码配对失败");
      } finally {
        this.scanPairLoading = false;
      }
    },
    manualPairByConfirm() {
      if (this.manualPairLoading) return;
      const manualPileIdInp = document.querySelector("#manualPileIdInp");
      manualPileIdInp && (manualPileIdInp.value = "");
      const manualPileChannelInp = document.querySelector(
        "#manualPileChannelInp"
      );
      manualPileChannelInp && (manualPileChannelInp.value = "");
      this.$confirm(
        `<div style="line-height: 70px;">
          <div><span style="display: inline-block; width: 200px;text-align: right;">充电桩ID：</span>
            <input id="manualPileIdInp" placeholder="请输入0~255整数" style="height: 60px; width: 300px;"></input>
          </div>
          <div><span style="display: inline-block; width: 200px;text-align: right;">通道号：</span>
            <input id="manualPileChannelInp" placeholder="请输入0~80整数" style="height: 60px; width: 300px;"></input>
          </div>
        </div>`,
        "手动配对",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      )
        .then(() => {
          const idInput = document.querySelector("#manualPileIdInp");
          const channelInput = document.querySelector("#manualPileChannelInp");
          const pileInfo = this.validatePileInput(
            idInput && idInput.value,
            channelInput && channelInput.value
          );
          if (!pileInfo) return;

          this.manualPairLoading = true;
          this.runPileConfigProcess(pileInfo)
            .then(() => {
              this.$message.success("手动配对成功");
            })
            .catch(e => {
              this.$message.error(e.message || "手动配对失败");
            })
            .finally(() => {
              this.manualPairLoading = false;
            });
        })
        .catch(() => {
          // 用户取消不提示
        });
    },
    async getScanCode() {
      const nativeCode = await this.tryNativeScan();
      if (nativeCode) return nativeCode;
      throw new Error("扫码失败，请重试");
    },
    tryNativeScan() {
      return new Promise(resolve => {
        const bridge = window.aidShowBridge;
        if (!bridge) {
          resolve("");
          return;
        }

        const callbackName = "__aidDockScanResult";
        window[callbackName] = result => {
          const value = (result || "").trim();
          resolve(value);
          delete window[callbackName];
        };

        const methods = ["scanQrCode", "scanQRCode", "scanCode", "scan"];
        for (let i = 0; i < methods.length; i++) {
          const method = methods[i];
          if (!bridge[method]) continue;
          try {
            const syncResult = bridge[method](callbackName);
            if (typeof syncResult === "string" && syncResult.trim()) {
              resolve(syncResult.trim());
              delete window[callbackName];
            }
            return;
          } catch (e) {
            console.log("[ bridge scan ERR ]", e);
          }
        }

        delete window[callbackName];
        resolve("");
      });
    },
    parsePileInfo(rawText) {
      const text = (rawText || "").trim();
      if (!text) return null;

      const normalize = value => {
        const num = Number(value);
        return Number.isInteger(num) ? num : null;
      };
      const normalizeSn = value => String(value || "").trim();
      const pickFirstDefined = list => {
        for (let i = 0; i < list.length; i++) {
          if (list[i] !== undefined && list[i] !== null) return list[i];
        }
        return undefined;
      };

      const valid = (id, channel) => {
        return (
          Number.isInteger(id) &&
          Number.isInteger(channel) &&
          id >= 0 &&
          id <= 255 &&
          channel >= 0 &&
          channel <= 80
        );
      };

      try {
        const obj = JSON.parse(text);
        const sn = normalizeSn(
          pickFirstDefined([
            obj.sn,
            obj.SN,
            obj.serial,
            obj.serial_no,
            obj.serialNo
          ])
        );
        const id = normalize(
          pickFirstDefined([obj.id, obj.pile_id, obj.pileId])
        );
        const channel = normalize(
          pickFirstDefined([
            obj.channel,
            obj.pile_channel,
            obj.pileChannel,
            obj.ch
          ])
        );
        if (valid(id, channel)) return { sn, id, channel };
      } catch (e) {
        // ignore JSON parse error
      }

      // Handle compact payload like: SN:HRS01DVT010YYG410001ID:03CH:003
      const compactMatch = text.match(
        /SN\s*:\s*(.*?)\s*ID\s*:\s*(\d{1,3})\s*CH(?:ANNEL)?\s*:\s*(\d{1,3})/i
      );
      if (compactMatch) {
        const sn = normalizeSn(compactMatch[1]);
        const id = normalize(compactMatch[2]);
        const channel = normalize(compactMatch[3]);
        if (valid(id, channel)) return { sn, id, channel };
      }

      const snMatch = text.match(
        /(?:^|\n)\s*sn\s*:\s*(.+?)(?=\s*(?:id|pile_id|ch|channel)\s*:|$)/i
      );
      const idMatch = text.match(/(?:^|[^a-z])(?:id|pile_id)\D*(\d{1,3})/i);
      const channelMatch = text.match(
        /(?:channel|pile_channel|ch)\D*(\d{1,3})/i
      );
      if (idMatch && channelMatch) {
        const sn = normalizeSn(snMatch && snMatch[1]);
        const id = normalize(idMatch[1]);
        const channel = normalize(channelMatch[1]);
        if (valid(id, channel)) return { sn, id, channel };
      }

      const nums = text.match(/\d+/g);
      if (nums && nums.length >= 2) {
        const sn = normalizeSn(snMatch && snMatch[1]);
        const id = normalize(nums[0]);
        const channel = normalize(nums[1]);
        if (valid(id, channel)) return { sn, id, channel };
      }

      return null;
    },
    validatePileInput(idRaw, channelRaw) {
      const id = Number(idRaw);
      const channel = Number(channelRaw);

      if (!Number.isInteger(id) || id < 0 || id > 255) {
        this.$message.error("充电桩ID必须是0~255的整数");
        return null;
      }
      if (!Number.isInteger(channel) || channel < 0 || channel > 80) {
        this.$message.error("通道号必须是0~80的整数");
        return null;
      }
      return { id, channel };
    },
    runPileConfigProcess(pileInfo) {
      this.savePileLoading = true;
      console.log(JSON.stringify(pileInfo));
      const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
      return this.setPileId(pileInfo.id)
        .then(() => wait(1000))
        .then(() => this.setPileChannel(pileInfo.channel))
        .then(() => wait(1000))
        .then(() => this.getPileId())
        .then(result => {
          const savedId = this.extractIdFromMessage(result.message || "");
          if (savedId !== pileInfo.id) {
            throw new Error(
              `校验失败：目标ID=${pileInfo.id}，当前ID=${savedId}`
            );
          }
          return result;
        })
        .finally(() => {
          this.savePileLoading = false;
        });
    },
    setPileId(id) {
      return new Promise((resolve, reject) => {
        console.log("[ setPileIdService start ---- ]");
        const msg = new ROSLIB.ServiceRequest({
          data: String(id)
        });
        setPileIdService.callService(
          msg,
          result => {
            console.log(
              "[ setPileIdService result ]",
              result,
              JSON.stringify(result)
            );
            if (!result.success) {
              reject(new Error(result.message || "设置充电桩ID失败"));
              return;
            }
            resolve(result);
          },
          result => {
            console.log("[ setPileIdService ERR ]", result);
            reject(new Error("设置充电桩ID接口调用失败"));
          }
        );
      });
    },
    setPileChannel(channel) {
      return new Promise((resolve, reject) => {
        console.log("[ setPileChannelService start ---- ]");
        const msg = new ROSLIB.ServiceRequest({
          data: String(channel)
        });
        setPileChannelService.callService(
          msg,
          result => {
            console.log(
              "[ setPileChannelService result ]",
              result,
              JSON.stringify(result)
            );
            if (!result.success) {
              reject(new Error(result.message || "设置通道失败"));
              return;
            }
            resolve(result);
          },
          result => {
            console.log("[ setPileChannelService ERR ]", result);
            reject(new Error("设置通道接口调用失败"));
          }
        );
      });
    },
    getPileId() {
      return new Promise((resolve, reject) => {
        const msg = new ROSLIB.ServiceRequest({
          data: ""
        });
        getPileIdService.callService(
          msg,
          result => {
            console.log(
              "[ getPileIdService result ]",
              result,
              JSON.stringify(result)
            );
            if (!result.success) {
              reject(new Error(result.message || "读取充电桩ID失败"));
              return;
            }
            resolve(result);
          },
          result => {
            console.log("[ getPileIdService ERR ]", result);
            reject(new Error("读取充电桩ID接口调用失败"));
          }
        );
      });
    },
    extractIdFromMessage(message) {
      const match = String(message || "").match(/\d+/);
      return match ? Number(match[0]) : NaN;
    }
  }
};
</script>

<style lang="less" scoped>
.header {
  display: flex;
  padding: 60px 50px 50px 50px;
}
.btns-row {
  padding: 0 50px;
  display: flex;
  align-items: center;
  justify-content: space-between;

  .btns {
    display: grid;
    grid-template-columns: repeat(4, minmax(0, 1fr));
    gap: 24px;
  }
  .divider {
    width: 3px;
    height: 100px;
    background-color: #2d6ac2;
    opacity: 0.5;
    justify-self: center;
  }
  .pile-config-row {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 24px;
  }
  .action {
    width: 260px;
    height: 110px;
    font-size: 35px;
    border-radius: 10px;
    opacity: 1;
    display: flex;
    justify-self: center;
    align-items: center;
    justify-content: center;
    box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
    background: linear-gradient(
      110deg,
      rgba(55, 89, 238, 0.64) 11%,
      rgba(30, 157, 244, 0.37) 89%
    );
  }
  .action.is-disabled {
    opacity: 0.5;
    cursor: not-allowed;
    background: #4f5478;
  }
  .pile-btn {
    width: 320px;
    height: 120px;
    font-size: 35px;
    justify-self: center;
  }
  .action,
  .pile-btn {
    -webkit-tap-highlight-color: transparent;
  }
  ::v-deep .el-button.action:hover,
  ::v-deep .el-button.action:focus,
  ::v-deep .el-button.action:active,
  ::v-deep .el-button.pile-btn:hover,
  ::v-deep .el-button.pile-btn:focus,
  ::v-deep .el-button.pile-btn:active {
    color: #fff;
    border-color: transparent;
    background: linear-gradient(
      110deg,
      rgba(55, 89, 238, 0.64) 11%,
      rgba(30, 157, 244, 0.37) 89%
    );
  }
}

.camera-grid {
  display: grid;
  grid-template-columns: repeat(2, minmax(0, 1fr));
  gap: 30px;
  margin: 50px 50px 0;
  .camera-switch {
    flex: 1;
    display: flex;
    align-items: center;
    color: #fff;
    margin-bottom: 30px;
    .camera-label {
      margin-right: 28px;
      font-size: 40px;
      line-height: 1;
    }
    /deep/ .el-switch {
      transform: scale(2) translateY(1px);
      transform-origin: left center;
      .el-switch__label:not(.is-active) {
        color: #fff;
      }
    }
  }
  .camera-panel {
    padding: 20px 30px;
    height: calc(561px - 20px - 20px); // 减去padding
    display: flex;
    flex-direction: column;
    overflow: hidden;
    border-radius: 20px;
    background: linear-gradient(
      132deg,
      rgba(71, 84, 141, 0.64) 17%,
      rgba(53, 81, 119, 0.15) 92%,
      rgba(53, 92, 119, 0.14) 93%
    );
    box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
    .camera-title {
      margin-bottom: 10px;
      color: #fff;
      font-size: 24px;
    }
    .camera-canvas {
      display: block;
      width: 100%;
      flex: 1;
      height: 480px;
      object-fit: contain;
      background: rgba(8, 15, 34, 0.8);
    }
    .camera-error {
      margin-top: 20px;
      color: #ffb4b4;
      font-size: 26px;
    }
  }
}
.el-loading-mask {
  background-color: rgba(100, 100, 100, 0.5);
}
</style>
