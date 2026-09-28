<template>
  <div class="main">
    <div class="box">
      <div v-for="(item, i) in box" :key="i">
        <div class="title">{{ item.title }}</div>
        <!-- 基础设置 -->
        <div v-if="item.title === '表情锁屏'">
          <div class="item">
            <span>开启/关闭</span>
            <el-switch
              v-model="value"
              active-color="#05E29E"
              inactive-color="#1A1931"
              active-text="开"
              inactive-text="关"
            >
            </el-switch>
          </div>
        </div>
        <div v-else>
          <div
            v-for="(v, k) in item.info"
            :key="k"
            class="item"
            :class="{ 'item-multi': Array.isArray(v) }"
          >
            <template v-if="Array.isArray(v)">
              <div class="item-label">{{ k }}:</div>
              <div class="item-values">
                <div v-for="(line, idx) in v" :key="idx">{{ line }}</div>
              </div>
            </template>
            <template v-else>{{ k }}:{{ v }}</template>
          </div>
          <div v-if="i === 0" class="item">
            <span>通信IP：{{ ip }}</span>
            <span class="ip-button" @click="setIP">设置</span>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script>
const mcuVersionLabel = "MCU信息";
const electricalBoardLabel = "电源管理板信息";
const releaseVersionLabel = "导航信息";
export default {
  data() {
    return {
      ip: "",
      value: true,
      input: "",
      box: [
        // {
        //   title: "表情锁屏"
        // },
        // {
        //   title: "设备信息",
        //   info: {
        //     '机器编号(SN)': "AIDR-T2301110001",
        //     '软件版本': "0.5.5.1.d",
        //   }
        // },
        {
          title: "主要硬件信息",
          info: {
            主控: "AIBOX Powerby Aidlux",
            辅控: "艺科 YKRC-2",
            激光雷达: "蓝海 LDS-50C-C20E",
            双目模组: "奥比中光 DaBai"
          }
        },
        {
          title: "版本信息",
          info: {
            [mcuVersionLabel]: [""],
            [electricalBoardLabel]: [""],
            [releaseVersionLabel]: [""]
          }
        }
      ]
    };
  },
  mounted() {
    const storageIP = localStorage.getItem("rosIP");
    this.ip = storageIP || rosIP;
    console.log(this.ip);
    this.fetchVersions();
  },
  methods: {
    setVersionInfo(key, value) {
      const rawText = (value || "").toString();
      const normalized =
        key === releaseVersionLabel ? rawText.replace(/\r?\n/g, " ") : rawText;
      this.box[1].info[key] = this.parseMessageItems(normalized);
    },
    parseMessageItems(message) {
      const text = (message || "")
        .toString()
        .replace(/\r\n/g, "\n")
        .trim();
      if (!text) return ["-"];
      const lines = [];
      text.split("\n").forEach(line => {
        line.split(",").forEach(part => {
          const item = part.trim();
          if (item) lines.push(item);
        });
      });
      return lines.length ? lines : ["-"];
    },
    async fetchVersions() {
      const mcuMsg = new ROSLIB.ServiceRequest({ data: "mcu_board" });
      getEmbeddedVersionService.callService(
        mcuMsg,
        result => {
          console.log("[  getEmbeddedVersionService MCU OK]-61", result); //{success: true, message: 'hw_ver: v0.1.1, sw_ver: v0.0.0, type: APP'}
          if (result && result.success) {
            this.setVersionInfo(mcuVersionLabel, result.message || "");
          } else {
            this.setVersionInfo(mcuVersionLabel, "-");
          }
        },
        result => {
          console.log("[  getEmbeddedVersionService MCU ERR]-61", result);
          this.setVersionInfo(mcuVersionLabel, "-");
        }
      );

      const electricalMsg = new ROSLIB.ServiceRequest({
        data: "electrical_board"
      });
      getEmbeddedVersionService.callService(
        electricalMsg,
        result => {
          console.log("[  getEmbeddedVersionService Electrical OK]-61", result); //{success: true, message: 'hw_ver: v1.0.1, sw_ver: v1.0.2, type: APP'}
          if (result && result.success) {
            this.setVersionInfo(electricalBoardLabel, result.message || "");
          } else {
            this.setVersionInfo(electricalBoardLabel, "-");
          }
        },
        result => {
          console.log(
            "[  getEmbeddedVersionService Electrical ERR]-61",
            result
          );
          this.setVersionInfo(electricalBoardLabel, "-");
        }
      );

      getReleaseVersionService.callService(
        null,
        result => {
          console.log("[  getReleaseVersionService OK]-61", result); // {success: true, message: 'version: t4.0.0, description: 1.这是一个测试版本\n2.新增头部电机日志 \n3.修复悬空障碍不绕行\n'}
          this.setVersionInfo(
            releaseVersionLabel,
            result ? result.message : ""
          );
        },
        result => {
          console.log("[  getReleaseVersionService ERR]-61", result);
          this.setVersionInfo(releaseVersionLabel, "-");
        }
      );
    },
    setIP() {
      const ipInp = document.querySelector("#ipInp");
      ipInp && (ipInp.value = "");
      this.$confirm(
        `<div> 通信IP：
        <input id="ipInp" placeholder=" 请输入IP，如 192.168.1.1" style="height: 70px;"></input>
        </div><div>保存后会自动回到首页并刷新页面。</div>`,
        "IP设置",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      ).then(() => {
        const ipInp = document.querySelector("#ipInp");
        const val = ipInp.value.trim();
        console.log(val);
        localStorage.setItem("rosIP", val);
        this.$router.push("/?reload=true");
      });
    }
  }
};
</script>

<style lang="less" scoped>
.main {
  overflow: auto;
  height: 100%;
}

.box {
  display: flex;
  flex-wrap: wrap;
  color: #fff;

  & > div {
    padding: 40px 70px;
    width: 670px;
    min-height: 220px;
    border-radius: 20px;
    background: linear-gradient(
      116deg,
      rgba(71, 84, 141, 0.64) 12%,
      rgba(71, 66, 124, 0.52) 90%
    );
    backdrop-filter: blur(10.88px);
    box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
    margin-top: 30px;
    margin-bottom: 30px;
    margin-left: 100px;

    .title {
      font-size: 40px;
      font-weight: bold;
    }

    & > div {
      margin-top: 40px;

      .item {
        margin-top: 20px;
        display: flex;
        align-items: center;
        justify-content: space-between;
        width: 730px;
        line-height: 1.2em;

        &.item-multi {
          display: block;
        }

        .item-label {
          margin-right: 16px;
          white-space: nowrap;
        }

        .item-values {
          margin-left: 50px;
          margin-top: 10px;
          text-align: left;

          & > div {
            margin-top: 20px;
          }

          & > div:first-child {
            margin-top: 0;
          }
        }

        .el-switch {
          transform: scale(2.5);
          margin-right: 30px;
        }
      }

      .itemTip {
        font-size: 28px;
        color: #c1c1c1;
      }
    }
  }

  .ip-button {
    margin-left: 20px;
    margin-right: 100px;
    padding: 2px 10px 5px;
    border-radius: 10px;
    opacity: 1;
    background: linear-gradient(120deg, #c061ff 15%, #7364ff 69%);
    box-shadow: 0px 5px 10px 0px rgba(0, 0, 0, 0.4);
    line-height: 1.2em;
  }
}
</style>
<style scoped>
.item >>> .el-switch__label {
  position: absolute;
  display: none;
  color: #fff;
}

/*打开时文字位置设置*/
.item >>> .el-switch__label--right {
  z-index: 1;
  right: 18px;
}

/*关闭时文字位置设置*/
.item >>> .el-switch__label--left {
  z-index: 1;
  left: 18px;
}

/*显示文字*/
.item >>> .el-switch__label.is-active {
  display: block;
}

.timeInp >>> .el-input {
  width: 140px;
  font-size: 35px;
}

.filesInp >>> .el-input {
  width: 300px;
  font-size: 35px;
}

.item >>> .el-input__inner {
  height: 70px;
  border-radius: 10px;
}
</style>
