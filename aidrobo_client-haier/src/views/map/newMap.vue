<template>
  <div class="newMapBox">
    <ShowMap />
    <div class="right">
      <p>IP:{{ $store.state.IP || "--" }}</p>
      <p>
        请使用无线键盘方向键或手机遥控App，控制机器人行走建图，完成扫描后点击完成扫描进入下一步
      </p>
      <p>注意：起始位置为起始点或充电桩，建图需完成回环后回到该位置</p>
      <div class="iconBtn" @click="onRelocation()" v-if="continueMapping">
        重定位
      </div>
      <div class="over" @click="onOver()">完成扫描</div>
      <div class="out" @click="onOut()">退出</div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/new";
import fullscreenLoading from "@/components/fullscreenLoading.js";

export default {
  components: {
    ShowMap
  },
  data() {
    return {
      mapName: this.$route.query.mapName
    };
  },
  computed: {
    continueMapping() {
      return this.$route.query.continueMapping === "true";
    }
  },
  beforeRouteLeave(to, from, next) {
    if (this.continueMapping && to.name === "relocation") {
      next();
      return;
    }
    console.log('robotMode 设为 localization')
    const modeMsg = new ROSLIB.ServiceRequest({
      action: "localization"
    });
    robotMode.callService(
      modeMsg,
      result => {
        console.log("[ 离开建图，状态重置为localization OK]-61", result);
      },
      result => {
        console.log("[ 离开建图，状态重置为localization ERR]-61", result);
      }
    );
    next();
  },
  mounted() {},
  methods: {
    onRelocation() {
      this.$router.push({ name: "relocation" });
    },
    onOver () {
      if (this.continueMapping) {
        // 继续建图模式：提示用户输入新地图名再保存
        const addInp = document.querySelector("#addInp");
        addInp && (addInp.value = "");
        this.$confirm(
          `<div> 名称：
          <input id="addInp" placeholder=" 请输入内容" style="height: 70px;"></input>
          </div>`,
          "地图命名",
          {
            dangerouslyUseHTMLString: true,
            center: true,
            confirmButtonClass: "save-map-confirm",
            cancelButtonClass: "save-map-cancel"
          }
        ).then(() => {
          const addInp = document.querySelector("#addInp");
          const mapName = addInp.value;
          this.doSaveMap(mapName);
        });
      } else {
        this.$confirm(`<div>是否确认完成扫描，确认后将生成地图进入编辑</div><div>（无法返回）</div>`, '完成扫描', {
          dangerouslyUseHTMLString: true,
          center: true,
          confirmButtonClass: "save-map-confirm",
          cancelButtonClass: "save-map-cancel"
        }).then(() => {
          this.doSaveMap(this.mapName);
        });
      }
    },
    async doSaveMap (mapName) {
      const loading = fullscreenLoading();
      try {
        const date = Date.now();

        const saveResult = await new Promise((resolve, reject) => {
          const msg = new ROSLIB.ServiceRequest({ map_file_name: `/maps/${date}` });
          saveMap.callService(msg, resolve, reject);
        });
        console.log('[  saveMap Res]-61', saveResult);
        if (!saveResult.success) {
          throw new Error('saveMap failed');
        }

        const dbResult = await new Promise((resolve, reject) => {
          const msg2 = new ROSLIB.ServiceRequest({ map_name: mapName, map_file: `/maps/${date}` });
          saveMapDb.callService(msg2, resolve, reject);
        });
        console.log('[  saveMapDb RES]-61', dbResult);
        if (!dbResult.success) {
          throw new Error('saveMapDb failed');
        }

        this.$router.push({ name: 'map' });
      } catch (err) {
        console.log('[  doSaveMap ERR]-61', err,);
        this.$message('保存失败');
      } finally {
        loading.close();
      }
    },
    onOut () {
      this.$confirm(`<div>是否确认退出</div><div>（已扫描地图不会保存）</div>`, '退出扫描', {
        dangerouslyUseHTMLString: true,
        center: true
      }).then(() => {
        this.$router.push({ name: 'map' })
        console.log('[  ]-72',)
        this.$store.state.actionStatus = 'idle'
        const msg = new ROSLIB.ServiceRequest({
          action: 'idle'
        });
        robotMode.callService(msg, (result) => {
          console.log('[  robotMode set idle OK]-61', result)
        }, (result) => {
          console.log('[  robotMode set idle ERR]-61', result)
        });
        console.log('[  ]-69',)
      })
    }
  }

}
</script>

<style lang="less" scoped>
.newMapBox {
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
  align-items: center;
  justify-content: space-evenly;
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
  justify-content: space-evenly;

  & > p {
    text-align: center;
    text-align: center;
    line-height: 50px;
    padding: 0px 40px;
  }
}

.iconBtn {
  display: flex;
  align-items: center;
  justify-content: space-evenly;
  width: 300px;
  height: 100px;
  border-radius: 10px;
  opacity: 1;
  background: linear-gradient(
    110deg,
    rgba(71, 84, 141, 0.64) 11%,
    rgba(53, 81, 119, 0.15) 88%,
    rgba(53, 92, 119, 0.14) 89%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.over {
  width: 300px;
  height: 100px;
  border-radius: 10px;
  opacity: 1;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  background: #b04cf3;
  display: flex;
  align-items: center;
  justify-content: center;
}

.out {
  width: 300px;
  height: 80px;
  border-radius: 10px;
  background: #4f5478;
  opacity: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}
</style>
<style>
.el-message-box .el-message-box__btns button.save-map-confirm {
  background: #b04cf3;
}
.el-message-box .el-message-box__btns button.save-map-cancel {
  background: #7f86b9;
}
</style>
