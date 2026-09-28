<template>
  <div class="homeBox">
    <router-link to="/utility">
      <div class="item">
        <img src="@/assets/img/home1.svg" />
        <p>应用功能</p>
      </div>
    </router-link>
    <router-link to="/map">
      <div class="item">
        <img src="@/assets/img/home2.svg" />
        <p>地图管理</p>
      </div>
    </router-link>
    <router-link to="/site">
      <div class="item">
        <img src="@/assets/img/home3.svg" />
        <p>设置</p>
      </div>
    </router-link>
    <div class="text">
      {{ text }}
    </div>
    <div class="logo">
      <img src="@/assets/img/logo.png" width="500px" />
    </div>
  </div>
</template>

<script>
export default {
  data() {
    return {
      text: process.env.VUE_APP_VERSION,
      showTc: true,
    };
  },
  beforeRouteEnter(to, from, next) {
    if(to && to.query && to.query.reload) {
      next(to.path)
      window.location.reload();
      return;
    }
    next();
  },
  mounted() {
    this.$store.state.tool = "";
    // 全局订阅机器人位置
    robotPosition.subscribe((message) => {
      if (message.pose) {
        this.$store.commit("updateRobotPose", message.pose);
      }
    });
    // 获取当前地图id
    getCurrentMapId.callService(
      null,
      (result) => {
        this.$store.state.nowMap = { id: result.map_id, name: result.map_name };

        console.log("[  finishMap OK]-61", result);
      },
      (result) => {
        console.log("[  finishMap ERR]-61", result);
      }
    );
    // 获取ip
    const msg = new ROSLIB.ServiceRequest();
    GetStrings.callService(
      msg,
      (result) => {
        this.$store.state.IP = result.result;
        console.log("[  get_ip OK]-61", result);
      },
      (result) => {
        console.log("[  get_ip ERR]-61", result);
      }
    );
    // 电量信息
    BatteryState.subscribe((result) => {
      // percentage 为 0 是有效值，不能用真值判断
      this.$store.state.percentage = result.percentage != null
        ? Math.ceil(result.percentage * 100)
        : 100;
      // 基于 charge 判断充电状态（带 5 次防抖）
      this.$store.commit('updateBatteryCharge', result.charge);
    });
  },
};
</script>

<style scoped>
.homeBox {
  display: flex;
  align-items: center;
  justify-content: space-around;
  width: 100%;
  height: calc(100% - 120px);
}

.item {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  font-size: 50px;
  color: #fff;
  top: 234px;
  width: 500px;
  height: 700px;
  border-radius: 20px;
  opacity: 1;
  background: linear-gradient(
    142deg,
    rgba(71, 84, 141, 0.64) 20%,
    rgba(53, 81, 119, 0.15) 94%,
    rgba(53, 92, 119, 0.14) 95%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
}

.text {
  font-size: 44px;
  position: absolute;
  bottom: 40px;
  left: 74px;
}

.logo {
  position: absolute;
  bottom: 30px;
  right: 30px;
}
</style>
