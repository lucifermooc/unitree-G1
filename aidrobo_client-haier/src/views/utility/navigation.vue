<template>
  <div class="newMapBox">
    <ShowMap :navigationPoint="true" />
    <div class="right point">
      <div class="titleBox" v-if="$store.state.navigationMapPoints.length">
        <p><img src="@/assets/img/editMap/point.svg" />位置点</p>
        <div class="navigationPointsList">
          <div
            v-for="(item, index) in navigationMapPoints"
            :key="index"
            class="navigationPointsList-item"
            @click="onStart(item, index)"
          >
            <span>{{ index + 1 }}：{{ item.name }}</span>
          </div>
        </div>
      </div>
      <div
        v-show="['navigateStart', 'navigatePause'].includes(actionStatus)"
        class="goPoint"
        @click="onClose"
      >
        关闭任务
      </div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/pointMap";
import { mapState } from "vuex";
import fullscreenLoading from "@/components/fullscreenLoading";

export default {
  components: {
    ShowMap
  },
  data() {
    return {};
  },
  computed: {
    ...mapState(["navigationMapPoints", "actionStatus"])
  },
  mounted() {
    let loading = fullscreenLoading();
    // 状态机
    // this.$store.state.actionStatus='point'
    const type = new ROSLIB.ServiceRequest({
      action: "patrol"
    });
    robotMode.callService(
      type,
      result => {
        loading.close();
        if (result.message !== "ok") {
          this.$message("状态切换失败");
        }
        console.log("[ robotMode OK]-61", result);
      },
      result => {
        loading.close();
        this.$message("状态切换失败");
        console.log("[ robotMode ERR]-61", result);
      }
    );
  },
  methods: {
    onStart(point, index) {
      this.$confirm(
        `<p style="line-height: 48px">将要导航到位置点${
          point.name ? `“${point.name}”` : (index + 1)
        }，请确认是否执行操作。</p>`,
        `定点导航`,
        {
          dangerouslyUseHTMLString: true,
          center: true,
          confirmButtonText: "是",
          confirmButtonClass: "recovery-confirm",
          cancelButtonText: "否",
          cancelButtonClass: "recovery-cancel"
        }
      ).then(() => {
        if (this.$store.getters.actionNeedCancel) {
          // 先取消上次的巡逻或导航，在开始本次的导航
          const type = new ROSLIB.ServiceRequest({
            cmd: "cancel"
          });
          patrolState.callService(
            type,
            res => {
              console.log("[ patrol_control cancel ok]-61", res);
              this.onStartNavigation(point);
            },
            res => {
              console.log("[ patrol_control cancel ERR]-61", res);
            }
          );
        } else {
          this.onStartNavigation(point);
        }
      });
    },
    onStartNavigation(point) {
      const msg = new ROSLIB.Message({
        header: {
          stamp: { sec: 0, nanosec: 0 },
          frame_id: "map"
        },
        pose: {
          position: {
            x: point.x,
            y: point.y,
            z: 0.0
          },
          // orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0},
          orientation: point.orientation
        }
      });
      StartNavigation.publish(msg);
    },
    onClose() {
      const type = new ROSLIB.ServiceRequest({
        cmd: "cancel"
      });
      patrolState.callService(
        type,
        res => {
          console.log("[ patrol_control cancel ok]-61", res);
        },
        res => {
          console.log("[ patrol_control cancel ERR]-61", res);
        }
      );
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

  & > p {
    text-align: center;
    text-align: center;
    line-height: 50px;
    padding: 0px 40px;
  }
}

.titleBox {
  height: 600px;
  width: 100%;
  display: flex;
  flex-direction: column;
  text-align: center;
  padding: 0 40px;
  margin: 55px 0;
  line-height: 50px;

  p:first-child {
    display: flex;
    align-items: center;
    justify-content: space-evenly;
    font-size: 50px;
  }
}

.navigationPointsList {
  margin-top: 20px;
  overflow-y: auto;

  &-item {
    width: 319px;
    height: 100px;
    border-radius: 10px;
    opacity: 1;
    background: #2f3758;
    backdrop-filter: blur(10px);
    display: flex;
    align-items: center;
    justify-content: center;
    margin: 20px auto 0;
    padding: 0 10px;

    span {
      width: 100%;
      max-height: 100%;
      line-height: 50px;
      overflow: hidden;
      word-break: break-all;
      text-overflow: ellipsis;
      line-clamp: 2;
      display: -webkit-box;
      -webkit-line-clamp: 2;
      -webkit-box-orient: vertical;
    }
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

.readyBtn {
  background: linear-gradient(
    110deg,
    rgba(55, 89, 238, 0.64) 11%,
    rgba(30, 157, 244, 0.37) 89%
  ) !important;
}
</style>
