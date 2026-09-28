<template>
  <div class="newMapBox">
    <ShowMap class="map" :toolType="toolType" :initData="initData" :navigationPoint="true" ref="mapRef" />
    <div class="right">
      <div
        v-show="point && !rubber && !stop"
        class="titleBox"
        style="margin-bottom: 10px;height: 540px;justify-content: unset;"
      >
        <p><img src="@/assets/img/editMap/point.svg" />位置点</p>
        <div
          v-if="isCurrentMap"
          class="saveCurrentBtn"
          @click="saveCurrentPosition"
        >
          保存当前位置
        </div>
        <div class="navigationPointsList">
          <div
            v-for="(item, index) in navigationMapPoints"
            :key="index"
            class="navigationPointsList-item"
            @click="deletePoint(item.id)"
          >
            <span>{{ index + 1 }}：{{ item.name }}</span>
          </div>
        </div>
      </div>
      <div class="point" @click="onPoint()" v-show="!point && !rubber && !stop">
        <img src="@/assets/img/editMap/point.svg" />
        <p>位置点</p>
      </div>
      <div v-show="rubber && !stop && !point" class="titleBox">
        <p><img src="@/assets/img/editMap/rubber.svg" />橡皮擦</p>
        <div class="rubberBtn" @click="clearRubber">清除未保存的橡皮擦</div>
        <p>请使用手指或鼠标进行擦除操作，地图可使用双指或滚轮中键拖动或缩放</p>
      </div>
      <div
        class="rubber"
        @click="onRubber()"
        v-show="!point && !rubber && !stop"
      >
        <img src="@/assets/img/editMap/rubber.svg" />
        <p>橡皮擦</p>
      </div>
      <div v-show="!rubber && stop && !point" class="stopLineContent titleBox">
        <p><img src="@/assets/img/editMap/stop.svg" />禁行线</p>
        <p style="margin: 15px 0">
          请使用手指或鼠标点击两点进行连线，地图可使用双指或滚轮中键拖动或缩放，点击列表可删除禁行线
        </p>
        <div class="stopLineList">
          <div
              v-for="(item, index) in linearCurveArrP"
              :key="index"
              class="stopLineListItem"
              @click="deleteStopLine(index)"
          >
            <span>禁行线 {{ index + 1 }}</span>
          </div>
        </div>
      </div>
      <div class="stop" @click="onStop()" v-show="!point && !rubber && !stop">
        <img src="@/assets/img/editMap/stop.svg" />
        <p>禁行线</p>
      </div>
      <div class="over" @click="onOver()">{{ overText }}</div>
<!--      <div class="out" @click="onOut()">退出</div>-->
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/edit";
import { mapState, mapMutations } from "vuex";

export default {
  components: {
    ShowMap
  },
  data() {
    return {
      overText: "保存地图",
      rubber: false,
      stop: false,
      point: false, // 管理位置点位的模式
      initData: false,
      toolType: "",
    };
  },
  computed: {
    ...mapState([
      "eraserArrP",
      "linearCurveArrP",
      "mapData",
      "navigationMapPoints",
      "nowMap"
    ]),
    isCurrentMap() {
      return String(this.nowMap.id) === String(this.$route.query.id);
    }
  },
  mounted() {
    this.$store.state.hasSave = false;
    this.$store.state.tool = "";
    this.resetLinearCurve();
    try {
      setTimeout(() => {
        this.getForbidden();
      }, 1000);
    } catch (error) {}
    // 状态机
    this.$store.state.actionStatus = "edit";
  },
  methods: {
    ...mapMutations(["resetLinearCurve"]),
    DrawMap(rectangle_array) {
      const msg2 = new ROSLIB.ServiceRequest({
        frame_id: "map",
        map_id: this.$route.query.id * 1,
        type: "point",
        data: [],
        rectangle_array
      });
      MapEditor.callService(
        msg2,
        result => {
          if (result.success) {
            console.log("[ msg ]-75", result);
          }
          console.log("[  DrawMap OK]-61", result);
        },
        result => {
          console.log("[  DrawMap ERR]-61", result);
        }
      );
    },
    DrawPicture(data) {
      const msg2 = new ROSLIB.ServiceRequest({
        frame_id: "map",
        map_id: this.$route.query.id * 1,
        type: "line",
        data,
        rectangle_array: []
      });
      DrawPicture.callService(
        msg2,
        result => {
          if (result.success) {
            console.log("[ msg ]-75", result);
          }
          console.log("[  DrawPicture OK]-61", result);
        },
        result => {
          console.log("[  DrawPicture ERR]-61", result);
        }
      );
    },
    getForbidden() {
      const msg2 = new ROSLIB.ServiceRequest({
        map_id: this.$route.query.id * 1
      });
      ForbiddenGet.callService(
        msg2,
        result => {
          if (result.success) {
            let msg = [];
            try {
              console.log("result.lines==>", typeof result.lines);
              if (result.lines.length) {
                msg = result.lines;
                this.initData = true;
                this.$store.state.linearCurveArrP = msg;
                this.hasHistory = true;
              }
            } catch (error) {
              this.$message("获取禁行线失败");
            }
          } else {
            this.$message("获取禁行线失败");
          }
          console.log("[  getForbidden OK]-61", result);
        },
        result => {
          console.log("[  getForbidden ERR]-61", result);
        }
      );
    },
    setForbidden(data) {
      const msg2 = new ROSLIB.ServiceRequest({
        map_id: this.$route.query.id * 1,
        frame_id: "map",
        lines: data
      });
      ForbiddenSet.callService(
        msg2,
        result => {
          console.log("[  setForbidden OK]-61", result);
        },
        result => {
          console.log("[  setForbidden ERR]-61", result);
        }
      );
    },
    deletePoint(id) {
      // this.$store.state.navigationMapPoints.splice(i, 1);
      // this.$store.state.navigationImgPoints.splice(i, 1);
      const msg = new ROSLIB.ServiceRequest({
        id,
        data_type: "waypoint_node",
      });
      NavigationPointDelete.callService(
        msg,
        result => {
          if (result.success) {
            console.log("[ NavigationPointDelete success ]-75", result);
            this.$refs.mapRef.getPoints();
          }
          console.log("[  NavigationPointDelete OK]-61", result);
        },
        result => {
          console.log("[  NavigationPointDelete ERR]-61", result);
        }
      );
    },
    deleteStopLine(index) {
      this.$refs.mapRef.deleteStopLine(index);
    },
    saveCurrentPosition() {
      this.$refs.mapRef.saveCurrentRobotPoint();
    },
    onRubber() {
      this.toolType = "eraser";
      this.rubber = true;
      this.overText = "完成";
      this.$store.state.tool = "";
    },
    clearRubber() {
      this.$refs.mapRef.clearRubber();
    },
    onPoint() {
      this.toolType = "point";
      this.point = true;
      this.overText = "完成";
      this.$store.state.tool = "";
    },
    onStop() {
      this.toolType = "stop";
      this.stop = true;
      this.overText = "完成";
      this.$store.state.tool = "";
    },
    onOver() {
      if (this.overText == '保存地图') {
        console.log('[ this.linearCurveArrP. ]-158', this.linearCurveArrP)
        console.log('[ this.eraserArrP. ]-158', this.eraserArrP)
        //禁行线
        if (this.hasHistory || this.linearCurveArrP.length) {
          this.setForbidden(this.linearCurveArrP);
          this.DrawPicture(this.linearCurveArrP);
        }
        if (this.eraserArrP.length) {
          this.DrawMap(this.eraserArrP)
        }
        this.$message("保存成功");
      } else {
        this.toolType = "";
        this.rubber = false;
        this.stop = false;
        this.point = false;
        this.$store.state.tool = "";
        this.overText = "保存地图";
      }
    },
    onOut() {
      this.$confirm(
        `<div>是否确认退出</div><div>（请确认所做操作已保存）</div>`,
        "退出编辑",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      )
        .then(() => {
          this.$router.push("/map");
          console.log("[  ]-69");
        })
        .catch(() => {
          console.log("[  ]-72");
        });
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
  justify-content: space-evenly;
  line-height: 50px;
  margin-left: 30px;
  margin-top: 30px;

  .titleBox {
    margin-bottom: 80px;
    height: 500px;
    width: 100%;
    display: flex;
    flex-direction: column;
    text-align: center;
    justify-content: space-between;
    padding: 0 40px;
    line-height: 50px;

    p:first-child {
      display: flex;
      align-items: center;
      justify-content: space-evenly;
      font-size: 50px;
    }
  }
}

.navigationPointsList {
  margin-top: 16px;
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

.saveCurrentBtn {
  margin: 20px auto 0;
  width: 319px;
  height: 80px;
  border-radius: 10px;
  opacity: 1;
  background: rgba(55, 89, 238, 0.64);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  align-items: center;
  justify-content: center;
  flex-shrink: 0;
}

.stopLineContent {
  margin-bottom: 20px!important;
  height: 560px!important;
  justify-content: flex-start!important;
}
.stopLineList {
  overflow-y: auto;

  .stopLineListItem {
    width: 319px;
    height: 80px;
    border-radius: 10px;
    opacity: 1;
    background: #2f3758;
    -webkit-backdrop-filter: blur(10px);
    backdrop-filter: blur(10px);
    display: -webkit-box;
    display: -ms-flexbox;
    display: flex;
    -webkit-box-align: center;
    -ms-flex-align: center;
    align-items: center;
    -webkit-box-pack: center;
    -ms-flex-pack: center;
    justify-content: center;
    margin: 20px auto 0;
    padding: 0 10px;
  }
}

.point,
.rubber,
.stop {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-evenly;
  width: 300px;
  height: 150px;
  border-radius: 20px;
  opacity: 1;
  background: linear-gradient(
    121deg,
    rgba(71, 84, 141, 0.64) 14%,
    rgba(53, 81, 119, 0.15) 90%,
    rgba(53, 92, 119, 0.14) 91%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.rubberBtn {
  margin: 0 auto;
  width: 380px;
  height: 100px;
  border-radius: 10px;
  opacity: 1;
  //box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  //background: #b04cf3;
  background: rgba(55, 89, 238, 0.64);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  align-items: center;
  justify-content: center;
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
