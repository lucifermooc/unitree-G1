<!-- jrf -->
<template>
  <div class="map" ref="map" @pointermove="map_move($event)">
    <div class="fa_map_box1">
      <div
        class="map_box1"
        ref="map_box1"
        @pointerdown="rubberstart($event)"
        @pointermove="rubbermove($event)"
        @pointerup="rubberend($event)"
        @pointercancel="rubbercancel($event)"
        v-bind:style="{ transform: 'translate(' + left + 'px,' + top + 'px)' }"
      >
        <img id="img1" :src="mapData.src" @load="init" ref="img1" />
        <!-- <img id="img1" src="../../../static2/img/map2.png" @load="init" ref="img1" /> -->
        <!-- 充电桩       -->
        <div
            v-if="dockPosePosition"
            class="dockPose"
            v-bind:style="{
            transform:
              'translate(' +
              (dockPosePosition.x * scale - 50) +
              'px,' +
              (dockPosePosition.y * scale - 60) +
              'px)'
          }"
        >
          <img src="@/assets/img/battery_charge.svg" width="70px" />
        </div>
        <!--  机器人      -->
        <div
          class="robot"
          v-bind:style="{
            transform:
              'translate(' +
              (robotXY.x * scale - 15) +
              'px,' +
              (robotXY.y * scale - 15) +
              'px)'
          }"
        ></div>
        <!--  点位      -->
        <div
          v-for="(item, index) in pointsInMapImage"
          :key="index"
          class="map_point"
          v-bind:style="{
            transform:
              'translate(' +
              (item.x * scale - 25) +
              'px,' +
              (item.y * scale + -70) +
              'px)'
          }"
        >
          <img src="../../../static2/img/point.png" width="50px" />
          <span class="pointNum">{{ index + 1 }}</span>
          <!-- 点位的角度 -->
          <div v-if="typeof item.deg === 'number'" class="pointArrow" :style="{transform: `rotate(${-item.deg}deg)`}"></div>
        </div>
        <div class="map_box2">
          <canvas id="operate" ref="operate"></canvas>
        </div>
      </div>
    </div>
    <div class="img2">
      <div
        class="show_img"
        ref="show_img"
        v-bind:style="{
          'margin-top': img2_top + 'px',
          'margin-left': img2_left + 'px'
        }"
      ></div>
      <img id="img2" :src="mapData.src" ref="img2" />
    </div>
    <div class="zoom">
      <img src="@/assets/img/seeMap/fda.png" @click="zoom('f')" />
      <img src="@/assets/img/seeMap/sxiao.png" @click="zoom('s')" />
    </div>
    <div class="active" v-if="!navigationPoint">
      <img
        src="@/assets/img/seeMap/active.png"
        @click="changeTool('')"
        v-if="tool == 'patrol' || tool == 'point'"
      />
      <img
        src="@/assets/img/seeMap/disActive.png"
        @click="changeTool('patrol')"
        v-else
      />
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
import {changeStr, mapToImg, imgToMap, checkPointPlacement} from "@/assets/common"
import {quatToDegrees} from "../../assets/common";
import { clampMapOffset } from "@/utils/map";

export default {
  props: ["initData", 'navigationPoint'],
  data() {
    return {
      mapData: {
        src: "",
        width: 1930,
        height: 3909,
        resolution: 0,
        positionX: 0,
        positionY: 0,
      },
      robotXY: { x: 0, y: 0 },
      scale: 1,
      left: 0,
      top: 0,
      img2_scale: 1,
      img2_top: 0,
      img2_left: 0,
      imgsrc: null,
      d_width: null,
      d_height: null,
      operate_txc: null,
      interval: null, //全局控制定时器
      touch_data: null, //触摸点
      screen_w: 1380,
      yEnd: 0,
      xEnd: 0,
      linearCurveArr: [], // 禁行线
      linearCurveArrP: [], // 禁行线 map point 点位
      dockPosePosition: null
    };
  },
  computed: {
    ...mapState([
      "robotPoint",
      "mcode",
      "rubber_data1",
      "rubber_data2",
      "head_h",
      "rubber_size",
      "go_rubber",
      "tool",
      "mapSrc",
      "init_img_data",
      "patrol_arr_area",
      "patrol_arr",
      "m_width",
      "m_height",
      "m_resolution",
      "_map_name",
      "zero",
      "stop_chang_data",
      "stop_point",
      "charge_po",
      "map_img_w",
      "navigationImgPoints",
    ]),
    pointsInMapImage () {
      return this.$props.navigationPoint ? this.navigationImgPoints : this.patrol_arr_area
    }
  },
  watch: {
    robotPoint: function (n) {
      this.robotXY = { x: mapToImg({ mapData: this.mapData, x: n.x }), y: mapToImg({ mapData: this.mapData, y: n.y }) }
    },
    initData: function (n) {
      if (n) {
        this.$store.state.patrol_arr_area = this.patrol_arr.map(e => {
          return { x: mapToImg({ mapData: this.mapData, x: e.x }), y: mapToImg({ mapData: this.mapData, y: e.y }) }
        })
      }
    }
  },
  mounted() {
    this.$store.state.map_width = this.$refs.map.offsetWidth;
    this.getMap()
  },
  methods: {
    changeTool(type) {
      if (!type) {
        this.$store.state.tool = ''
      } else {
        if (window.location.hash.includes('patrol')) {
          this.$store.state.tool = 'patrol'
        } else {
          this.$store.state.tool = 'point'
        }
      }

    },
    inputPoint(event) {
      if (event && event.touches && event.touches.length) {
        return event.touches[0];
      }
      if (event && event.changedTouches && event.changedTouches.length) {
        return event.changedTouches[0];
      }
      return event;
    },
    getMap() {
      const msg = new ROSLIB.ServiceRequest({
        id: this.$store.state.nowMap.id * 1
      });
      console.log('getMapImage', msg)
      getMapImage.callService(msg, (res) => {
        console.log('[ getMapImage OK]-61', res)
        if (res.success) {
          this.mapData = changeStr(res.map)
          this.$props.navigationPoint && this.getNavigationPoints();
          this.getForbidden();
          this.getDockPose();
        }
      }, (result) => {
        console.log('[ getMapImage ERR]-61', result)
      });

    },
    getForbidden() {
      const msg2 = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id,
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
                this.linearCurveArrP = msg;
                // this.$store.state.linearCurveArrP = msg;
                this.linearCurveArr = msg.map((e, i) => {
                  return [{
                    x: mapToImg({mapData: this.mapData, x: e.start.x}),
                    y: mapToImg({mapData: this.mapData, y: e.start.y})
                  },
                  {
                    x: mapToImg({mapData: this.mapData, x: e.end.x}),
                    y: mapToImg({mapData: this.mapData, y: e.end.y})
                  }]
                })
                console.log(this.linearCurveArr, msg)
                this.initBarrier()
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
    // 充电桩位置
    getDockPose() {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id,
      });
      getDockPoseService.callService(
          msg,
          result => {
            console.log("[  getDockPoseService result]-61", result);
            if (result.success) {
              this.dockPosePosition = {
                x: mapToImg({mapData: this.mapData, x: result.point.position.x}),
                y: mapToImg({mapData: this.mapData, y: result.point.position.y})
              }
              console.log(this.dockPosePosition)
            }
          },
          result => {
            console.log("[  getDockPoseService ERR]-61", result);
          }
      );
    },
    zoom(type){
      let img_w = this.$refs.img1.width;
      let img_h = this.$refs.img1.height;
      let left = this.left;
      let top = this.top;
      let s_h = top / img_h;
      let s_w = left / img_w;

      if(type==='f'){
        if (this.scale < 20) {
          this.$refs.map_box1.style.transition = "transform 1s";
          this.scale += 0.1;
          this.img2_scale -= 0.025;
          img_w = this.d_width * this.scale;
          this.$refs.img1.width = img_w;
          img_h = this.$refs.img1.height;
          this.$refs.operate.style.transform = "scale(" + this.scale + ")";
          this.$refs.show_img.style.width =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) / img_w + "px";
          this.$refs.show_img.style.height =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) / img_h +
            "px";

          this.top = s_h * img_h;
          this.left = s_w * img_w;
        }
      }else{
        if (img_w > this.screen_w) {
          this.$refs.map_box1.style.transition = "transform 1s";
          this.scale > 1 ? (this.scale -= 0.1) : (this.scale = 1);
          this.img2_scale += 0.025;
          img_w < this.screen_w
            ? (img_w = this.screen_w)
            : (img_w = this.d_width * this.scale);
          this.$refs.img1.width = img_w;
          img_h = this.$refs.img1.height;
          this.$refs.operate.style.transform = "scale(" + this.scale + ")";
          this.$refs.show_img.style.width =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) / img_w + "px";
          this.$refs.show_img.style.height =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) / img_h +
            "px";
          if (this.scale == 1) {
            this.img2_left = this.img2_top = this.left = this.top = 0;
          }
        }
      }
    },
    touchStart(e, n) {
      this.$store.state.init_img_data = false;
      this.$refs.map_box1.style.transition = "none";
      e.currentTarget.classList.add("cli_box");
    },
    touchend(e) {
      clearInterval(this.interval);
      this.interval = null;
      e.currentTarget.classList.remove("cli_box");
    },
    init() {
      this.top = this.left = this.img2_top = this.img2_left = 0;
      this.scale = 1;
      this.$refs.operate.style.transform = "scale(" + this.scale + ")";
      let img1 = document.getElementById("img1");

      const sc = 1380 / this.mapData.width
      sc > 1 && (this.scale = sc)
      img1.width = this.scale * this.mapData.width


      let operate = document.getElementById("operate");
      this.$store.state.x_can = operate;
      this.operate_txc = operate.getContext("2d");
      // 获取的图片进行等比适配
      this.$store.state.map_img_w = this.d_width = operate.width =this.mapData.width;
      this.d_height = operate.height =  this.mapData.height;
      this.$refs.img2.width = img1.width / 11;
      this.$refs.show_img.style.width = this.$refs.map.offsetWidth / 11 + "px";
      this.$refs.show_img.style.height =
        this.$refs.map.offsetHeight / 11 + "px";
    },
    rubberstart(e) {
      let set_time = 0;
      if (e.button !== undefined && e.button !== 0) {
        return;
      }
      this.$refs.map_box1.style.transition = "none";
      let ctx = this.operate_txc;
      this.touch_data = this.inputPoint(e) || this.touch_data;
      if (e.pointerId !== undefined && e.currentTarget.setPointerCapture) {
        e.currentTarget.setPointerCapture(e.pointerId);
      }
      if (this.tool == "") {
        // 地图滑动
        this.xEnd = Math.round(this.touch_data.pageX / this.scale);
        this.yEnd = Math.round(
          (this.touch_data.pageY - this.head_h) / this.scale
        );
      }
    },
    yy2(y) {
      return this.mapData.height - (y - this.mapData.positionY) / this.mapData.resolution;
    },
    xx2(x) {
      return (x - this.mapData.positionX) / this.mapData.resolution;
    },
    rubbermove(e) {
      let ctx = this.operate_txc;
      let r_x = Math.round((this.touch_data.pageX - this.left) / this.scale);
      let r_y = Math.round(
        (this.touch_data.pageY - this.head_h - this.top) / this.scale
      );
      e.preventDefault();
      this.touch_data = this.inputPoint(e) || this.touch_data;
    },
    rubberend(e) {
      this.touch_data = this.inputPoint(e) || this.touch_data;
      if (this.mcode <= 1 && (this.tool == "point" || this.tool == "patrol")) {
        this.patrol(e);
      }
    },
    rubbercancel(e) {
      if (e.pointerId !== undefined && e.currentTarget.releasePointerCapture &&
          e.currentTarget.hasPointerCapture(e.pointerId)) {
        e.currentTarget.releasePointerCapture(e.pointerId);
      }
    },
    map_move(e) {
      try {
        if (this.tool == "") {
          if (e) {
            e.preventDefault();
            this.touch_data = this.inputPoint(e);
          }
          let cT =
            Math.round((this.touch_data.pageY - this.head_h) / this.scale) -
            this.yEnd;
          let cL = Math.round(this.touch_data.pageX / this.scale) - this.xEnd;
          this.yEnd = Math.round(
            (this.touch_data.pageY - this.head_h) / this.scale
          );
          let show_img_w =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) /
            this.$refs.img1.width;
          let show_img_h =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) /
            this.$refs.img1.height;
          this.xEnd = Math.round(this.touch_data.pageX / this.scale);
          this.top = this.top + cT*10;
          this.left = this.left + cL*10;
          this.img2_top -= (show_img_h / this.$refs.map.offsetHeight) * cT; //小地图边界判断(通过比例值获取)
          this.img2_left -= (show_img_w / this.$refs.map.offsetWidth) * cL;
          this.top = clampMapOffset(
            this.top,
            this.$refs.map.offsetHeight - this.$refs.img1.height
          );
          this.img2_top = clampMapOffset(
            this.img2_top,
            this.$refs.img2.height - show_img_h
          );
          this.left = clampMapOffset(
            this.left,
            this.$refs.map.offsetWidth - this.$refs.img1.width
          );
          this.img2_left = clampMapOffset(
            this.img2_left,
            this.$refs.img2.width - show_img_w
          );
        }
      } catch (e) {
      }
    },
    patrol(e) {
      let circleX = Math.round(
        (this.touch_data.pageX - 30 - this.left) / this.scale
      );
      let circleY = Math.round(
        (this.touch_data.pageY - 150 - this.top) / this.scale
      );

      const imgPoint = { x: circleX, y: circleY };
      const mapPoint = {
        x: imgToMap({ mapData: this.mapData, x: circleX }),
        y: imgToMap({ mapData: this.mapData, y: circleY })
      };
      const checkError = checkPointPlacement(this, {
        imgPoint,
        mapPoint,
        lines: this.linearCurveArrP
      })
      if(checkError) return;

      if (this.tool == "point") {
        this.$store.state.patrol_arr_area = []
      }
      this.$store.state.patrol_arr_area.push({
        x: circleX,
        y: circleY
      });
      const xx_yy = this.patrol_arr_area.map(e => {
        return { x: imgToMap({ mapData: this.mapData, x: e.x }), y: imgToMap({ mapData: this.mapData, y: e.y }) };
      });
      this.$store.state.patrol_arr = xx_yy;
    },
    getNavigationPoints() {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id * 1,
        data_type: 'waypoint_node'
      });
      NavigationPointsGet.callService(
        msg,
        result => {
          try {
            let points = JSON.parse(result.message);
            let res = points.map(p => {
              p.point_list = JSON.parse(p.point_list)
              return {
                x: p.point_list.position.x,
                y: p.point_list.position.y,
                name: p.point_list.name,
                deg: quatToDegrees(p.point_list.orientation),
                orientation: p.point_list.orientation,
                id: p.id
              }
            })
            this.$store.state.navigationMapPoints = res;
            this.$store.state.navigationImgPoints = res.map(it => ({
              x: mapToImg({ mapData: this.mapData, x: it.x }),
              y: mapToImg({ mapData: this.mapData, y: it.y }),
              name: it.name,
              deg: it.deg,
              orientation: it.orientation,
              id: it.id
            }));
            console.log("[  NavigationPointsGet OK]-points", res);
          } catch (error) {
            console.log(error)
            this.$message("获取定点导航点位列表失败");
          }
          console.log("[  NavigationPointsGet OK]-61", result);
        },
        result => {
          console.log("[  NavigationPointsGet ERR]-61", result);
        }
      );
    },
    initBarrier (msg) {
      let operate = document.getElementById("operate");
      let ctx = operate.getContext("2d");
      let data = msg ? msg : this.linearCurveArr
      data.map((e) => {
        ctx.save();
        ctx.beginPath();
        ctx.strokeStyle = "red";
        ctx.lineWidth = "3";
        ctx.moveTo(e[0].x, e[0].y);
        ctx.lineTo(e[1].x, e[1].y);
        ctx.stroke();
        ctx.restore();
      })
      this.$refs.operate.style.transform = "scale(" + this.scale + ")";
    },
  }
};
</script>

<style scoped>
.map {
  position: relative;
  top: 0;
  height: calc(100% - 70px);
  width: 1380px;
  border-radius: 5px;
  background: #526cad;
  overflow: hidden;
  margin-top: 30px;
  margin-left: 30px;
}

.map_box1 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
  touch-action: none;
}

.loc_robot {
  position: absolute;
  top: 0;
  left: 0;
}

.map_box2 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 2;
}

.rubber_sel {
  position: absolute;
  width: 150px;
  height: 50px;
  text-align: center;
  line-height: 50px;
  border: 1px solid #948f8f;
  background: #409eff;
  color: #fff;
  border-radius: 11px;
  margin-left: 50px;
  margin-top: 30px;
}

.rubber_sel2 {
  background: #d9dbdb;
  color: #000;
}

#operate {
  transform-origin: left top;
}

#operate2 {
  position: absolute;
  top: 20px;
  left: 80px;
  z-index: 20;
}

.img2 {
  position: absolute;
  right: 0;
  opacity: 0.7;
}

.show_img {
  background: #615555;
  opacity: 0.5;
  border: 1px solid;
  position: absolute;
  margin-top: 0px;
  margin-left: 0px;
  transform-origin: left top;
}

.patrol_list {
  margin-left: 5%;
  margin-top: 1%;
  font-size: 12px;
  width: 83%;
}

.list-complete-item {
  width: 75px;
  display: inline-block;
  height: 25px;
  line-height: 25px;
  margin-left: 3px;
  margin-top: 3px;
  position: relative;
}

.map_tool {
  position: fixed;
  width: 170px;
  height: 120px;
  left: 20px;
  bottom: 20px;
  z-index: 5;
  display: flex;
  flex-wrap: wrap;
  align-content: space-between;
  justify-content: space-between;
}

.map_tool div {
  height: 50px;
  width: 50px;
  font-size: 30px;
  background: rgba(26, 26, 26, 0.6);
  color: #505050;
  box-shadow: 1px 1px 11px #415d5d;
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 6px;
  transition: transform 0.1s linear;
}

.cli_box {
  box-shadow: inset 1px 1px 11px #436c6c !important;
}

.tram1 {
  transform: translate(60px, 60px);
}

.tram2 {
  transform: translate(0px, 60px);
}

.tram3 {
  transform: translate(-60px, 60px);
}

.tram4 {
  transform: translate(60px, 0px);
}

.tram5 {
  z-index: 2;
  background: #709080 !important;
  transform: rotate(90deg);
}

.tram6 {
  transform: translate(-60px, 0px);
}

.tram7 {
  transform: translate(60px, -60px);
}

.tram8 {
  transform: translate(0px, -60px);
}

.tram9 {
  transform: translate(-60px, -60px);
}

.map_point {
  position: absolute;
  top: 0;
  left: 0;
}

.robot {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  top: 0;
  left: 0;
  z-index: 11;
  background: linear-gradient(
    135deg,
    rgb(255 172 85) 0%,
    rgb(255 13 52 / 81%) 100%
  );
  box-shadow: -1px -2px 3px -5px rgb(249 249 249),
    4px 4px 10px -5px rgb(0 0 0 / 30%);
}

.dockPose {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  top: 0;
  left: 0;
  z-index: 11;
  box-shadow: -1px -2px 3px -5px rgb(249 249 249),
    4px 4px 10px -5px rgb(0 0 0 / 30%);
}

.pointNum {
  display: inline-block;
  position: absolute;
  font-size: 1.5rem;
  color: #000;
  top: 10px;
  left: 0;
  width: 50px;
  text-align: center;
}

.pointArrow {
  --color: #b04cf3;
  position: relative;
  margin-left: 25px;
  width: 60px;
  height: 8px;
  background: var(--color);
  transform-origin: left center; /* 以左端为旋转中心 */
  transition: transform 0.3s ease;
}

/* 箭头尖端 */
.pointArrow::after {
  content: "";
  position: absolute;
  right: 0;
  top: 50%;
  transform: translateY(-50%) rotate(45deg);
  width: 20px;
  height: 20px;
  border-top: 8px solid var(--color);
  border-right: 8px solid var(--color);
}

.charge {
  width: 20px;
  height: 20px;
  background: #ccc;
  position: absolute;
  top: 0;
  left: 0;
  border-radius: 50%;
  animation: mymove 3s infinite;
}

@keyframes mymove {
  0% {
    transform: scale(1);
    opacity: 1;
  }

  100% {
    transform: scale(2);
    opacity: 0.1;
  }
}

.active {
  position: fixed;
  bottom: 50px;
  left: 50px;
}
.zoom {
  position: fixed;
  bottom: 50px;
  left: 1130px;
}
</style>
