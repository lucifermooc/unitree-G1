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
        <div class="map_box2">
          <canvas id="operate" ref="operate"></canvas>
        </div>
        <!-- 机器人实时位置 -->
        <div
          v-if="isCurrentMap"
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
        <!-- 导航点位  -->
        <div
          v-for="(item, index) in navigationImgPoints"
          :key="`navigationImgPoints-${index}`"
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
          <div v-if="typeof item.deg === 'number'" class="pointArrow" :style="{transform: `rotate(${-item.deg}deg)`}"></div>
        </div>
        <!-- 禁行线序号, 固定显示在线段左边 -->
        <div
            v-for="([{x: x1, y: y1},{x: x2, y: y2}], index) in linearCurveArr"
            :key="`linearCurveArr-${index}`"
            class="stop_line_point"
            v-bind:style="{
            transform:
              'translate(' +
              ((x1 < x2? x1:x2) * scale - 45) +
              'px,' +
              ((x1 < x2? y1:y2) * scale + -25) +
              'px)'
          }"
        >
          <span class="stop_line_number">{{ index + 1 }}</span>
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
    <!-- <div class="recover">
      <img src="@/assets/img/editMap/revocation.png" @click="revocation()" />
      <img src="@/assets/img/editMap/recover.png" @click="recover()" />
    </div>-->
    <div
      class="active"
      v-if="toolType == 'stop' || toolType == 'eraser' || toolType == 'point'"
    >
      <img
        src="@/assets/img/seeMap/active.png"
        @click="changeTool('')"
        v-if="tool == 'stop' || tool == 'eraser' || tool == 'point'"
      />
      <img
        src="@/assets/img/seeMap/disActive.png"
        @click="changeTool(toolType)"
        v-else
      />
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState } from "vuex";
import {changeStr, mapToImg, imgToMap, checkPointPlacement} from "@/assets/common"
import {angleToQuaternion, quatToDegrees} from "../../assets/common";
import { clampMapOffset } from "@/utils/map";

export default {
  props: ['toolType', 'initData', 'navigationPoint'],
  data () {
    return {
      recoverArr: [],
      linearCurveArr: [],
      stop_chang_data: [],//禁行区的点
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
      items: [
        {
          name: "icon-zoom-in",
          cla: "",
          clan: "tram7"
        },
        {
          name: "icon-updown",
          cla: "transform: rotate(270deg);",
          clan: "tram2"
        },
        {
          name: "icon-zoom-out",
          cla: "",
          clan: "tram9"
        },
        {
          name: "icon-updown",
          cla: "transform: rotate(180deg);",
          clan: "tram4"
        },
        {
          name: "icon-updown",
          cla: "transform: rotate(90deg);",
          clan: "tram8"
        },
        {
          name: "icon-updown",
          cla: "",
          clan: "tram6"
        }
      ],
      screen_w: 1380,
      yEnd: 0,
      xEnd: 0
    };
  },
  computed: {
    ...mapState([
      "eraserArr",
      "eraserArrP",
      "linearCurveArrP",
      "robotPoint",
      "mapData",
      "head_h",
      "tool",
      "navigationImgPoints",
      "navigationMapPoints",
      "nowMap",
      "robotOrientation",
    ]),
    isCurrentMap() {
      return String(this.nowMap.id) === String(this.$route.query.id);
    },
  },
  watch: {
    robotPoint: function (n) {
      this.robotXY = { x: mapToImg({ mapData: this.mapData, x: n.x }), y: mapToImg({ mapData: this.mapData, y: n.y }) }
    },
    initData: function (n) {
      //斤新鲜会天
      if (n) {
        const arr = []
        this.linearCurveArr = this.linearCurveArrP.map((e, i) => {
          return [{
            x: mapToImg({ mapData: this.mapData, x: e.start.x }),
            y: mapToImg({ mapData: this.mapData, y: e.start.y })
          },
          {
            x: mapToImg({ mapData: this.mapData, x: e.end.x }),
            y: mapToImg({ mapData: this.mapData, y: e.end.y })
          }]
        })
        this.initBarrier()
      }
    }
  },
  mounted () {
    this.$store.state.map_width = this.$refs.map.offsetWidth;
    this.getMap()
  },
  methods: {
    quatToDegrees,
    changeTool (type) {
      this.$store.state.tool = type
    },
    inputPoint (event) {
      if (event && event.touches && event.touches.length) {
        return event.touches[0];
      }
      if (event && event.changedTouches && event.changedTouches.length) {
        return event.changedTouches[0];
      }
      return event;
    },
    getMap () {
      const msg = new ROSLIB.ServiceRequest({
        id: this.$route.query.id * 1
      });
      getMapImage.callService(msg, (res) => {
        console.log('[ getMapImage OK]-61', res)
        if (res.success) {
          this.$store.state.mapData = changeStr(res.map)
          this.$props.navigationPoint && this.getPoints();
        }
      }, (result) => {
        console.log('[ getMapImage ERR]-61', result)
      });

    },
    getPoints() {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$route.query.id * 1,
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
                deg: quatToDegrees(p.point_list.orientation),
                orientation: p.point_list.orientation,
                name: p.point_list.name,
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
    touchStart (e, n) {
      this.$refs.map_box1.style.transition = "none";
    },
    touchend (e) {
      clearInterval(this.interval);
      this.interval = null;
      e.currentTarget.classList.remove("cli_box");
    },
    init () {
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
      this.d_width = operate.width = this.mapData.width;
      this.d_height = operate.height = this.mapData.height;
      this.$refs.operate.style.transform = "scale(" + this.scale + ")";
      this.$refs.img2.width = img1.width / 11;
      this.$refs.show_img.style.width = this.$refs.map.offsetWidth / 11 + "px";
      this.$refs.show_img.style.height = this.$refs.map.offsetHeight / 11 + "px";
    },
    circleXY (n) {
      let circleX = Math.round(
        (this.touch_data.pageX - 30 - this.left) / this.scale
      );
      let circleY = Math.round((this.touch_data.pageY - 150 - this.top) / this.scale);
      if (n === 'x') {
        return circleX
      } else {
        return circleY
      }
    },
    rubberstart (e) {
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
      } else if (this.tool == "eraser") {
        let ctx = this.operate_txc;
        ctx.save();
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(this.circleXY('x') - 5, this.circleXY('y') - 5, 10, 10);
        this.$store.state.eraserArr.push({
          x: this.circleXY('x'),
          y: this.circleXY('y')
        })
        this.$store.state.eraserArrP.push(this.setPointData({
          x: this.circleXY('x'),
          y: this.circleXY('y')
        }))
      }
    },
    rubbermove (e) {
      e.preventDefault();
      this.touch_data = this.inputPoint(e) || this.touch_data;
      if (this.tool == "eraser") {
        let ctx = this.operate_txc;
        ctx.save();
        ctx.fillStyle = "#ffffff";
        ctx.fillRect(this.circleXY('x') - 5, this.circleXY('y') - 5, 10, 10);
        this.$store.state.eraserArr.push({
          x: this.circleXY('x'),
          y: this.circleXY('y')
        })
        this.$store.state.eraserArrP.push(this.setPointData({
          x: this.circleXY('x'),
          y: this.circleXY('y')
        }))
      }
    },
    rubberend (e) {
      this.touch_data = this.inputPoint(e) || this.touch_data;
      if (this.tool == "stop") {
        this.barrier()
      } else if(this.tool === 'point') {
        const _this = this;
        // 使用延时是因为点击弹框的取消按钮区域时会误触
        setTimeout(() => {
          _this.setPoint()
        }, 50)
      }
    },
    rubbercancel (e) {
      if (e.pointerId !== undefined && e.currentTarget.releasePointerCapture &&
          e.currentTarget.hasPointerCapture(e.pointerId)) {
        e.currentTarget.releasePointerCapture(e.pointerId);
      }
    },
    map_move (e) {
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
          this.top = this.top + cT * 10;
          this.left = this.left + cL * 10;
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
    //<撤销
    revocation () {
      const del = this.linearCurveArr.splice(-1, 1)
      console.log('[ del ]-452', del)
      this.recoverArr.push(del[0])
      let ctx = this.operate_txc;
      ctx.clearRect(0, 0, this.d_width, this.d_height);
      this.initBarrier()
      this.$store.state.linearCurveArrP = this.setLineData(this.linearCurveArr)
    },
    //恢复>
    recover (e) {
      console.log('[ recover ]-458',)
      const del = this.recoverArr.splice(-1, 1)
      this.linearCurveArr.push(del[0])
      console.log('[  ]-461', this.linearCurveArr)
      let ctx = this.operate_txc;
      ctx.clearRect(0, 0, this.d_width, this.d_height);
      this.initBarrier()
      this.$store.state.linearCurveArrP = this.setLineData(this.linearCurveArr)
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
    },
    barrier () {
      let ctx = this.operate_txc;
      ctx.save();
      ctx.beginPath();
      ctx.fillStyle = "red";
      ctx.arc(this.circleXY('x'), this.circleXY('y'), 2, 0, 2 * Math.PI);
      ctx.fill();
      ctx.restore();
      this.stop_chang_data.push({
        x: this.circleXY('x'),
        y: this.circleXY('y')
      });
      if (this.stop_chang_data.length >= 2) {
        ctx.save();
        ctx.beginPath();
        ctx.strokeStyle = "red";
        ctx.lineWidth = "1";
        ctx.moveTo(this.stop_chang_data[0].x, this.stop_chang_data[0].y);
        ctx.lineTo(this.circleXY('x'), this.circleXY('y'));
        ctx.stroke();
        ctx.restore();
        this.linearCurveArr.push(this.stop_chang_data)
        this.$store.state.linearCurveArrP = this.setLineData(this.linearCurveArr)
        this.stop_chang_data = [];
      }
    },
    setPoint() {
      const imgPoint = {
        x: this.circleXY('x'),
        y: this.circleXY('y')
      }
      const mapPoint = {
        x: imgToMap({ mapData: this.mapData, x: imgPoint.x }),
        y: imgToMap({ mapData: this.mapData, y: imgPoint.y }),
        z: 0.0
      }
      const checkError = checkPointPlacement(this, {
        imgPoint,
        mapPoint,
        lines: this.linearCurveArrP
      })
      if(checkError) return;

      const addInp = document.querySelector("#addInp");
      addInp && (addInp.value = "");
      const degInp = document.querySelector("#degInp");
      degInp && (degInp.value = undefined);

      const h = this.$createElement;
      this.$confirm(
        h('div', null, [
          h('div', null, [
            '名称',
            h('input', {
              attrs: {
                id: 'addInp',
                placeholder: '请输入点位名称',
                autocomplete: 'off'
              },
              style: 'height: 70px;width: 405px;margin-left: 18px;padding: 0 15px;',
            })
          ]),
          h('div', null, [
            '角度',
            h('input', {
              attrs: {
                id: 'degInp',
                placeholder: '请输入点位角度(0~360)',
                autocomplete: 'off',
                type: 'number',
                min: 0,
                max: 360
              },
              style: 'height: 70px;width: 405px;margin-left: 18px;padding: 0 15px;',
              on: {
                change: (e) => {
                  let val = e.target.value;
                  val = Math.max(val, 0);
                  val = Math.min(val, 360);
                  e.target.value = val;
                }
              }
            })
          ])
        ]),
        "点位信息",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      ).then(() => {
        const addInp = document.querySelector("#addInp");
        const degInp = document.querySelector("#degInp");
        const pointName = addInp.value;
        const pointDeg = degInp.value;
        addInp.value = '';
        degInp.value = undefined;
        this.addNavigationPoint({
          mapPoint,
          orientation: angleToQuaternion(Number(pointDeg)),
          pointName
        });
      })
    },
    saveCurrentRobotPoint() {
      if (!this.isCurrentMap) {
        this.$message("仅当前地图可保存机器人当前位置");
        return;
      }
      const mapPoint = {
        x: Number(this.robotPoint.x),
        y: Number(this.robotPoint.y),
        z: 0.0
      }
      const imgPoint = {
        x: mapToImg({ mapData: this.mapData, x: mapPoint.x }),
        y: mapToImg({ mapData: this.mapData, y: mapPoint.y })
      }
      const orientation= this.robotOrientation

      const checkError = checkPointPlacement(this, {
        imgPoint,
        mapPoint,
        lines: this.linearCurveArrP
      })
      if(checkError) return;

      const saveCurrentInp = document.querySelector("#saveCurrentInp");
      saveCurrentInp && (saveCurrentInp.value = "");

      const h = this.$createElement;
      this.$confirm(
        h('div', null, [
          h('div', null, [
            '点位名称',
            h('input', {
              attrs: {
                id: 'saveCurrentInp',
                placeholder: '请输入点位名称',
                autocomplete: 'off'
              },
              style: 'height: 70px;width: 405px;margin-left: 18px;padding: 0 15px;',
            })
          ]),
          h('div', {
            style: 'font-size: 28px;line-height: 40px;margin-top: 20px;'
          }, [``])
        ]),
        "保存机器人当前位置",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      ).then(() => {
        const saveCurrentInp = document.querySelector("#saveCurrentInp");
        const pointName = String((saveCurrentInp && saveCurrentInp.value) || "").trim();
        if (!pointName) {
          this.$message("请输入点位名称");
          return;
        }
        saveCurrentInp.value = '';
        this.addNavigationPoint({
          mapPoint,
          orientation,
          pointName
        });
      })
    },
    addNavigationPoint({ mapPoint, orientation, pointName }) {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$route.query.id * 1,
        frame_id: "map",
        data_type: "waypoint_node",
        data: JSON.stringify({
          position: mapPoint,
          orientation,
          name: pointName
        }),
      });
      NavigationPointAdd.callService(
        msg,
        result => {
          if (result.success) {
            console.log("[ NavigationPointAdd success ]-75", result);
            this.getPoints();
          } else {
            this.$message("保存失败");
          }
          console.log("[  NavigationPointAdd OK]-61", result);
        },
        result => {
          this.$message("保存失败");
          console.log("[  NavigationPointAdd ERR]-61", result);
        }
      );
    },
    setLineData (arr) {
      return arr.map(e => {
        return {
          start: { x: imgToMap({ mapData: this.mapData, x: e[0].x }), y: imgToMap({ mapData: this.mapData, y: e[0].y }), z: 0.0 },
          end: { x: imgToMap({ mapData: this.mapData, x: e[1].x }), y: imgToMap({ mapData: this.mapData, y: e[1].y }), z: 0.0 }
        }
      })
    },
    setPointData(e) {
      return {
        center_point: { x: imgToMap({ mapData: this.mapData, x: e.x }), y: imgToMap({ mapData: this.mapData, y: e.y }), z: 0.0 },
        side_length: 0.5,
        grayscale: 0
      }
    },
    deleteStopLine(index) {
      this.linearCurveArr.splice(index, 1)
      this.$store.state.linearCurveArrP = this.setLineData(this.linearCurveArr)
      // 重新绘画
      let ctx = this.operate_txc;
      ctx.clearRect(0, 0, this.d_width, this.d_height);
      this.initBarrier()
      ctx.save();
      ctx.fillStyle = "#ffffff";
      this.$store.state.eraserArr.forEach(item => {
        ctx.fillRect(item.x - 5 , item.y - 5 , 10,10);
      })
      ctx.restore();
    },
    clearRubber() {
      let ctx = this.operate_txc;
      ctx.clearRect(0, 0, this.d_width, this.d_height);
      this.$store.state.eraserArr = [];
      this.$store.state.eraserArrP = [];
    }
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

.map_point {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 3;
}

.robot {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  top: 0;
  left: 0;
  z-index: 4;
  background: linear-gradient(
    135deg,
    rgb(255 172 85) 0%,
    rgb(255 13 52 / 81%) 100%
  );
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
  content: '';
  position: absolute;
  right: 0;
  top: 50%;
  transform: translateY(-50%) rotate(45deg);
  width: 20px;
  height: 20px;
  border-top: 8px solid var(--color);
  border-right: 8px solid var(--color);
}

.stop_line_point {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 3;
}

.stop_line_number {
  display: inline-block;
  position: absolute;
  font-size: 1.5rem;
  color: red;
  top: 10px;
  left: 0;
  width: 50px;
  text-align: center;
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

.recover {
  position: fixed;
  bottom: 160px;
  left: 1150px;
}

.active {
  position: fixed;
  bottom: 50px;
  left: 50px;
}
.zoom {
  position: fixed;
  bottom: 50px;
  left: 1160px;
}
</style>
