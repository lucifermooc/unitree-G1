<template>
  <div class="map" ref="map" @touchstart="rubberstart" @touchmove="map_move">
    <div class="fa_map_box1">
      <div
        class="map_box1"
        ref="map_box1"
        v-bind:style="{ transform: 'translate(' + left + 'px,' + top + 'px)' }"
      >
        <img id="img1" :src="mapData.src" @load="init" ref="img1" />
        <!-- <img id="img1" src="../../../static2/img/map2.png" @load="init" ref="img1" />-->
        <div
          class="robot"
          v-bind:style="{
            transform:
              'translate(' +
              (robotXY.x * scale - 15) +
              'px,' +
              (robotXY.y * scale - 15) +
              'px)',
          }"
        ></div>
        <div
          v-for="(item, index) in patrol_arr_area"
          :key="index"
          class="map_point"
          v-bind:style="{
            transform:
              'translate(' +
              (item.x * scale - 25) +
              'px,' +
              (item.y * scale + -70) +
              'px)',
          }"
        >
          <img src="../../../static2/img/point.png" width="35px" />
          <span class="pointNum">{{ index + 1 }}</span>
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
          'margin-left': img2_left + 'px',
        }"
      ></div>
      <img id="img2" :src="mapData.src" ref="img2" />
    </div>
    <div class="zoom">
      <img src="@/assets/img/seeMap/fda.png" @click="zoom('f')" />
      <img src="@/assets/img/seeMap/sxiao.png" @click="zoom('s')" />
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState } from "vuex";
import { changeStr, mapToImg } from "@/assets/common"

export default {
  props: ["initData"],
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
    ])
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
    this.$store.state.tool = ''
    this.$store.state.map_width = this.$refs.map.offsetWidth;
    this.getMap()
    this.getPoint();
  },
  methods: {
    getPoint () {
      const msg2 = new ROSLIB.ServiceRequest(
          {
            map_id: Number(this.$store.state.nowMap.id),
            data_type: 'waypoint'
          }
      );
      getMapLinkedDataList.callService(msg2, (result) => {
        if (result.success) {
          let msg = []
          try {
            msg = JSON.parse(result.message)
            this.$store.state.patrol_arr = JSON.parse(msg[0].point_list);
            this.$store.state.patrol_arr_area = this.patrol_arr.map(e => {
              return { x: mapToImg({ mapData: this.mapData, x: e.x }), y: mapToImg({ mapData: this.mapData, y: e.y }) }
            })
          } catch (error) {
            this.$message('获取巡逻点失败');
          }
        } else {
          this.$message('获取巡逻点失败');
        }
      }, (result) => {
        console.log('[  getPoint ERR]-61', result)
      });
    },
    getMap() {
      const msg = new ROSLIB.ServiceRequest({
        id: this.$store.state.nowMap.id * 1
      });
      getMapImage.callService(msg, (res) => {
        console.log('[ getMapImage OK]-61', res)
        if (res.success) {
          this.mapData = changeStr(res.map)
        }
      }, (result) => {
        console.log('[ getMapImage ERR]-61', result)
      });


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
        const container_w = this.$refs.map.offsetWidth;
        let scale = this.scale;
        scale -= 0.1;
        let img_w_new = this.d_width * scale;
        if(img_w_new < container_w) return;
        this.scale = scale;
        this.$refs.map_box1.style.transition = "transform 1s";
        this.img2_scale += 0.025;

        img_w = this.d_width * this.scale;
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

      const sc = this.$refs.map.offsetWidth / this.mapData.width;
      sc < 1 && (this.scale = sc)
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
      this.$refs.map_box1.style.transition = "none";
      let ctx = this.operate_txc;
      e != undefined
        ? (this.touch_data = e.touches[0])
        : (this.touch_data = this.touch_data);
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
      e != undefined
        ? (this.touch_data = e.touches[0])
        : (this.touch_data = this.touch_data);
    },
    map_move(e) {
      try {
        if (this.tool == "") {
          e.preventDefault();
          e != undefined
              ? (this.touch_data = e.touches[0])
              : (this.touch_data = this.touch_data);
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
          this.top = this.top + cT*2*this.scale;
          this.left = this.left + cL*2*this.scale;
          this.img2_top -= (show_img_h / this.$refs.map.offsetHeight) * cT; //小地图边界判断(通过比例值获取)
          this.img2_left -= (show_img_w / this.$refs.map.offsetWidth) * cL;


          if(this.$refs.img1.height < this.$refs.map.offsetHeight) {
            // y方向小图
            this.top = Math.min(this.top, this.$refs.map.offsetHeight - this.$refs.img1.height);
            this.top = Math.max(this.top, 0);
          } else {
            // y方向大图
            this.top = Math.max(this.top, this.$refs.map.offsetHeight - this.$refs.img1.height);
            this.top = Math.min(this.top, 0);
          }

          if(this.$refs.img1.width < this.$refs.map.offsetWidth) {
            // x方向小图
            this.left = Math.min(this.left, this.$refs.map.offsetWidth - this.$refs.img1.width);
            this.left = Math.max(this.left, 0);
          } else {
            // x方向大图
            this.left = Math.max(this.left, this.$refs.map.offsetWidth - this.$refs.img1.width);
            this.left = Math.min(this.left, 0);
          }
        }
      } catch (e) {
      }
    },
  }
};
</script>

<style scoped>
.map {
  position: relative;
  top: 0;
  height: 100%;
  width: 100%;
  border-radius: 5px;
  background: #526cad;
  overflow: hidden;
}

.map_box1 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
}

.map_box2 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 2;
}

#operate {
  transform-origin: left top;
}

.img2 {
  position: absolute;
  right: 0;
  opacity: 0;
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

.pointNum {
  display: inline-block;
  position: absolute;
  font-size: 18px;
  color: #000;
  top: 3px;
  left: -8px;
  width: 50px;
  text-align: center;
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

.zoom {
  position: fixed;
  bottom: 30px;
  left: 50%;
  transform: translateX(-50%);
}
.zoom img {
  width: 86px;
  height: 86px;
}
</style>
