<template>
  <div class="map" ref="map">
    <div class="fa_map_box1">
      <div class="map_box1" ref="map_box1"
        v-bind:style="{ transform: 'translate(' + left + 'px,' + top + 'px)' }">
        <img id="img1" src="../../static2/img/map2.png" @load="init" ref="img1" />
        <div class="robot" v-bind:style="{
          transform:
            'translate(' +
            (robotXY.x * scale - 6) +
            'px,' +
            (robotXY.y * scale - 6) +
            'px)',
        }">
        </div>
        <div class="charge" v-bind:style="{
          marginTop: 316 * scale - 10 + 'px',
          marginLeft: 483 * scale - 10 + 'px',
        }"></div>
        <div v-for="(item, index) in $store.state.patrol_chang_data" :key="index" class="map_point" v-bind:style="{
          transform:
            'translate(' +
            (item.x * scale - 25) +
            'px,' +
            (item.y * scale + -70) +
            'px)',
        }">
          <img src="../../static2/img/point.png" width="50px" />
          <span class="pointNum">{{ index + 1 }}</span>
        </div>
        <div class="map_box2">
          <canvas id="operate" ref="operate"></canvas>
        </div>
      </div>
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
export default {
  data () {
    return {
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
      patrol_arr_area: [],
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
      screen_w:
        window.innerWidth ||
        document.documentElement.clientWidth ||
        document.body.clientWidth,
      yEnd: 0,
      xEnd: 0
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
      "patrol_arr",
      "m_width",
      "m_height",
      "m_resolution",
      "_map_name",
      "zero",
      "stop_chang_data",
      "stop_point",
      "charge_po",
      "map_img_w"
    ])
  },
  mounted () {
    this.robotXY = { x: this.xx2(0.5), y: this.yy2(-2) };
    this.$store.state.map_width = this.$refs.map.offsetWidth;
  },
  methods: {
    ...mapMutations(["chang_data", "rubber_chang_data"]),
    init () {
      if (!this.init_img_data) {
        return false;
      }
      this.top = this.left = this.img2_top = this.img2_left = 0;
      this.scale = 1;
      this.$refs.operate.style.transform = "scale(" + this.scale + ")";
      let img1 = document.getElementById("img1");
      let operate = document.getElementById("operate");
      this.$store.state.x_can = operate;
      this.operate_txc = operate.getContext("2d");
      // 获取的图片进行等比适配
      this.$store.state.map_img_w = this.d_width = operate.width = img1.width;
      this.d_height = operate.height = img1.height;
    },
    xx_yy (a) {
      return Math.round(
        ((a / this.d_width) * this.m_width) / this.m_resolution
      );
    },
    yy2 (y) {
      return 992 - (y + 25) / 0.05;
    },
    xx2 (x) {
      return (x + 25) / 0.05;
    },
    xx0 (a) {
      return Math.round(a * 0.05 - 25);
    },
    yy0 (a) {
      return Math.round((992 - a) * 0.05 - 25);
    },
  }
};
</script>

<style lang="less" scoped>
.map {
  position: relative;
  top: 0;
  height: calc(100% - 70px);
  width: 1380px;
  border-radius: 5px;
  background: #526CAD;
  overflow: hidden;

}

.map_box1 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
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
  width: 12px;
  height: 12px;
  border-radius: 50%;
  top: 0;
  left: 0;
  z-index: 11;
  background: linear-gradient(95deg,
      rgba(71, 84, 141, 0.64) 9%,
      rgba(53, 81, 119, 0.15) 86%,
      rgba(53, 92, 119, 0.14) 88%);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
}

.pointNum {
  display: inline-block;
  position: absolute;
  font-size: 1.5rem;
  color: #fff;
  top: 14px;
  left: 20px;
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
</style>
