<template>
  <div class="map" ref="map">
    <div class="fa_map_box1">
      <div
        class="map_box1"
        ref="map_box1"
        v-bind:style="{ transform: 'translate(' + left + 'px,' + top + 'px)' }"
      >
        <img id="img1" :src="mapData.src" ref="img1" />
        <div
          class="robot"
          v-bind:style="{
            transform:
              'translate(' +
              (xx2(robotPoint.x) * scale - 15) +
              'px,' +
              (yy2(robotPoint.y) * scale - 15) +
              'px)'
          }"
          v-if="showType != 'see'"
        ></div>
        <div
          class="charge"
          v-bind:style="{
            transform:
              'translate(' +
              (chargeXY.x * scale - 46) +
              'px,' +
              (chargeXY.y * scale - 124) +
              'px)'
          }"
          v-else
        >
          <img
            id="img1"
            src="@/assets/img/startPoint.svg"
            @load="init"
            ref="img1"
          />
        </div>
      </div>
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
import { changeStr } from "@/assets/common"
export default {
  props: ['showType'],
  data () {
    return {
      mapData: {
        src: "",
        width: 0,
        height: 0,
        resolution: 0,
        positionX: 0,
        positionY: 0,
      },
      chargeXY: { x: 0, y: 0 },
      scale: 1,
      left: 0,
      top: 0,
    };
  },
  computed: {
    ...mapState([
      "robotPoint",
    ])
  },
  mounted () {
    this.$store.state.map_width = this.$refs.map.offsetWidth;
    this.getMap()
  },
  beforeDestroy () {
    if (this.showType !== 'see') {
      robotMap.unsubscribe();
    }
  },
  methods: {
    getMap () {
      if (this.showType === 'see') {
        const msg = new ROSLIB.ServiceRequest({
          id: this.$route.query.id * 1
        });
        getMapImage.callService(msg, (res) => {
          console.log('[ getMapImage OK]-61', res)
          if (res.success) {
            this.mapData = changeStr(res.map)
            this.chargeXY = { x: this.xx2(0), y: this.yy2(0) }
            this.init()
          }
        }, (result) => {
          console.log('[ getMapImage ERR]-61', result)
        });
      } else {
        robotMap.subscribe(res => {
          this.mapData = changeStr(res)
          this.init()
        })
      }
    },
    init () {
      let img1 = document.getElementById("img1");
      const sc = 1380 / this.mapData.width
      this.scale = sc
      img1.width = this.scale * this.mapData.width
      img1.height = this.scale * this.mapData.height
    },
    yy2 (y) {
      return this.mapData.height - (y - this.mapData.positionY) / this.mapData.resolution;
    },
    xx2 (x) {
      return (x - this.mapData.positionX) / this.mapData.resolution;
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
  background: #526cad;
  overflow: auto;
}

.map_box1 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
}

.robot {
  position: absolute;
  width: 30px;
  height: 30px;
  border-radius: 50%;
  top: 0;
  left: 0;
  z-index: 11;
  background: #ff9b44;
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
}

.charge {
  width: 92px;
  height: 124px;
  position: absolute;
  top: 0;
  left: 0;
}
</style>
