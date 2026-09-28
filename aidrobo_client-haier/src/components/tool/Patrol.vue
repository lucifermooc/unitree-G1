<template>
  <div class="tooldetails">
    <div class="toolH">
      <div class="toolT">
        <img src="../../../static2/img/btn/xl.png" width="40px" />
        巡逻模式
      </div>
      <div class="toolC" @click="goBack()">        <img src="../../../static2/img/close.png" width="30px" />
</div>
    </div>
    <div class="content" v-if="$store.state.patrol_arr.length">
      <p v-for="(e, i) in $store.state.patrol_arr" :key="i" class="point">
        <img src="../../../static2/img/pointTool.png" width="20px" />
        <span>({{ e.x }},{{ e.y }})</span>
      </p>
      <p v-if="$store.state.patrol_arr.length && !active">已选择位置点</p>
      <p v-if="$store.state.patrol_arr.length && active">正在前往...</p>
    </div>
    <div class="content" v-else>
      <img src="../../../static2/img/dww.png" width="60px" />
      <span>请在左侧地图点击依次选择要巡逻的位置点（最多可选择6个点）</span>
    </div>
    <div
      class="toolBtn"
      @click="goActive()"
      v-if="$store.state.patrol_arr.length && active"
    >
      取消前往
    </div>
    <div
      class="toolBtn"
      @click="goActive(1)"
      v-if="$store.state.patrol_arr.length && !active"
    >
      确认前往
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
export default {
  data() {
    return { active: 0, pointIndex: 0 };
  },
  computed: {
    ...mapState(["tool", "robotPoint", "patrol_arr"])
  },
  watch: {
    robotPoint: function(n, o) {
      if(!this.patrol_arr.length){
        return
      }
      const point = this.patrol_arr[this.pointIndex];
      if ( Math.abs(point.x - n.x) < 0.2&&Math.abs(point.y - n.y)<0.2) {
        this.pointIndex+=1
        if(this.pointIndex== this.patrol_arr.length){
          this.pointIndex=0
        }
        this.goNext({ x: this.patrol_arr[this.pointIndex].x, y: this.patrol_arr[this.pointIndex].y });
      }
    },
    deep: true
  },
  mounted() {
    this.$store.state.tool = "xld";
  },
  methods: {
    goBack() {
      this.$store.state.tool = "";
      this.$store.state.patrol_arr = [];
      this.$store.state.patrol_chang_data = [];
      this.stop();
      this.$router.push({
        path: "/"
      });
    },
    stop() {
      const point = {
        header: {
          stamp: {
            sec: 0,
            nanosec: 0
          },
          frame_id: "map"
        },
        pose: {
          position: {
            x: this.$store.state.robotPoint.x,
            y: this.$store.state.robotPoint.y,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
          }
        }
      };
      var map_edit_msg = new ROSLIB.Message(point);
      TalkerPoint.publish(map_edit_msg);
    },
    goNext({ x, y }) {
      console.log("[ x,y ]-103", x, y);
      this.point = { x, y };
      const point = {
        header: {
          stamp: {
            sec: 0,
            nanosec: 0
          },
          frame_id: "map"
        },
        pose: {
          position: {
            x,
            y,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
          }
        }
      };
      var map_edit_msg = new ROSLIB.Message(point);
      TalkerPoint.publish(map_edit_msg);
    },
    goActive(e) {
      this.active = !this.active;
      const point = {
        header: {
          stamp: {
            sec: 0,
            nanosec: 0
          },
          frame_id: "map"
        },
        pose: {
          position: {
            x: this.robotPoint.x,
            y: this.robotPoint.y,
            z: 0.0
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: 0.0,
            w: 1.0
          }
        }
      };
      if (e) {
        point.pose.position = {
          x: this.patrol_arr[0].x,
          y: this.patrol_arr[0].y,
          z: 0.0
        };
      } else {
        this.goBack();
      }
      var map_edit_msg = new ROSLIB.Message(point);
      TalkerPoint.publish(map_edit_msg);
    }
  },
  components: {}
};
</script>

<style>
@import "../../../static2/css/public.css";
.content {
  display: flex;
  flex-flow: column;
  align-items: center;
  justify-content: center;
  padding: 2rem;
  line-height: 1.25rem;
  margin-top: 1rem;
}
.content img {
  margin-bottom: 1rem;
}
.point {
  display: flex;
  width: 8rem;
  justify-content: space-around;
}
.point > span {
  display: inline-block;
  width: 5rem;
  text-align: justify;
}
</style>
