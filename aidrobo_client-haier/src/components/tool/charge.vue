<template>
  <div class="tooldetails">
    <div class="toolH">
      <div class="toolT">
        <img src="../../../static2/img/btn/cdz.png" width="30px" />回充返回
      </div>
      <div class="toolC" @click="goBack()">        <img src="../../../static2/img/close.png" width="30px" />
</div>
    </div>
    <div class="content">
      <img src="../../../static2/img/back.png" width="60px" />
      <span>正在返回...</span>
    </div>
    <div class="toolBtn" @click="goBack()">取消前往</div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
export default {
  data() {
    return {};
  },
  mounted() {
    this.goActive(1);
  },
  watch: {},
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
    goActive(e) {
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
            x: -1,
            y: 9,
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
    }
  },
  computed: {
    ...mapState([
      "tool",
      "x_can",
      "charge_po",
      "map_img_w",
      "m_width",
      "m_resolution",
      "charge_po_back"
    ])
  },
  components: {}
};
</script>

<style>
@import "../../../static2/css/public.css";
.orientation {
  width: 400px;
  margin-left: 40px;
}
</style>
