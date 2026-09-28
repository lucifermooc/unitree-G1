<template>
  <div class="tooldetails">
    <div class="toolH">
      <div class="toolT">
        <img src="../../../static2/img/btn/yk.png" width="30px" />
        遥控模式
      </div>
      <div class="toolC" @click="goBack()">
        <img src="../../../static2/img/close.png" width="30px" />
      </div>
    </div>
    <div class="direction">
      <div
        v-for="(item, index) in items"
        :key="index"
        :class="'item' + index"
        @touchstart="touchStart($event, index)"
        @touchend="touchend($event)"
      >
        <img :src="item.name" :style="item.cla" width="30px" />
      </div>
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
import control from  "../../../static2/img/control.png";
import controlL from  "../../../static2/img/controlL.png";
import controlR from  "../../../static2/img/controlR.png";

export default {
  data() {
    return {
      items: [
        {
          name: control,
          clan: "tram2"
        },
        {
          name: controlL,
          clan: "tram4"
        },
        {
          name: control,
          cla: "transform: rotate(180deg);",
          clan: "tram8"
        },
        {
          name: controlR,
          clan: "tram6"
        }
      ]
    };
  },
  computed: {
    ...mapState(["tool"])
  },
  methods: {
    goBack() {
    this.$store.state.tool = "";
      this.$router.push({
        path: "/"
      });
    },
    touchStart(e, n) {
      e.currentTarget.classList.add("cli_box");
      const linear = {
        x: 0,
        y: 0.0,
        z: 0.0
      };
      const angular = {
        x: 0,
        y: 0.0,
        z: 0.0
      };
      if (n == 0) {
        linear.x = 0.5;
      } else if (n == 1) {
        // ←
        linear.x = 0.2;
        angular.z = 0.25;
      } else if (n == 2) {
        // 向下
        linear.x = -0.5;
      } else if (n == 3) {
        // →
        linear.x = 0.2;
        angular.z = -0.25;
      }
      var map_edit_msg = new ROSLIB.Message({ linear, angular });
      console.log('[ map_edit_msg ]-86', map_edit_msg)
      controlRobot.publish(map_edit_msg);
    },
    touchend(e) {
      e.currentTarget.classList.remove("cli_box");
      setTimeout(() => {
        var map_edit_msg = new ROSLIB.Message({
          linear: {
            x: 0,
            y: 0.0,
            z: 0.0
          },
          angular: {
            x: 0.0,
            y: 0.0,
            z: 0.0
          }
        });
        controlRobot.publish(map_edit_msg);
      }, 3000);
    }
  },
  components: {}
};
</script>

<style scoped>
@import "../../../static2/css/public.css";
.toolType {
  width: 100%;
  display: flex;
  flex-flow: column;
  align-items: center;
}
.toolType div {
  width: 80%;
  height: 50px;
  border-radius: 30px;
  background-color: #b26436;
  margin-top: 20px;
  text-align: center;
  font-size: 24px;
  line-height: 50px;
  color: #fff;
}
.direction {
  width: 13rem;
  display: flex;
  flex-wrap: wrap;
  align-items: center;
  justify-content: center;
}

.direction div {
  height: 3rem;
  width: 3rem;
  background: rgba(16, 34, 81, 0.47);
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  justify-content: center;
  align-items: center;
  transition: transform 0.1s linear;
}
.item0 {
  margin: 0px 4rem 20px 4rem;
}
.item2 {
  margin: 0 1rem;
}
.cli_box {
  box-shadow: inset 1px 2px 11px #076464 !important;
}
</style>
