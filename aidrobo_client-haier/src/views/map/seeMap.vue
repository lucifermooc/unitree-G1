<template>
  <div class="newMapBox">
    <ShowMap :showType="'see'" />
    <div class="right">
      <div class="titleBox">
        <p>地图名称：</p>
        <p class="mapName">{{ $route.query.mapName }}</p>
      </div>
      <div class="iconBtn" @click="onUse()" v-if="!isUse">
        <img src="@/assets/img/seeMap/use.svg" />
        <p>使用地图</p>
      </div>
      <div class="iconBtn usemap" v-else>
        <img src="@/assets/img/seeMap/now.svg" />
        <p>当前地图</p>
      </div>
      <div class="iconBtn" @click="onContinueMapping()" v-if="isUse">
        <img src="@/assets/img/seeMap/continueMap.svg" />
        <p>继续建图</p>
      </div>
      <div class="iconBtn" @click="onEdit()">
        <img src="@/assets/img/seeMap/edit.svg" />
        <p>编辑地图</p>
      </div>
      <div class="over" @click="onDel()">
        <img src="@/assets/img/del.svg" />
        <p>删除地图</p>
      </div>
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/new";

export default {
  components: {
    ShowMap
  },
  data() {
    return {
      continuingMappingLoading: false
    };
  },
  computed: {
    isUse: function () {
      return this.$store.state.nowMap.id == this.$route.query.id
    }
  },
  created() {},
  methods: {
    onUse() {
      // 设置使用地图
      const msg = new ROSLIB.ServiceRequest({
        id: this.$route.query.id * 1
      });
      setCurrentMapId.callService(
        msg,
        result => {
          this.$store.state.nowMap.id = this.$route.query.id;
          this.$store.state.nowMap.name = this.$route.query.mapName;
          this.$message("设置成功");
          console.log("[ setCurrentMapId OK]-61", result);
        },
        result => {
          this.$message("设置失败");
          console.log("[ setCurrentMapId ERR]-61", result);
        }
      );
      // 状态控制
      this.$store.state.actionStatus = "localization";
      const modeMsg = new ROSLIB.ServiceRequest({
        action: "localization"
      });
      robotMode.callService(
        modeMsg,
        result => {
          console.log("[ robotMode OK]-61", result);
        },
        result => {
          console.log("[ robotMode ERR]-61", result);
        }
      );
      // 重定位
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
            x: 0.0,
            y: 0.0,
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
      var pose_msg = new ROSLIB.Message(point);
      PoseStamped.publish(pose_msg);
    },
    onContinueMapping() {
      if (this.continuingMappingLoading) return;
      this.continuingMappingLoading = true;
      const modeMsg = new ROSLIB.ServiceRequest({
        action: "continue_mapping"
      });
      robotMode.callService(
        modeMsg,
        result => {
          this.continuingMappingLoading = false;
          console.log("[ robotMode continue_mapping OK]", result);
          this.$router.push({
            name: "newMap",
            query: {
              mapName: this.$route.query.mapName,
              continueMapping: "true",
            }
          });
        },
        result => {
          this.continuingMappingLoading = false;
          console.log("[ robotMode continue_mapping ERR]", result);
          this.$message(result);
        }
      );
    },
    onEdit() {
      this.$router.push({
        name: "editMap",
        query: { id: this.$route.query.id }
      });
    },
    onDel() {
      this.$confirm(
        `<div>是否确认删除地图</div><div>（本操作无法恢复）</div>`,
        "删除地图",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      )
        .then(() => {
          const msg = new ROSLIB.ServiceRequest({
            id: this.$route.query.id * 1,
            data_type: "map"
          });
          deleteMap.callService(
            msg,
            result => {
              if (result.success) {
                this.$message("删除成功");
                this.$router.push({ name: "map" });
              } else {
                this.$message("删除失败");
              }
              console.log("[ deleteMap OK]-61", result);
            },
            result => {
              console.log("[ deleteMap ERR]-61", result);
            }
          );
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
  align-items: center;
  justify-content: space-evenly;
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
}

.iconBtn {
  display: flex;
  align-items: center;
  justify-content: space-evenly;
  width: 300px;
  height: 120px;
  border-radius: 10px;
  opacity: 1;
  background: linear-gradient(
    110deg,
    rgba(71, 84, 141, 0.64) 11%,
    rgba(53, 81, 119, 0.15) 88%,
    rgba(53, 92, 119, 0.14) 89%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.titleBox {
  display: flex;
  flex-direction: column;
  justify-content: space-between;
  height: 146px;

  .mapName {
    width: 329px;
    height: 80px;
    border-radius: 10px;
    opacity: 1;
    background: #2f3758;
    backdrop-filter: blur(10px);
    display: flex;
    align-items: center;
    justify-content: space-around;
  }
}

.over {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  opacity: 1;
  background: #d94040;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  align-items: center;
  justify-content: center;
}
.usemap {
  background: linear-gradient(
    110deg,
    rgba(55, 89, 238, 0.64) 11%,
    rgba(30, 157, 244, 0.37) 89%
  );
}
</style>
