<template>
  <div class="newMapBox">
    <ShowMap />
    <div class="right point">
      <div v-if="false">
        <p>选择位置点</p>
        <p>请点击想前往的位置</p>
      </div>
      <div class="titleBox" v-if="$store.state.patrol_arr.length">
        <p>选择位置点:</p>
        <p class="mapName">({{ ($store.state.patrol_arr[0].x).toFixed(2) }},{{ ($store.state.patrol_arr[0].y).toFixed(2)
        }})</p>
      </div>
      <div class="titleBox">
        <p>机器人当前位置:</p>
        <p class="mapName">({{ ($store.state.robotPoint.x).toFixed(2) }},{{ ($store.state.robotPoint.y).toFixed(2) }})</p>
      </div>
      <div class="goPoint" @click="onBegin()" v-loading="!text">{{ text }}</div>
      <!-- <div class="goPoint">{{ text }}</div> -->
    </div>
  </div>
</template>

<script>
import ShowMap from "@/components/map/pointMap";

export default {
  components: {
    ShowMap
  },
  data () {
    return {
      text: '选择位置',
    }
  },
  mounted () {
    this.$store.state.hasSave = false;
    this.$store.state.patrol_arr = []
    // 状态机
    this.$store.state.actionStatus='point'
    const type = new ROSLIB.ServiceRequest({
      action: 'patrol'
    });
    robotMode.callService(type, (result) => {
      if (result.message!=='ok') {
        this.$message('状态切换失败');
        this.text = ''
      }
      console.log('[ robotMode OK]-61', result)
    }, (result) => {
      this.$message('状态切换失败');
      this.text = ''
      console.log('[ robotMode ERR]-61', result)
    });
  },
  methods: {
    onBegin () {
      if (this.text === '选择位置') {
        this.$store.state.tool = 'point'
        this.text = '前往位置'
      } else if (this.text === '前往位置') {
        if (!this.$store.state.patrol_arr.length) {
          return false;
        }
        const point = {
          poses: [{
            header: {
              stamp: {
                sec: 0,
                nanosec: 0
              },
              frame_id: "map"
            },
            pose: {
              position: {
                x: this.$store.state.patrol_arr[0].x,
                y: this.$store.state.patrol_arr[0].y,
                z: 0.0
              },
              orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0
              }
            }
          }]
        };
        const msg = new ROSLIB.Message(point);
        TalkerPoint.publish(msg);
        this.text = '关闭任务'
      } else {
        const type = new ROSLIB.ServiceRequest({
          cmd: 'cancel'
        });
        patrolState.callService(type, (res) => {
          console.log('[ patrol_control ok]-61', res)
        }, (res) => {
          console.log('[ patrol_control ERR]-61', res)
        });
        this.$router.push({ name: 'utility' })
      }
    },
  }

}
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
  background: linear-gradient(155deg, rgba(71, 84, 141, 0.64) 24%, rgba(71, 66, 124, 0.52) 98%);
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: space-evenly;
  line-height: 50px;
  margin-left: 30px;
  margin-top: 30px;

  &>p {
    text-align: center;
    text-align: center;
    line-height: 50px;
    padding: 0px 40px;
  }
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
    background: #2F3758;
    backdrop-filter: blur(10px);
    display: flex;
    align-items: center;
    justify-content: space-around;
  }
}

.goPoint {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  background: #4F5478;
  opacity: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
}

.readyBtn {
  background: linear-gradient(110deg, rgba(55, 89, 238, 0.64) 11%, rgba(30, 157, 244, 0.37) 89%) !important;
}
</style>
