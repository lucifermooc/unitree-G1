<template>
  <div class="following-container">
    <div class="video-container" id="video"></div>
    <div class="right">
      <div class="rText">
        <p class="title">{{ text.title }}</p>
        <p class="description">{{ text.description }}</p>
      </div>

<!--      <div>-->
<!--        <div class="message hide" ref="message">-->
<!--          <img src="@/assets/img/forbid.svg" />-->
<!--          <span>跟踪丢失</span>-->
<!--        </div>-->

<!--        <div v-if="isStart" class="action danger-status" @click="stopFollow">-->
<!--          停止跟随-->
<!--        </div>-->
<!--        <div-->
<!--          v-else-->
<!--          :class="['action', { readyBtn: personStatus }]"-->
<!--          @click="startFollow(personStatus)"-->
<!--        >-->
<!--          开始跟随-->
<!--        </div>-->
<!--      </div>-->
    </div>
  </div>
</template>

<script>
const TEXT = {
  idle: {
    // title: "选择跟随目标",
    title: "人形跟随",
    // description: "请选择跟随目标后点击下方开始跟随按钮"
    description: "自动开始跟随目标"
  },
  following: {
    title: "正在跟随...",
    description: "正在跟随标记的目标，行走的速度请勿过快"
  }
};
export default {
  data() {
    return {
      isStart: false, // 是否开启了跟随
      loading: false,
      personStatus: false, // 人物识别状态 false: 没有跟随； true: 跟随中
      lost: false
    };
  },
  computed: {
    text() {
      return this.isStart ? TEXT.following : TEXT.idle;
    }
  },
  mounted() {
    this.initVideo();
    // getFollowStatus.subscribe(res => {
    //   console.log("personStatus:", res.data);
    //   this.personStatus = res.data;
    //
    //   if (this.lost && this.personStatus) {
    //     // 取消 ‘跟踪丢失’ 的提示
    //     this.setLost(false);
    //   }
    //   if (this.isStart && !this.personStatus) {
    //     // ‘跟踪丢失’ 的提示
    //     this.setLost(true);
    //   }
    // });
  },
  beforeDestroy() {
    stopCamera.callService(
      null,
      res => {
        console.log("[ cam_stop ok]-61", res);
      },
      res => {
        console.log("[ cam_stop ERR]-61", res);
      }
    );
    // getFollowStatus.unsubscribe(res => {
    //   console.log("unsubscribe:", res);
    // });
    if (window.aidShowBridge && window.aidShowBridge.close) {
      window.aidShowBridge.close();
    }
  },
  methods: {
    initVideo() {
      const rgbWidth = this.$route.query.w;
      const rgbHeight = this.$route.query.h;
      const video = document.getElementById("video");
      const top = video.getBoundingClientRect().top;
      const left = video.getBoundingClientRect().left;
      const width = video.getBoundingClientRect().width;
      const height = video.getBoundingClientRect().height;
      if (window.aidShowBridge && window.aidShowBridge.setSurfaceLocation) {
        window.aidShowBridge.setSurfaceLocation(
          left,
          top,
          width,
          height,
          Number(rgbWidth),
          Number(rgbHeight),
          1920
        );
      }
    },
    startFollow(ready) {
      if (!ready || this.loading) return;
      this.loading = true;
      startFollow.callService(
        null,
        res => {
          console.log("[ follow_start ok]-61", res);
          this.isStart = true;
          this.loading = false;
        },
        res => {
          console.log("[ follow_start ERR]-61", res);
          this.loading = false;
        }
      );
    },
    stopFollow() {
      if (this.loading) return;
      this.loading = true;
      stopFollow.callService(
        null,
        res => {
          console.log("[ follow_stop ok]-61", res);
          this.isStart = false;
          this.loading = false;
        },
        res => {
          console.log("[ follow_stop ERR]-61", res);
          this.loading = false;
        }
      );
    },
    setLost(lost) {
      if (this.lost === lost) {
        // 状态没变
        return;
      }
      this.lost = lost;
      this.$refs.message.classList.remove("hide");

      if (!lost) {
        this.$refs.message.classList.add("hide");
      }
    }
  }
};
</script>

<style lang="less" scoped>
.following-container {
  box-sizing: border-box;
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
  padding-top: 30px;
  padding-left: 30px;
}

.video-container {
  position: relative;
  top: 0;
  height: calc(100% - 30px);
  width: 1380px;
  //background: transparent;
  //border-radius: 5px;
  background: #526cad;
  //overflow: hidden;
}

.right {
  width: 434px;
  height: calc(100% - 30px);
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
  line-height: 50px;
  margin-left: 30px;
  justify-content: space-between;
  padding: 20px 20px 70px 20px;
  box-sizing: border-box;

  .rText {
    display: flex;
    align-items: center;
    flex-direction: column;
    padding: 20px 34px;
    p {
      line-height: 45px;
      text-align: center;
    }
    .title {
      padding-bottom: 10px;
    }
    .description {
      margin-top: 35px;
    }
  }
}

.message {
  width: 300px;
  height: 80px;
  border-radius: 10px;
  background: #2e3449;
  backdrop-filter: blur(10px);
  margin-bottom: 60px;
  display: flex;
  justify-content: center;
  align-items: center;
  transition: all 0.3s;
  &.hide {
    transform: translateY(40px);
    opacity: 0;
  }
  span {
    margin-top: -5px;
    margin-left: 20px;
    font-size: 35px;
    font-weight: normal;
    line-height: 45px;
    letter-spacing: 1px;
  }
}
.action {
  width: 300px;
  height: 120px;
  border-radius: 10px;
  background: #4f5478;
  opacity: 1;
  display: flex;
  align-items: center;
  justify-content: center;
  box-shadow: 0px 2px 10px 0px rgba(1, 29, 90, 0.72);
  letter-spacing: 1px;
}
.readyBtn {
  background: linear-gradient(
    110deg,
    rgba(55, 89, 238, 0.64) 11%,
    rgba(30, 157, 244, 0.37) 89%
  ) !important;
}
.danger-status {
  background: #d94040 !important;
}
</style>
