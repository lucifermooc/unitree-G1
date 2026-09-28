<template>
  <div class="object-recognition-container">
    <div class="video-container" id="video"></div>
  </div>
</template>

<script>
export default {
  data() {
    return {};
  },
  mounted() {
    this.initVideo();
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
    }
  }
};
</script>

<style lang="less" scoped>
.object-recognition-container {
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
  width: 1840px;
  //background: transparent;
  //border-radius: 5px;
  background: #526cad;
  //overflow: hidden;
}
</style>
