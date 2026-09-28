<template>
  <div class="screen-container">
    <div class="video-container" id="video"></div>
    <div class="right-info">
      <div class="map">
        <MiniMap />
      </div>
      <div class="extra">
        <img src="@/assets/img/aid-logo1.png" alt="" />
        <img src="@/assets/img/aid-logo2.png" alt="" />
      </div>
    </div>
  </div>
</template>

<script>
import MiniMap from "../../components/map/miniMap.vue";

export default {
  components: { MiniMap },
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
.screen-container {
  --content-h: calc(100% - 30px);

  box-sizing: border-box;
  display: flex;
  color: #fff;
  width: 100%;
  height: 100%;
  position: relative;
  padding: 30px 30px 0 30px;
}

.video-container {
  position: relative;
  top: 0;
  height: var(--content-h);
  flex: 1;
  //background: transparent;
  //border-radius: 5px;
  background: #526cad;
  //overflow: hidden;
}
.right-info {
  margin-left: 20px;
  width: 434px;
  height: var(--content-h);
  display: flex;
  flex-direction: column;
}
.map,
.extra {
  flex: 1;
  background: linear-gradient(
    137deg,
    rgba(71, 84, 141, 0.64) 19%,
    rgba(71, 66, 124, 0.52) 94%
  );
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
}
.map {
  margin-bottom: 20px;
}
.extra {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;

  img {
    width: 70%;
    object-fit: contain;
    margin: 40px 0;
  }
}
</style>
