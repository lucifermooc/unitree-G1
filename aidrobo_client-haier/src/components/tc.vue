<template>
  <transition name="fade">
    <div class="tcBox" v-show="showTc">
      <div class="bg" @click="$store.state.showTc=false"></div>
      <div class="contentBox">
        <div class="title">{{ title }}</div>
        <slot></slot>
        <div class="lBtn btn">确定</div>
        <div class="rBtn btn">取消</div>
      </div>
    </div>
  </transition>
</template>

<script>
import { mapState } from "vuex";

export default {
  props: [ "title"],
  computed: {
    ...mapState([
      "showTc",
    ])
  },
  data () {
    return {
    };
  }
};
</script>

<style lang="less" scoped>
.tcBox {
  position: absolute;
  z-index: 2000;
  width: 100%;
  height: calc(100% - 120px);
  margin-top: 120px;
  color: #fff;
  top: 0;
  left: 0;

  .bg {
    width: 100%;
    height: 100%;
    position: absolute;
    background: #00000059;
    z-index: -1;
  }
}

.fade-enter-active,
.fade-leave-active {
  transition: opacity .5s;
}

.fade-enter,
.fade-leave-to {
  opacity: 0;
}

.contentBox {
  position: absolute;
  width: 800px;
  height: 500px;
  left: 50%;
  margin-left: -400px;
  margin-top: 200px;
  border-radius: 20px;
  opacity: 1;
  background: linear-gradient(122deg, #47548c 0%, #324387 100%);
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);

  .title {
    font-size: 50PX;
    text-align: center;
    margin-top: 50PX;
  }

  .btn {
    position: absolute;
    width: 280px;
    height: 80px;
    border-radius: 60px;
    bottom: 70px;
    text-align: center;
    line-height: 80px;
    ;
  }

  .lBtn {
    left: 100px;
    background: #d94040;
  }

  .rBtn {
    right: 100px;
    background: #7F86B9;
  }
}
</style>
