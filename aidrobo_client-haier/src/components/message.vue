<template>
  <transition name="fade">
    <div class="msgBox" v-show="showMsg">
    <!-- TODO:两个类型的图标 -->
    <div class="content">
     <img src="" /><slot></slot>
    </div>
  </div>
  </transition>
</template>

<script>
import { mapState } from "vuex";

export default {
  computed: {
    ...mapState([
      "showMsg",
    ])
  },
  data() {
    return {
    };
  },
  watch:{
    showMsg:function(n){
      if(n){
        setTimeout(()=>{
          this.$store.state.showMsg=false
        },3000)
      }
    }
  },
  methods:{
  },
  mounted(){
    console.log('[  ]-21')
  }
};
</script>

<style lang="less" scoped>
.msgBox {
  position: absolute;
  z-index: 2000;
  padding: 8px 16px;
  left: 50%;
  transform: translateX(-50%);
  color: #fff;
  top: 180px;
  height: 160px;
  border-radius: 20px;
  background: #212a53;
  border: 5px solid #45538f;
  backdrop-filter: blur(10.88px);
  box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);
  .content{
    padding: 0px 60px;
    display: flex;
    line-height: 160px;
  }
}
.fade-enter-active, .fade-leave-active {
  transition: opacity .5s;
}
.fade-enter, .fade-leave-to {
  opacity: 0;
}
</style>
