<template>
  <div class="map">
    <div class="empty" v-if="!data.length">
      <img src="@/assets/img/empty.svg" />
      <p>无地图，请新建地图</p>
    </div>
    <div class="list" v-else>
      <div class="item" v-for="(e, i) in data" :key="i" @click="onSel(e)">
        <div>{{ e.name }}</div>
        <div v-show="e.id == $store.state.nowMap.id">当前地图</div>
        <div>{{ new Date(e.create_timestamp * 1000).toLocaleString() }}</div>
        <div class="line"></div>
        <img
          v-show="e.id != $store.state.nowMap.id"
          src="@/assets/img/del.svg"
          @click="onDel($event, e)"
        />
      </div>
    </div>
    <div class="addBtn" @click="addMap">
      <img src="@/assets/img/add.svg" />
      <span>新建地图</span>
    </div>
  </div>
</template>

<script>
import Tc from "@/components/tc";
import {mapMutations, mapState} from "vuex";

export default {
  components: {
    Tc
  },
  data() {
    return {
      showAdd: false,
      showDel: false,
      data: [],
      isSel: null
    };
  },
  mounted() {
    this.getMap();
    this.resetNavigationMapPoints();
  },
  methods: {
    ...mapMutations(["resetNavigationMapPoints"]),
    getMap() {
      getMapList.callService(
        null,
        result => {
          console.log('[getMapList] ok', result)
          try {
            this.data = JSON.parse(result.map_list);
          } catch (error) {
            console.log("[  finishMap ERR]-61", error);
          }
          console.log("[  finishMap OK]-61", result);
        },
        result => {
          console.log("[  finishMap ERR]-61", result);
        }
      );
    },
    addMap() {
      const addInp = document.querySelector("#addInp");
      addInp && (addInp.value = "");
      this.$confirm(
        `<div> 名称：
        <input id="addInp" placeholder=" 请输入内容" style="height: 70px;"></input>
        </div>`,
        "地图命名",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      ).then(e => {
        const addInp = document.querySelector("#addInp");
        const mapName = addInp.value;
        this.$router.push({ name: "newMap", query: { mapName } });
        // 建图模式
        const msg = new ROSLIB.ServiceRequest({
          action: "mapping"
        });
        robotMode.callService(
          msg,
          result => {
            console.log("[ robotMode OK]-61", result);
          },
          result => {
            console.log("[ robotMode ERR]-61", result);
          }
        );
        // 遥控模式
        this.$store.state.actionStatus = "remote";
        const msg2 = new ROSLIB.ServiceRequest({
          action: "remote_control"
        });
        robotMode.callService(
          msg2,
          result => {
            console.log("[ robotMode OK]-61", result);
          },
          result => {
            console.log("[ robotMode ERR]-61", result);
          }
        );
      });
    },
    onSel(e) {
      this.$router.push({
        name: "seeMap",
        query: { mapName: e.name, id: e.id }
      });
    },
    onDel(event, e) {
      event.stopPropagation();
      this.$confirm(
        `<div> 地图名：${e.name}</div><div>（本操作无法恢复）</div>`,
        "删除地图",
        {
          dangerouslyUseHTMLString: true,
          center: true
        }
      ).then(() => {
        const msg = new ROSLIB.ServiceRequest({
          id: e.id * 1,
          data_type: "map"
        });
        deleteMap.callService(
          msg,
          result => {
            if (result.success) {
              this.$message("删除成功");
              this.getMap();
            } else {
              this.$message("删除失败");
            }
          },
          result => {
            console.log("[ deleteMap ERR]-61", result);
          }
        );
      });
    }
  }
};
</script>

<style lang="less" scoped>
.map {
  color: #fff;
  width: 100%;
  height: calc(100% - 120px);
}

.empty {
  display: flex;
  height: 100%;
  width: 100%;
  align-items: center;
  justify-content: center;
  flex-direction: column;

  & p {
    margin-top: 40px;
  }
}

.list {
  width: 1740px;
  overflow: auto;
  margin: 100px auto 0 auto;
  height: 100%;
  padding: 20px 0 0 20px;

  .item {
    display: flex;
    align-items: center;
    font-size: 40px;
    position: relative;
    width: 1660px;
    height: 158px;
    border-radius: 20px;
    margin-bottom: 100px;
    background: linear-gradient(
      95deg,
      rgba(71, 84, 141, 0.64) 9%,
      rgba(53, 81, 119, 0.15) 86%,
      rgba(53, 92, 119, 0.14) 88%
    );
    backdrop-filter: blur(10.88px);
    box-shadow: 0px 2px 31px 0px rgba(1, 29, 90, 0.72);

    div:first-child {
      margin-left: 16px;
    }

    div:nth-child(2) {
      width: 165px;
      height: 58px;
      border-radius: 6px;
      background: #c061ff;
      text-align: center;
      line-height: 58px;
      margin-left: 2rem;
    }

    div:nth-child(3) {
      position: absolute;
      right: 240px;
    }

    &:active {
      border: 5px solid;
    }

    & .line {
      position: absolute;
      width: 3px;
      height: 48px;
      right: 200px;
      top: 60px;
      background: #ccc;
    }

    & img {
      position: absolute;
      right: 30px;
      top: 34px;
      width: 90px;
      height: 90px;
    }
  }
}

.addBtn {
  position: absolute;
  left: 1502px;
  top: 959px;
  width: 303px;
  height: 90px;
  border-radius: 100px;
  opacity: 1;
  background: linear-gradient(120deg, #c061ff 15%, #7364ff 69%);
  box-shadow: 0px 5px 10px 0px rgba(0, 0, 0, 0.4);
  z-index: 2;
  display: flex;
  align-items: center;
  justify-content: space-evenly;
}

.delContent {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  height: 300px;
}
</style>
