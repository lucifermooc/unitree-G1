<!-- jrf -->
<template>
  <div class="map" ref="map" @touchmove="map_move($event)">
    <div class="fa_map_box1">
      <div
        class="map_box1"
        ref="map_box1"
        @touchstart="rubberstart($event)"
        @touchmove="rubbermove($event)"
        @touchend="rubberend($event)"
        v-bind:style="{ transform: 'translate(' + left + 'px,' + top + 'px)' }"
      >
        <img id="img1" :src="mapData.src" @load="init" ref="img1" />
        <!-- <img id="img1" src="../../../static2/img/map2.png" @load="init" ref="img1" /> -->
        <svg
          v-if="planEnabled && planSvgPoints"
          class="plan_path"
          :viewBox="'0 0 ' + mapData.width + ' ' + mapData.height"
          :width="mapData.width * scale"
          :height="mapData.height * scale"
        >
          <polyline :points="planSvgPoints" />
        </svg>
        <svg
          v-if="
            showDirectionOverlay &&
              (patrolDirectionSegments.length || draftDirectionSegment)
          "
          class="direction_layer"
          :viewBox="'0 0 ' + mapData.width + ' ' + mapData.height"
          :width="mapData.width * scale"
          :height="mapData.height * scale"
        >
          <g
            v-for="(segment, index) in patrolDirectionSegments"
            :key="'direction-' + index"
          >
            <defs>
              <linearGradient
                :id="'direction-gradient-' + index"
                gradientUnits="userSpaceOnUse"
                :x1="segment.startX"
                :y1="segment.startY"
                :x2="segment.gradientX2"
                :y2="segment.gradientY2"
              >
                <stop offset="0%" stop-color=" #4f78ff" />
                <stop offset="100%" stop-color="#ffffff" />
              </linearGradient>
            </defs>
            <line
              class="direction_line"
              :x1="segment.startX"
              :y1="segment.startY"
              :x2="segment.endX"
              :y2="segment.endY"
              :stroke="'url(#direction-gradient-' + index + ')'"
            />
            <polygon
              class="direction_arrow"
              :points="segment.arrowPoints"
              :fill="'url(#direction-gradient-' + index + ')'"
            />
            <circle
              class="direction_badge"
              :cx="segment.badgeX"
              :cy="segment.badgeY"
              :r="segment.badgeRadius"
            />
            <text
              class="direction_label"
              :x="segment.badgeX"
              :y="segment.badgeY"
              :style="{
                fontSize: getDirectionLabelFontSize(segment, index + 1) + 'px'
              }"
            >
              {{ index + 1 }}
            </text>
          </g>
          <g v-if="draftDirectionSegment">
            <defs>
              <linearGradient
                id="direction-gradient-draft"
                gradientUnits="userSpaceOnUse"
                :x1="draftDirectionSegment.startX"
                :y1="draftDirectionSegment.startY"
                :x2="draftDirectionSegment.gradientX2"
                :y2="draftDirectionSegment.gradientY2"
              >
                <stop offset="0%" stop-color="#4f78ff" />
                <stop offset="100%" stop-color="#42f4aa" />
              </linearGradient>
            </defs>
            <line
              class="direction_line direction_preview"
              :x1="draftDirectionSegment.startX"
              :y1="draftDirectionSegment.startY"
              :x2="draftDirectionSegment.endX"
              :y2="draftDirectionSegment.endY"
              stroke="url(#direction-gradient-draft)"
            />
            <polygon
              class="direction_arrow direction_preview"
              :points="draftDirectionSegment.arrowPoints"
              fill="url(#direction-gradient-draft)"
            />
          </g>
        </svg>
        <div
          class="robot"
          v-bind:style="{
            transform:
              'translate(' +
              (robotXY.x * scale - 12) +
              'px,' +
              (robotXY.y * scale - 12) +
              'px) rotate(' +
              (90 - robotYaw) +
              'deg)'
          }"
        ></div>
        <div
          v-if="navigationPoint"
          v-for="(item, index) in pointsInMapImage"
          :key="index"
          class="map_point"
          v-bind:style="{
            transform:
              'translate(' +
              (item.x * scale - 25) +
              'px,' +
              (item.y * scale + -70) +
              'px)'
          }"
        >
          <img src="../../../static2/img/point.png" width="50px" />
          <span class="pointNum">{{ index + 1 }}</span>
        </div>
        <div class="map_box2">
          <canvas id="operate" ref="operate"></canvas>
        </div>
      </div>
    </div>
    <div class="img2">
      <div
        class="show_img"
        ref="show_img"
        @touchstart.stop.prevent="startMiniMapDrag($event)"
        @touchmove.stop.prevent="moveMiniMapDrag($event)"
        @touchend.stop.prevent="endMiniMapDrag"
        @touchcancel.stop.prevent="endMiniMapDrag"
        v-bind:style="{
          'margin-top': img2_top + 'px',
          'margin-left': img2_left + 'px'
        }"
      ></div>
      <img id="img2" :src="mapData.src" ref="img2" />
    </div>
    <div class="zoom">
      <img src="@/assets/img/seeMap/fda.png" @click="zoom('f')" />
      <img src="@/assets/img/seeMap/sxiao.png" @click="zoom('s')" />
    </div>
    <div class="active" v-if="showActiveToggle" :title="tool">
      <img
        src="@/assets/img/seeMap/active.png"
        @click="changeTool('')"
        v-if="tool == 'patrol' || tool == 'point' || tool == 'relocation'"
      />
      <img
        src="@/assets/img/seeMap/disActive.png"
        @click="changeTool(defaultToolType)"
        v-else
      />
    </div>
  </div>
</template>

<script type="text/ecmascript-6">
import { mapState, mapMutations } from "vuex";
import { applyTransformToPoint, changeStr, composeTransforms, createQuaternionFromYaw, imgToMap, invertTransform, mapToImg, normalizeFrameId, normalizePatrolPoints, quaternionToYawDeg, quaternionToYawRad, resolvePatrolPointYaw, resolveTransform, rosTimeToMillis, updateTransformGraph } from "@/assets/common"
import { clampMapOffset } from "@/utils/map";

export default {
  props: ["initData", 'navigationPoint', 'showPlan', 'showScan', 'relocationMode', 'navigationTargetMode'],
  data() {
    return {
      mapData: {
        src: "",
        width: 1930,
        height: 3909,
        resolution: 0,
        positionX: 0,
        positionY: 0,
      },
      robotXY: { x: 0, y: 0 },
      scale: 1,
      left: 0,
      top: 0,
      img2_scale: 1,
      img2_top: 0,
      img2_left: 0,
      imgsrc: null,
      d_width: null,
      d_height: null,
      operate_txc: null,
      interval: null, //全局控制定时器
      touch_data: null, //触摸点
      screen_w: 1380,
      yEnd: 0,
      xEnd: 0,
      directionDraft: null,
      directionArrowLength: 1,
      patrolDirectionMinDistance: 24,
      planMapPoints: [],
      planListener: null,
      scanMapPoints: [],
      latestScanMessage: null,
      scanRenderFrameId: null,
      scanListener: null,
      robotTfListener: null,
      robotTransform: null,
      scanFrameId: '',
      scanTransform: null,
      relocationPreviewPose: null,
      tfMessageListener: null,
      tfStaticListener: null,
      tfGraph: {},
      miniMapDragging: false,
      miniMapDragOffsetX: 0,
      miniMapDragOffsetY: 0
    };
  },
  computed: {
    ...mapState([
      "robotPoint",
      "robotYaw",
      "mcode",
      "rubber_data1",
      "rubber_data2",
      "head_h",
      "rubber_size",
      "tool",
      "mapSrc",
      "init_img_data",
      "patrol_arr_area",
      "patrol_arr",
      "m_width",
      "m_height",
      "m_resolution",
      "_map_name",
      "zero",
      "stop_point",
      "charge_po",
      "map_img_w",
      "navigationImgPoints",
    ]),
    pointsInMapImage () {
      return this.$props.navigationPoint ? this.navigationImgPoints : this.patrol_arr_area
    },
    planEnabled () {
      return !!(this.navigationPoint || this.showPlan)
    },
    scanEnabled () {
      return !!this.showScan
    },
    planImgPoints () {
      if (!this.mapData.resolution) {
        return []
      }
      return this.planMapPoints.map(point => ({
        x: mapToImg({ mapData: this.mapData, x: point.x }),
        y: mapToImg({ mapData: this.mapData, y: point.y })
      }))
    },
    planSvgPoints () {
      return this.planImgPoints
        .filter(point => Number.isFinite(point.x) && Number.isFinite(point.y))
        .map(point => `${point.x},${point.y}`)
        .join(' ')
    },
    defaultToolType() {
      if (this.relocationMode) {
        return 'relocation'
      }
      if (this.navigationTargetMode) {
        return 'navigation-target'
      }
      return window.location.hash.includes('patrol') ? 'patrol' : 'point'
    },
    showActiveToggle() {
      return !this.navigationPoint && !this.navigationTargetMode
    },
    showDirectionOverlay() {
      return !this.navigationPoint
    },
    patrolDirectionSegments () {
      if (this.navigationPoint) {
        return []
      }

      return this.patrol_arr_area
        .map(point => this.createDirectionSegmentFromPoint(point))
        .filter(Boolean)
    },
    draftDirectionSegment () {
      if (!this.directionDraft) {
        return null
      }

      return this.buildDirectionSegment(
        { x: this.directionDraft.startX, y: this.directionDraft.startY },
        { x: this.directionDraft.endX, y: this.directionDraft.endY }
      )
    }
  },
  watch: {
    robotPoint: function (n) {
      this.robotXY = { x: mapToImg({ mapData: this.mapData, x: n.x }), y: mapToImg({ mapData: this.mapData, y: n.y }) }
    },
    initData: function (n) {
      if (n) {
        this.syncPatrolImagePoints()
      }
    },
    'mapData.resolution': function (n) {
      if (n && this.initData) {
        this.syncPatrolImagePoints()
      }
    },
    tool: function (n) {
      if (n !== 'patrol' && n !== 'relocation' && n !== 'navigation-target') {
        this.clearDirectionDraft()
      }
    }
  },
  mounted() {
    this.$store.state.map_width = this.$refs.map.offsetWidth;
    this.subscribeTfMessages()
    this.subscribePlan()
    this.subscribeScan()
    this.getMap()
  },
  beforeDestroy() {
    if (this.scanRenderFrameId) {
      cancelAnimationFrame(this.scanRenderFrameId)
      this.scanRenderFrameId = null
    }
    this.unsubscribeTfMessages()
    this.unsubscribePlan()
    this.unsubscribeScan()
  },
  methods: {
    subscribeTfMessages() {
      if (this.tfMessageListener || this.tfStaticListener) {
        return
      }
      this.tfMessageListener = message => {
        updateTransformGraph(this.tfGraph, message.transforms || [], { isStatic: false })
        this.refreshResolvedTransforms()
      }
      this.tfStaticListener = message => {
        updateTransformGraph(this.tfGraph, message.transforms || [], { isStatic: true })
        this.refreshResolvedTransforms()
      }
      RobotTF.subscribe(this.tfMessageListener)
      RobotTFStatic.subscribe(this.tfStaticListener)
    },
    unsubscribeTfMessages() {
      if (this.tfMessageListener) {
        RobotTF.unsubscribe(this.tfMessageListener)
      }
      if (this.tfStaticListener) {
        RobotTFStatic.unsubscribe(this.tfStaticListener)
      }
      this.tfMessageListener = null
      this.tfStaticListener = null
      this.robotTfListener = null
      this.robotTransform = null
      this.scanTransform = null
      this.tfGraph = {}
    },
    refreshResolvedTransforms() {
      const baseLinkTransform = resolveTransform(this.tfGraph, 'map', 'base_link')
      if (baseLinkTransform) {
        this.robotTransform = baseLinkTransform
        this.$store.state.robotPoint = {
          x: Number(baseLinkTransform.translation.x || 0),
          y: Number(baseLinkTransform.translation.y || 0)
        }
        this.$store.state.robotYaw = quaternionToYawDeg(baseLinkTransform.rotation)
      }
      if (this.scanFrameId) {
        this.scanTransform = resolveTransform(this.tfGraph, 'map', this.scanFrameId)
        if (this.relocationPreviewPose) {
          this.updateScanMapPoints()
        }
      }
    },
    subscribePlan() {
      if (!this.planEnabled || this.planListener) {
        return
      }
      this.planListener = message => {
        this.planMapPoints = (message.poses || []).map(item => item.pose.position)
      }
      NavigationPlan.subscribe(this.planListener)
    },
    unsubscribePlan() {
      if (!this.planListener) {
        return
      }
      NavigationPlan.unsubscribe(this.planListener)
      this.planListener = null
      this.planMapPoints = []
    },
    subscribeScan() {
      if (!this.scanEnabled || this.scanListener) {
        return
      }
      this.scanListener = message => {
        this.latestScanMessage = message
        this.ensureScanFrameSubscription(
          message && message.header ? message.header.frame_id : '',
          this.getMessageTimestampMs(message)
        )
        this.updateScanMapPoints()
      }
      RobotScan.subscribe(this.scanListener)
    },
    unsubscribeScan() {
      if (!this.scanListener) {
        return
      }
      RobotScan.unsubscribe(this.scanListener)
      this.scanListener = null
      this.latestScanMessage = null
      this.scanMapPoints = []
      this.scheduleScanRender()
    },
    ensureScanFrameSubscription(frameId, timestampMs = null) {
      const nextFrameId = normalizeFrameId(frameId)
      if (!nextFrameId) {
        return
      }
      this.scanFrameId = nextFrameId
      this.scanTransform = resolveTransform(this.tfGraph, 'map', nextFrameId, { timestampMs })
    },
    convertScanToMapPoints(message) {
      const ranges = Array.isArray(message && message.ranges) ? message.ranges : []
      if (!ranges.length) {
        return []
      }

      const scanFrameId = normalizeFrameId(message && message.header ? message.header.frame_id : '')
      const timestampMs = this.getMessageTimestampMs(message)
      const isMapFrame = scanFrameId === 'map'
      const activeScanTransform = isMapFrame ? null : this.getActiveScanTransform(timestampMs)
      if (!isMapFrame && !activeScanTransform) {
        return []
      }

      const angleMin = Number(message.angle_min || 0)
      const angleIncrement = Number(message.angle_increment || 0)
      const rangeMin = Number(message.range_min || 0)
      const rangeMax = Number(message.range_max || Infinity)
      const points = []

      for (let index = 0; index < ranges.length; index += 1) {
        const range = Number(ranges[index])
        if (!Number.isFinite(range) || range < rangeMin || range > rangeMax) {
          continue
        }
        const angle = angleMin + angleIncrement * index
        const localPoint = {
          x: range * Math.cos(angle),
          y: range * Math.sin(angle),
          z: 0
        }
        points.push(isMapFrame ? localPoint : applyTransformToPoint(localPoint, activeScanTransform))
      }

      return points
    },
    updateScanMapPoints() {
      this.scanMapPoints = this.convertScanToMapPoints(this.latestScanMessage)
      this.scheduleScanRender()
    },
    getMessageTimestampMs(message) {
      return rosTimeToMillis((((message || {}).header || {}).stamp) || {})
    },
    getActiveScanTransform(timestampMs = null) {
      const scanTransform = resolveTransform(this.tfGraph, 'map', this.scanFrameId, { timestampMs }) || this.scanTransform
      if (!this.relocationMode || !this.relocationPreviewPose || !scanTransform) {
        return scanTransform
      }

      const robotTransform = resolveTransform(this.tfGraph, 'map', 'base_link', { timestampMs }) || this.robotTransform
      if (!robotTransform) {
        return scanTransform
      }

      const baseLinkToScanTransform = composeTransforms(
        invertTransform(robotTransform),
        scanTransform
      )

      return composeTransforms(
        this.createTransformFromPose(this.relocationPreviewPose),
        baseLinkToScanTransform
      )
    },
    createTransformFromPose(pose = {}) {
      return {
        translation: {
          x: Number(pose.x || 0),
          y: Number(pose.y || 0),
          z: Number(pose.z || 0)
        },
        rotation: pose.orientation || createQuaternionFromYaw(Number(pose.yaw || 0))
      }
    },
    scheduleScanRender() {
      if (this.scanRenderFrameId) {
        cancelAnimationFrame(this.scanRenderFrameId)
      }

      this.scanRenderFrameId = requestAnimationFrame(() => {
        this.scanRenderFrameId = null
        this.drawScanLayer()
      })
    },
    drawScanLayer() {
      if (!this.operate_txc || !this.$refs.operate) {
        return
      }

      const canvas = this.$refs.operate
      const ctx = this.operate_txc
      ctx.clearRect(0, 0, canvas.width, canvas.height)

      if (!this.scanEnabled || !this.mapData.resolution || !this.scanMapPoints.length) {
        return
      }

      const maxRenderedPoints = this.scale > 2 ? 1400 : 900
      const step = Math.max(1, Math.ceil(this.scanMapPoints.length / maxRenderedPoints))
      const pointSize = this.scale > 1 ? 1 : 1.2

      ctx.save()
      ctx.fillStyle = 'rgba(255, 196, 61, 0.35)'

      for (let index = 0; index < this.scanMapPoints.length; index += step) {
        const point = this.scanMapPoints[index]
        const x = mapToImg({ mapData: this.mapData, x: point.x })
        const y = mapToImg({ mapData: this.mapData, y: point.y })

        if (!Number.isFinite(x) || !Number.isFinite(y)) {
          continue
        }

        if (x < 0 || y < 0 || x > this.mapData.width || y > this.mapData.height) {
          continue
        }

        ctx.fillRect(x - pointSize / 2, y - pointSize / 2, pointSize, pointSize)
      }

      ctx.restore()
    },
    changeTool(type) {
      this.clearDirectionDraft()
      // if (this.relocationMode) {
      //   this.$store.state.tool = 'relocation'
      //   return
      // }
      if (this.navigationTargetMode) {
        this.$store.state.tool = 'navigation-target'
        return
      }
      if (!type) {
        this.$store.state.tool = ''
      } else {
        this.$store.state.tool = type || this.defaultToolType
      }

    },
    getMap() {
      const msg = new ROSLIB.ServiceRequest({
        id: this.$store.state.nowMap.id * 1
      });
      console.log('getMapImage', msg)
      getMapImage.callService(msg, (res) => {
        console.log('[ getMapImage OK]-61', res)
        if (res.success) {
          this.mapData = changeStr(res.map)
          this.initData && this.syncPatrolImagePoints()
          this.$props.navigationPoint && this.getPoints();
        }
      }, (result) => {
        console.log('[ getMapImage ERR]-61', result)
      });

    },
    zoom(type){
      let img_w = this.$refs.img1.width;
      let img_h = this.$refs.img1.height;
      let left = this.left;
      let top = this.top;
      let s_h = top / img_h;
      let s_w = left / img_w;

      if(type==='f'){
        if (this.scale < 20) {
          this.$refs.map_box1.style.transition = "transform 1s";
          this.scale += 0.1;
          this.img2_scale -= 0.025;
          img_w = this.d_width * this.scale;
          this.$refs.img1.width = img_w;
          img_h = this.$refs.img1.height;
          this.$refs.operate.style.transform = "scale(" + this.scale + ")";
          this.$refs.show_img.style.width =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) / img_w + "px";
          this.$refs.show_img.style.height =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) / img_h +
            "px";

          this.top = s_h * img_h;
          this.left = s_w * img_w;
        }
      }else{
        if (img_w > this.screen_w) {
          this.$refs.map_box1.style.transition = "transform 1s";
          this.scale > 1 ? (this.scale -= 0.1) : (this.scale = 1);
          this.img2_scale += 0.025;
          img_w < this.screen_w
            ? (img_w = this.screen_w)
            : (img_w = this.d_width * this.scale);
          this.$refs.img1.width = img_w;
          img_h = this.$refs.img1.height;
          this.$refs.operate.style.transform = "scale(" + this.scale + ")";
          this.$refs.show_img.style.width =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) / img_w + "px";
          this.$refs.show_img.style.height =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) / img_h +
            "px";
          if (this.scale == 1) {
            this.img2_left = this.img2_top = this.left = this.top = 0;
          }
        }
      }
      this.scheduleScanRender()
    },
    touchStart(e, n) {
      this.$store.state.init_img_data = false;
      this.$refs.map_box1.style.transition = "none";
      e.currentTarget.classList.add("cli_box");
    },
    touchend(e) {
      clearInterval(this.interval);
      this.interval = null;
      e.currentTarget.classList.remove("cli_box");
    },
    init() {
      this.top = this.left = this.img2_top = this.img2_left = 0;
      this.scale = 1;
      let img1 = document.getElementById("img1");

      const sc = 1380 / this.mapData.width
      sc > 1 && (this.scale = sc)
      img1.width = this.scale * this.mapData.width


      let operate = document.getElementById("operate");
      this.$store.state.x_can = operate;
      this.operate_txc = operate.getContext("2d");
      // 获取的图片进行等比适配
      this.$store.state.map_img_w = this.d_width = operate.width =this.mapData.width;
      this.d_height = operate.height =  this.mapData.height;
      this.$refs.operate.style.transform = "scale(" + this.scale + ")";
      this.$refs.img2.width = img1.width / 11;
      this.$refs.show_img.style.width = this.$refs.map.offsetWidth / 11 + "px";
      this.$refs.show_img.style.height =
        this.$refs.map.offsetHeight / 11 + "px";
      this.scheduleScanRender()
    },
    rubberstart(e) {
      let set_time = 0;
      this.$refs.map_box1.style.transition = "none";
      let ctx = this.operate_txc;
      e != undefined
        ? (this.touch_data = e.touches[0])
        : (this.touch_data = this.touch_data);
      if (this.tool == "") {
        // 地图滑动
        this.xEnd = Math.round(this.touch_data.pageX / this.scale);
        this.yEnd = Math.round(
          (this.touch_data.pageY - this.head_h) / this.scale
        );
      } else if (this.tool == "patrol" || this.tool == "relocation" || this.tool == "navigation-target") {
        const touch = this.getTouchFromEvent(e)
        const startPoint = this.getImagePointFromTouch(touch)
        if (!startPoint) {
          return
        }
        this.directionDraft = {
          startX: startPoint.x,
          startY: startPoint.y,
          endX: startPoint.x,
          endY: startPoint.y
        }
        this.updateRelocationPreviewPose(this.directionDraft)
      }
    },
    yy2(y) {
      return this.mapData.height - (y - this.mapData.positionY) / this.mapData.resolution;
    },
    xx2(x) {
      return (x - this.mapData.positionX) / this.mapData.resolution;
    },
    getMiniMapViewportMetrics() {
      if (!this.$refs.img2 || !this.$refs.show_img || !this.$refs.img1 || !this.$refs.map) {
        return null
      }

      const miniMapWidth = this.$refs.img2.width || 0
      const miniMapHeight = this.$refs.img2.height || 0
      const viewportWidth = this.$refs.show_img.offsetWidth || 0
      const viewportHeight = this.$refs.show_img.offsetHeight || 0
      const mainOverflowX = Math.max(this.$refs.img1.width - this.$refs.map.offsetWidth, 0)
      const mainOverflowY = Math.max(this.$refs.img1.height - this.$refs.map.offsetHeight, 0)

      return {
        miniMapRect: this.$refs.img2.getBoundingClientRect(),
        maxMiniMapLeft: Math.max(miniMapWidth - viewportWidth, 0),
        maxMiniMapTop: Math.max(miniMapHeight - viewportHeight, 0),
        mainOverflowX,
        mainOverflowY
      }
    },
    syncViewportFromMiniMap(nextLeft, nextTop) {
      const metrics = this.getMiniMapViewportMetrics()
      if (!metrics) {
        return
      }

      const miniMapLeft = Math.max(0, Math.min(metrics.maxMiniMapLeft, nextLeft))
      const miniMapTop = Math.max(0, Math.min(metrics.maxMiniMapTop, nextTop))

      this.img2_left = miniMapLeft
      this.img2_top = miniMapTop
      this.left = metrics.maxMiniMapLeft
        ? -(miniMapLeft / metrics.maxMiniMapLeft) * metrics.mainOverflowX
        : 0
      this.top = metrics.maxMiniMapTop
        ? -(miniMapTop / metrics.maxMiniMapTop) * metrics.mainOverflowY
        : 0
    },
    startMiniMapDrag(event) {
      const touch = this.getTouchFromEvent(event)
      const metrics = this.getMiniMapViewportMetrics()
      if (!touch || !metrics || !this.$refs.show_img) {
        return
      }

      const viewportRect = this.$refs.show_img.getBoundingClientRect()
      this.miniMapDragging = true
      this.miniMapDragOffsetX = touch.clientX - viewportRect.left
      this.miniMapDragOffsetY = touch.clientY - viewportRect.top
    },
    moveMiniMapDrag(event) {
      if (!this.miniMapDragging) {
        return
      }

      const touch = this.getTouchFromEvent(event)
      const metrics = this.getMiniMapViewportMetrics()
      if (!touch || !metrics) {
        return
      }

      const nextLeft = touch.clientX - metrics.miniMapRect.left - this.miniMapDragOffsetX
      const nextTop = touch.clientY - metrics.miniMapRect.top - this.miniMapDragOffsetY
      this.syncViewportFromMiniMap(nextLeft, nextTop)
    },
    endMiniMapDrag() {
      this.miniMapDragging = false
    },
    rubbermove(e) {
      let ctx = this.operate_txc;
      let r_x = Math.round((this.touch_data.pageX - this.left) / this.scale);
      let r_y = Math.round(
        (this.touch_data.pageY - this.head_h - this.top) / this.scale
      );
      e.preventDefault();
      e != undefined
        ? (this.touch_data = e.touches[0])
        : (this.touch_data = this.touch_data);
      if ((this.tool == 'patrol' || this.tool == 'relocation' || this.tool == 'navigation-target') && this.directionDraft) {
        const touch = this.getTouchFromEvent(e)
        const currentPoint = this.getImagePointFromTouch(touch)
        if (!currentPoint) {
          return
        }
        this.directionDraft = {
          ...this.directionDraft,
          endX: currentPoint.x,
          endY: currentPoint.y
        }
        this.updateRelocationPreviewPose(this.directionDraft)
      }
    },
    rubberend(e) {
      if (this.mcode > 1) {
        return
      }

      if (this.tool == "point") {
        const touch = this.getTouchFromEvent(e)
        this.selectPointFromTouch(touch)
      } else if (this.tool == "patrol") {
        const touch = this.getTouchFromEvent(e)
        this.selectPatrolPoint(touch)
      } else if (this.tool == "relocation") {
        const touch = this.getTouchFromEvent(e)
        this.selectRelocationPoint(touch)
      } else if (this.tool == "navigation-target") {
        const touch = this.getTouchFromEvent(e)
        this.selectNavigationTargetPoint(touch)
      }
    },
    map_move(e) {
      try {
        if (this.tool == "") {
          if (e) {
            e.preventDefault();
            this.touch_data = e.touches[0];
          }
          let cT =
            Math.round((this.touch_data.pageY - this.head_h) / this.scale) -
            this.yEnd;
          let cL = Math.round(this.touch_data.pageX / this.scale) - this.xEnd;
          this.yEnd = Math.round(
            (this.touch_data.pageY - this.head_h) / this.scale
          );
          let show_img_w =
            (this.$refs.img2.width * this.$refs.map.offsetWidth) /
            this.$refs.img1.width;
          let show_img_h =
            (this.$refs.img2.height * this.$refs.map.offsetHeight) /
            this.$refs.img1.height;
          this.xEnd = Math.round(this.touch_data.pageX / this.scale);
          this.top = this.top + cT*10;
          this.left = this.left + cL*10;
          this.img2_top -= (show_img_h / this.$refs.map.offsetHeight) * cT; //小地图边界判断(通过比例值获取)
          this.img2_left -= (show_img_w / this.$refs.map.offsetWidth) * cL;
          this.top = clampMapOffset(
            this.top,
            this.$refs.map.offsetHeight - this.$refs.img1.height
          );
          this.img2_top = clampMapOffset(
            this.img2_top,
            this.$refs.img2.height - show_img_h
          );
          this.left = clampMapOffset(
            this.left,
            this.$refs.map.offsetWidth - this.$refs.img1.width
          );
          this.img2_left = clampMapOffset(
            this.img2_left,
            this.$refs.img2.width - show_img_w
          );
        }
      } catch (e) {
      }
    },
    syncPatrolImagePoints() {
      if (!this.mapData.resolution) {
        return
      }

      if (!this.patrol_arr.length) {
        this.$store.state.patrol_arr_area = []
        return
      }

      this.$store.state.patrol_arr_area = normalizePatrolPoints(this.patrol_arr).map(point => ({
        x: mapToImg({ mapData: this.mapData, x: point.x }),
        y: mapToImg({ mapData: this.mapData, y: point.y }),
        yaw: point.yaw,
        orientation: point.orientation
      }))
    },
    clearDirectionDraft() {
      this.directionDraft = null
      if (this.relocationPreviewPose) {
        this.relocationPreviewPose = null
        this.updateScanMapPoints()
      }
    },
    updateRelocationPreviewPose(draft) {
      if (!this.relocationMode) {
        return
      }

      this.relocationPreviewPose = this.buildRelocationPreviewPose(draft)
      this.updateScanMapPoints()
    },
    buildRelocationPreviewPose(draft) {
      if (!draft || !this.mapData.resolution) {
        return null
      }

      const startPoint = { x: draft.startX, y: draft.startY }
      const endPoint = { x: draft.endX, y: draft.endY }
      const mapStartPoint = {
        x: imgToMap({ mapData: this.mapData, x: startPoint.x }),
        y: imgToMap({ mapData: this.mapData, y: startPoint.y })
      }

      let yaw = this.robotTransform ? quaternionToYawRad(this.robotTransform.rotation) : 0
      if (this.getGestureDistance(startPoint, endPoint) >= 2) {
        const mapEndPoint = {
          x: imgToMap({ mapData: this.mapData, x: endPoint.x }),
          y: imgToMap({ mapData: this.mapData, y: endPoint.y })
        }
        yaw = Math.atan2(
          mapEndPoint.y - mapStartPoint.y,
          mapEndPoint.x - mapStartPoint.x
        )
      }

      return {
        x: mapStartPoint.x,
        y: mapStartPoint.y,
        z: Number((((this.robotTransform || {}).translation || {}).z) || 0),
        yaw,
        orientation: createQuaternionFromYaw(yaw)
      }
    },
    getTouchFromEvent(event) {
      if (!event) {
        return null
      }

      if (event.changedTouches && event.changedTouches.length) {
        return event.changedTouches[0]
      }

      if (event.touches && event.touches.length) {
        return event.touches[0]
      }

      return null
    },
    clampImagePoint(point) {
      return {
        x: Math.max(0, Math.min(this.mapData.width, Math.round(point.x))),
        y: Math.max(0, Math.min(this.mapData.height, Math.round(point.y)))
      }
    },
    getImagePointFromTouch(touch) {
      if (!touch || !this.$refs.map_box1) {
        return null
      }

      const rect = this.$refs.map_box1.getBoundingClientRect()
      return this.clampImagePoint({
        x: (touch.clientX - rect.left) / this.scale,
        y: (touch.clientY - rect.top) / this.scale
      })
    },
    getGestureDistance(startPoint, endPoint) {
      return Math.hypot(endPoint.x - startPoint.x, endPoint.y - startPoint.y)
    },
    buildDirectionSegment(startPoint, endPoint) {
      if (!startPoint || !endPoint) {
        return null
      }

      const angle = Math.atan2(endPoint.y - startPoint.y, endPoint.x - startPoint.x)
      const headLength = 18
      const headAngle = Math.PI / 7
      const forwardX = Math.cos(angle)
      const forwardY = Math.sin(angle)
      const perpendicularX = -forwardY
      const perpendicularY = forwardX
      const baseHalfWidth = headLength * Math.tan(headAngle)
      const tipPoint = {
        x: endPoint.x + forwardX * ((headLength * 2) / 3),
        y: endPoint.y + forwardY * ((headLength * 2) / 3)
      }
      const baseCenterPoint = {
        x: endPoint.x - forwardX * (headLength / 3),
        y: endPoint.y - forwardY * (headLength / 3)
      }
      const leftBasePoint = {
        x: baseCenterPoint.x + perpendicularX * baseHalfWidth,
        y: baseCenterPoint.y + perpendicularY * baseHalfWidth
      }
      const rightBasePoint = {
        x: baseCenterPoint.x - perpendicularX * baseHalfWidth,
        y: baseCenterPoint.y - perpendicularY * baseHalfWidth
      }
      const sideA = Math.hypot(rightBasePoint.x - leftBasePoint.x, rightBasePoint.y - leftBasePoint.y)
      const sideB = Math.hypot(rightBasePoint.x - tipPoint.x, rightBasePoint.y - tipPoint.y)
      const sideC = Math.hypot(leftBasePoint.x - tipPoint.x, leftBasePoint.y - tipPoint.y)
      const perimeter = sideA + sideB + sideC
      const doubleArea = Math.abs(
        tipPoint.x * (leftBasePoint.y - rightBasePoint.y) +
        leftBasePoint.x * (rightBasePoint.y - tipPoint.y) +
        rightBasePoint.x * (tipPoint.y - leftBasePoint.y)
      )
      const badgeRadius = perimeter ? (doubleArea / 2) / (perimeter / 2) : 0
      const badgeX = perimeter
        ? (sideA * tipPoint.x + sideB * leftBasePoint.x + sideC * rightBasePoint.x) / perimeter
        : endPoint.x
      const badgeY = perimeter
        ? (sideA * tipPoint.y + sideB * leftBasePoint.y + sideC * rightBasePoint.y) / perimeter
        : endPoint.y

      return {
        startX: startPoint.x,
        startY: startPoint.y,
        endX: endPoint.x,
        endY: endPoint.y,
        gradientX2: tipPoint.x,
        gradientY2: tipPoint.y,
        badgeX,
        badgeY,
        badgeRadius,
        arrowPoints: [
          `${tipPoint.x},${tipPoint.y}`,
          `${leftBasePoint.x},${leftBasePoint.y}`,
          `${rightBasePoint.x},${rightBasePoint.y}`
        ].join(' ')
      }
    },
    createDirectionSegmentFromPoint(point) {
      if (!point || !Number.isFinite(point.x) || !Number.isFinite(point.y)) {
        return null
      }

      const yaw = resolvePatrolPointYaw(point)
      const endPoint = {
        x: point.x + this.directionArrowLength * Math.cos(yaw),
        y: point.y - this.directionArrowLength * Math.sin(yaw)
      }

      return this.buildDirectionSegment(point, endPoint)
    },
    getDirectionLabelFontSize(segment, label) {
      const text = String(label || '')
      const textLength = Math.max(text.length, 1)
      const maxDiameter = (segment.badgeRadius || 0) * 2
      const estimatedWidthFactor = 0.62
      const maxByWidth = maxDiameter / (textLength * estimatedWidthFactor)
      const maxByHeight = (segment.badgeRadius || 0) * 1.15

      return Math.max(7, Math.min(maxByWidth, maxByHeight))
    },
    buildPatrolPointFromGesture(startPoint, endPoint) {
      const mapStartPoint = {
        x: imgToMap({ mapData: this.mapData, x: startPoint.x }),
        y: imgToMap({ mapData: this.mapData, y: startPoint.y })
      }
      const mapEndPoint = {
        x: imgToMap({ mapData: this.mapData, x: endPoint.x }),
        y: imgToMap({ mapData: this.mapData, y: endPoint.y })
      }
      const yaw = Math.atan2(
        mapEndPoint.y - mapStartPoint.y,
        mapEndPoint.x - mapStartPoint.x
      )
      const orientation = createQuaternionFromYaw(yaw)

      return {
        mapPoint: {
          x: mapStartPoint.x,
          y: mapStartPoint.y,
          z: 0,
          yaw,
          orientation
        },
        imagePoint: {
          x: startPoint.x,
          y: startPoint.y,
          yaw,
          orientation
        }
      }
    },
    selectPointFromTouch(touch) {
      const point = this.getImagePointFromTouch(touch)
      if (!point) {
        return
      }

      this.$store.state.patrol_arr_area = [point]
      this.$store.state.patrol_arr = [{
        x: imgToMap({ mapData: this.mapData, x: point.x }),
        y: imgToMap({ mapData: this.mapData, y: point.y })
      }]
    },
    selectPatrolPoint(touch) {
      const endPoint = this.getImagePointFromTouch(touch)
      const draft = this.directionDraft
      this.clearDirectionDraft()

      if (!draft || !endPoint) {
        return
      }

      const startPoint = { x: draft.startX, y: draft.startY }
      if (this.getGestureDistance(startPoint, endPoint) < this.patrolDirectionMinDistance) {
        this.$message('请按住点位后滑动一小段距离来确定方向')
        return
      }

      const nextPoint = this.buildPatrolPointFromGesture(startPoint, endPoint)
      this.$store.state.patrol_arr_area.push(nextPoint.imagePoint)
      this.$store.state.patrol_arr.push(nextPoint.mapPoint)
    },
    selectRelocationPoint(touch) {
      const endPoint = this.getImagePointFromTouch(touch)
      const draft = this.directionDraft
      this.clearDirectionDraft()

      if (!draft || !endPoint) {
        return
      }

      const startPoint = { x: draft.startX, y: draft.startY }
      if (this.getGestureDistance(startPoint, endPoint) < this.patrolDirectionMinDistance) {
        this.$message('请按住点位后滑动一小段距离来确定重定位角度')
        return
      }

      const nextPoint = this.buildPatrolPointFromGesture(startPoint, endPoint)
      this.$store.state.patrol_arr_area = [nextPoint.imagePoint]
      this.$store.state.patrol_arr = [nextPoint.mapPoint]
      this.$emit('relocation-selected', nextPoint.mapPoint)
    },
    selectNavigationTargetPoint(touch) {
      const endPoint = this.getImagePointFromTouch(touch)
      const draft = this.directionDraft
      this.clearDirectionDraft()

      if (!draft || !endPoint) {
        return
      }

      const startPoint = { x: draft.startX, y: draft.startY }
      if (this.getGestureDistance(startPoint, endPoint) < this.patrolDirectionMinDistance) {
        this.$message('请按住点位后滑动一小段距离来确定导航角度')
        return
      }

      const nextPoint = this.buildPatrolPointFromGesture(startPoint, endPoint)
      this.$store.state.patrol_arr_area = [nextPoint.imagePoint]
      this.$store.state.patrol_arr = [nextPoint.mapPoint]
      this.$emit('navigation-target-selected', nextPoint.mapPoint)
    },
    getPoints() {
      const msg = new ROSLIB.ServiceRequest({
        map_id: this.$store.state.nowMap.id * 1,
        data_type: 'waypoint_node'
      });
      NavigationPointsGet.callService(
        msg,
        result => {
          try {
            let points = JSON.parse(result.message);
            let res = points.map(p => {
              p.point_list = JSON.parse(p.point_list)
              return {
                x: p.point_list.position.x,
                y: p.point_list.position.y,
                name: p.point_list.name,
                id: p.id
              }
            })
            this.$store.state.navigationMapPoints = res;
            this.$store.state.navigationImgPoints = res.map(it => ({
              x: mapToImg({ mapData: this.mapData, x: it.x }),
              y: mapToImg({ mapData: this.mapData, y: it.y }),
              name: it.name,
              id: it.id
            }));
            console.log("[  NavigationPointsGet OK]-points", res);
          } catch (error) {
            console.log(error)
            this.$message("获取定点导航点位列表失败");
          }
          console.log("[  NavigationPointsGet OK]-61", result);
        },
        result => {
          console.log("[  NavigationPointsGet ERR]-61", result);
        }
      );
    },
  }
};
</script>

<style scoped>
.map {
  position: relative;
  top: 0;
  height: calc(100% - 70px);
  width: 1380px;
  border-radius: 5px;
  background: #526cad;
  overflow: hidden;
  margin-top: 30px;
  margin-left: 30px;
}

.map_box1 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 0;
}

.loc_robot {
  position: absolute;
  top: 0;
  left: 0;
}

.map_box2 {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 4;
}

.plan_path {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 3;
  overflow: visible;
  pointer-events: none;
}

.plan_path polyline {
  fill: none;
  stroke: #37f2ff;
  stroke-width: 8;
  stroke-linecap: round;
  stroke-linejoin: round;
  opacity: 0.9;
}

.direction_layer {
  position: absolute;
  top: 0;
  left: 0;
  z-index: 5;
  overflow: visible;
  pointer-events: none;
}

.direction_line {
  stroke-width: 2;
  stroke-linecap: round;
}

.direction_arrow {
  stroke: none;
}

.direction_badge {
  fill: rgba(235, 255, 244, 0.96);
}

.direction_label {
  fill: #350af4;
  font-weight: 700;
  text-anchor: middle;
  dominant-baseline: middle;
  pointer-events: none;
}

.direction_preview {
  opacity: 0.95;
}

.rubber_sel {
  position: absolute;
  width: 150px;
  height: 50px;
  text-align: center;
  line-height: 50px;
  border: 1px solid #948f8f;
  background: #409eff;
  color: #fff;
  border-radius: 11px;
  margin-left: 50px;
  margin-top: 30px;
}

.rubber_sel2 {
  background: #d9dbdb;
  color: #000;
}

#operate {
  transform-origin: left top;
  pointer-events: none;
}

#operate2 {
  position: absolute;
  top: 20px;
  left: 80px;
  z-index: 20;
}

.img2 {
  position: absolute;
  right: 0;
  opacity: 0.7;
}

.show_img {
  background: #615555;
  opacity: 0.5;
  border: 1px solid;
  position: absolute;
  margin-top: 0px;
  margin-left: 0px;
  transform-origin: left top;
  touch-action: none;
}

.patrol_list {
  margin-left: 5%;
  margin-top: 1%;
  font-size: 12px;
  width: 83%;
}

.list-complete-item {
  width: 75px;
  display: inline-block;
  height: 25px;
  line-height: 25px;
  margin-left: 3px;
  margin-top: 3px;
  position: relative;
}

.map_tool {
  position: fixed;
  width: 170px;
  height: 120px;
  left: 20px;
  bottom: 20px;
  z-index: 5;
  display: flex;
  flex-wrap: wrap;
  align-content: space-between;
  justify-content: space-between;
}

.map_tool div {
  height: 50px;
  width: 50px;
  font-size: 30px;
  background: rgba(26, 26, 26, 0.6);
  color: #505050;
  box-shadow: 1px 1px 11px #415d5d;
  display: flex;
  justify-content: center;
  align-items: center;
  border-radius: 6px;
  transition: transform 0.1s linear;
}

.cli_box {
  box-shadow: inset 1px 1px 11px #436c6c !important;
}

.tram1 {
  transform: translate(60px, 60px);
}

.tram2 {
  transform: translate(0px, 60px);
}

.tram3 {
  transform: translate(-60px, 60px);
}

.tram4 {
  transform: translate(60px, 0px);
}

.tram5 {
  z-index: 2;
  background: #709080 !important;
  transform: rotate(90deg);
}

.tram6 {
  transform: translate(-60px, 0px);
}

.tram7 {
  transform: translate(60px, -60px);
}

.tram8 {
  transform: translate(0px, -60px);
}

.tram9 {
  transform: translate(-60px, -60px);
}

.map_point {
  position: absolute;
  top: 0;
  left: 0;
}

.robot {
  position: absolute;
  width: 24px;
  height: 24px;
  top: 0;
  left: 0;
  z-index: 11;
  transform-origin: 50% 50%;
  background: linear-gradient(
    180deg,
    rgb(255, 239, 133) 0%,
    rgb(255, 84, 84) 100%
  );
  clip-path: polygon(50% 0%, 100% 100%, 50% 74%, 0% 100%);
  box-shadow: -1px -2px 3px -5px rgb(249, 249, 249),
    4px 4px 10px -5px rgba(0, 0, 0, 0.3);
}

.pointNum {
  display: inline-block;
  position: absolute;
  font-size: 1.5rem;
  color: #000;
  top: 10px;
  left: 0;
  width: 50px;
  text-align: center;
}

.charge {
  width: 20px;
  height: 20px;
  background: #ccc;
  position: absolute;
  top: 0;
  left: 0;
  border-radius: 50%;
  animation: mymove 3s infinite;
}

@keyframes mymove {
  0% {
    transform: scale(1);
    opacity: 1;
  }

  100% {
    transform: scale(2);
    opacity: 0.1;
  }
}

.active {
  position: fixed;
  bottom: 50px;
  left: 50px;
}
.zoom {
  position: fixed;
  bottom: 50px;
  left: 1130px;
}
</style>
