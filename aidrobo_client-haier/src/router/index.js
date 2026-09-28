import Vue from 'vue'
import Router from 'vue-router'
// import patrol from '@/components/tool/patrol'
// import charge from '@/components/tool/charge'
// import point from '@/components/tool/point'
// import control from '@/components/tool/control'
import Home from '@/views/home'

import Utility from '@/views/utility/index'

import relocation from '@/views/utility/relocation'
import goPoint from '@/views/utility/goPoint'
import navigation from '@/views/utility/navigation'
import patrol from '@/views/utility/patrol'
import charge from '@/views/utility/charge'
import telecontrol from '@/views/utility/telecontrol'
import following from '@/views/utility/following'
import gesture from '@/views/utility/gesture'
import objectRecognition from '@/views/utility/objectRecognition'
import screen from '@/views/utility/screen'
import calibration from '@/views/utility/calibration'
import aging from '@/views/utility/aging'
import radarCalibration from '@/views/utility/radarCalibration'
import cliffCalibration from '@/views/utility/cliffCalibration'

import Site from '@/views/site'
import Map from '@/views/map/index'
import NewMap from '@/views/map/newMap'
import EditMap from '@/views/map/editMap'
import SeeMap from '@/views/map/seeMap'
Vue.use(Router)

// const originalPush = Router.prototype.push
// Router.prototype.push = function push (location) {
//   return originalPush.call(this, location).catch(err => err)
// }
export default new Router({
  routes: [
    {
      path: '/',
      name:'home',
      component: Home,
    },
    {
      path: '/map',
      name:'map',
      component: Map,
      // children: [
      //   {
      //     path: 'control',
      //     name: 'control',
      //     component: control
      //   },
      //   {
      //     path: 'patrol',
      //     name: 'patrol',
      //     component: patrol
      //   },
      //   {
      //     path: 'charge',
      //     name: 'charge',
      //     component: charge
      //   },
      //   {
      //     path: 'point',
      //     name: 'point',
      //     component: point
      //   }
      // ]
    },
    {
      path: '/newMap',
      name: 'newMap',
      component: NewMap,
    },
    {
      path: '/editMap',
      name: 'editMap',
      component: EditMap,
    },
    {
      path: '/seeMap',
      name: 'seeMap',
      component: SeeMap,
    },
    {
      path: '/site',
      name:'site',
      component: Site,
    },
    {
      path: '/utility',
      name: 'utility',
      component: Utility,
    },
    {
      path: '/utility/relocation',
      name:'relocation',
      component: relocation,
    },
    {
      path: '/utility/goPoint',
      name:'goPoint',
      component: goPoint,
    },
    {
      path: '/utility/navigation',
      name:'navigation',
      component: navigation,
    },
    {
      path: '/utility/patrol',
      name:'patrol',
      component: patrol,
    },
    {
      path: '/utility/charge',
      component: charge,
    },
    {
      path: '/utility/telecontrol',
      name:'telecontrol',
      component: telecontrol,
    },
    {
      path: '/utility/following',
      component: following,
    },
    {
      path: '/utility/gesture',
      component: gesture,
    },
    {
      path: '/utility/object-recognition',
      component: objectRecognition,
    },
    {
      path: '/utility/screen',
      name: 'screen',
      component: screen,
    },
    {
      path: '/utility/calibration',
      name: 'calibration',
      component: calibration,
    },
    {
      path: '/utility/aging',
      name: 'aging',
      component: aging,
    },
    {
      path: '/utility/radarCalibration',
      name: 'radarCalibration',
      component: radarCalibration,
    },
    {
      path: '/utility/cliffCalibration',
      name: 'cliffCalibration',
      component: cliffCalibration,
    },
  ]
})
