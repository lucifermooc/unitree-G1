export const changeStr = function (res) {
  const { data, info: { width, height, resolution, origin: { position: { x, y } } } } = res
  const uint = (data) => {
    const uint8Array = new Uint8Array(data);
    const decoder = new TextDecoder('utf-8');
    return decoder.decode(uint8Array);
  }
  return {
    src: 'data:image/jpg;base64,'+uint(data),
    width,
    height,
    resolution,
    positionX: x,
    positionY: y,
  }
}
export const mapToImg = ({ mapData, y, x }) => {
  if (y !== undefined && y !== null) {
    return mapData.height - (y - mapData.positionY) / mapData.resolution;
  }
  if (x !== undefined && x !== null) {
    return (x - mapData.positionX) / mapData.resolution;
  }
}
export const imgToMap = ({ mapData, y, x }) => {
  if (y !== undefined && y !== null) {
    return(mapData.height - y) * mapData.resolution + mapData.positionY;
  }
  if (x !== undefined && x !== null) {
    return  x * mapData.resolution +  mapData.positionX;
  }
}
// 角度转四元数(坐标系：X 向右，Y 向上，Z 垂直屏幕向外)
export function angleToQuaternion(deg) {
  // 1 角度归一化到 [-180, 180)
  let normalizedYaw = ((deg + 180) % 360 + 360) % 360 - 180;

  const rad = deg * Math.PI / 180; // 角度先转成欧拉角
  const half = rad / 2;

  return {
    x: 0,
    y: 0,
    z: Math.sin(half),
    w: Math.cos(half),
  };
}
// 四元数转角度
export function quatToDegrees(q) {
  const { w, x, y, z } = q;
  const yawRad = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  let deg = yawRad * 180 / Math.PI;
  if (deg < 0) deg += 360;
  return Math.round(deg); // 忽略0.999999999这种的精度问题
}

/**
 * 获取地图图片上指定像素点的颜色
 * @param {HTMLImageElement} imgEl - 地图 img 元素（已加载完成）
 * @param {number} imgX - 图片像素 X（原始分辨率，非 CSS 缩放后）
 * @param {number} imgY - 图片像素 Y（原始分辨率，非 CSS 缩放后）
 * @returns {{ r, g, b, a, hex } | null}
 */
export function getMapPixelColor(imgEl, imgX, imgY) {
  if (!imgEl || !imgEl.complete) return null;
  // 用 imgEl.naturalWidth/Height 保证是原始尺寸
  const w = imgEl.naturalWidth;
  const h = imgEl.naturalHeight;
  if (imgX < 0 || imgY < 0 || imgX >= w || imgY >= h) return null;

  // 复用或创建隐藏 canvas（挂在 imgEl 上避免重复创建）
  let canvas = imgEl._colorCanvas;
  if (!canvas || canvas._w !== w || canvas._h !== h) {
    canvas = document.createElement('canvas');
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(imgEl, 0, 0, w, h);
    canvas._w = w;
    canvas._h = h;
    imgEl._colorCanvas = canvas;
  }

  const ctx = canvas.getContext('2d');
  const [r, g, b, a] = ctx.getImageData(Math.round(imgX), Math.round(imgY), 1, 1).data;
  const hex = '#' + [r, g, b].map(v => v.toString(16).padStart(2, '0')).join('');
  return { r, g, b, a, hex };
}

/** 检查以 (cx, cy) 为圆心、radiusPx 为半径的圆形区域内所有像素是否均为白色
 * @param {HTMLImageElement} imgEl
 * @param {number} cx - 圆心 X（图片像素坐标）
 * @param {number} cy - 圆心 Y（图片像素坐标）
 * @param {number} radiusPx - 半径（像素）
 * @param {number} threshold - 白色阈值，默认 240 (0xF0)
 * @returns {boolean} true=全白可通行, false=存在非白色障碍物/膨胀层
 */
export function isRegionAllWhite(imgEl, cx, cy, radiusPx, threshold = 0xF0) {
  if (!imgEl || !imgEl.complete) return false;
  const w = imgEl.naturalWidth;
  const h = imgEl.naturalHeight;
  if (cx < 0 || cy < 0 || cx >= w || cy >= h) return false;

  // 复用 getMapPixelColor 的缓存 canvas
  let canvas = imgEl._colorCanvas;
  if (!canvas || canvas._w !== w || canvas._h !== h) {
    canvas = document.createElement('canvas');
    canvas.width = w;
    canvas.height = h;
    const ctx = canvas.getContext('2d');
    ctx.drawImage(imgEl, 0, 0, w, h);
    canvas._w = w;
    canvas._h = h;
    imgEl._colorCanvas = canvas;
  }

  const r = Math.ceil(radiusPx);
  const x0 = Math.max(0, Math.floor(cx - r));
  const y0 = Math.max(0, Math.floor(cy - r));
  const x1 = Math.min(w - 1, Math.ceil(cx + r));
  const y1 = Math.min(h - 1, Math.ceil(cy + r));
  const boxW = x1 - x0 + 1;
  const boxH = y1 - y0 + 1;

  const ctx = canvas.getContext('2d');
  const imageData = ctx.getImageData(x0, y0, boxW, boxH);
  const data = imageData.data;
  const r2 = radiusPx * radiusPx;

  for (let row = 0; row < boxH; row++) {
    for (let col = 0; col < boxW; col++) {
      const px = x0 + col;
      const py = y0 + row;
      const dx = px - cx;
      const dy = py - cy;
      if (dx * dx + dy * dy <= r2) {
        const i = (row * boxW + col) * 4;
        if (data[i] < threshold || data[i + 1] < threshold || data[i + 2] < threshold) {
          return false;
        }
      }
    }
  }
  return true;
}

// checkPointPlacement：检查该点位是否可以使用
const POINT_SAFE_DISTANCE = 0.2;
export function checkPointPlacement(_this, { imgPoint, mapPoint, lines = [], mapData, imgEl } = {}) {
  if (!_this || !imgPoint || !mapPoint) return true;
  const currentMapData = mapData || _this.mapData;
  const currentImgEl = imgEl || (_this.$refs && _this.$refs.img1);
  const radiusPx = POINT_SAFE_DISTANCE / currentMapData.resolution;

  // 障碍物周围0.2都不允许打点；0.2地图单位(米)为半径，区域内任一像素非白色则不可打点
  if (!isRegionAllWhite(currentImgEl, imgPoint.x, imgPoint.y, radiusPx)) {
    _this.$message('该位置不可达，请重新选择');
    return true;
  }

  // 距离禁行线太近不让打点
  const checkError = checkDistance(_this, mapPoint, lines);
  if (checkError) return true;

  // 可以打点
  return false;
}

export const normalizeFrameId = (frameId = '') => String(frameId).replace(/^\//, '')

export const rosTimeToMillis = (stamp = {}) => {
  const sec = Number(stamp.sec !== undefined ? stamp.sec : stamp.secs || 0)
  const nanosec = Number(stamp.nanosec !== undefined ? stamp.nanosec : stamp.nsecs || 0)

  if (!Number.isFinite(sec) || !Number.isFinite(nanosec)) {
    return null
  }

  return sec * 1000 + nanosec / 1000000
}

export const quaternionToYawRad = (orientation = {}) => {
  const x = Number(orientation.x || 0)
  const y = Number(orientation.y || 0)
  const z = Number(orientation.z || 0)
  const w = Number(orientation.w || 1)
  const sinyCosp = 2 * (w * z + x * y)
  const cosyCosp = 1 - 2 * (y * y + z * z)
  return Math.atan2(sinyCosp, cosyCosp)
}

export const quaternionToYawDeg = (orientation = {}) => {
  return (quaternionToYawRad(orientation) * 180) / Math.PI
}

export const createQuaternionFromYaw = (yaw = 0) => {
  const angle = Number(yaw || 0)
  return {
    x: 0,
    y: 0,
    z: Math.sin(angle / 2),
    w: Math.cos(angle / 2)
  }
}

export const resolvePatrolPointYaw = (point = {}, nextPoint = null, previousPoint = null) => {
  const yaw = Number(point.yaw)
  if (Number.isFinite(yaw)) {
    return yaw
  }

  const angle = Number(point.angle)
  if (Number.isFinite(angle)) {
    return angle
  }

  const orientation = point.orientation
  if (orientation) {
    return quaternionToYawRad(orientation)
  }

  if (nextPoint) {
    return Math.atan2(Number(nextPoint.y) - Number(point.y), Number(nextPoint.x) - Number(point.x))
  }

  if (previousPoint) {
    return Math.atan2(Number(point.y) - Number(previousPoint.y), Number(point.x) - Number(previousPoint.x))
  }

  return 0
}

export const normalizePatrolPoint = (point = {}, nextPoint = null, previousPoint = null) => {
  const normalizedPoint = {
    ...point,
    x: Number(point.x || 0),
    y: Number(point.y || 0),
    z: Number(point.z || 0)
  }
  const yaw = resolvePatrolPointYaw(normalizedPoint, nextPoint, previousPoint)

  return {
    ...normalizedPoint,
    yaw,
    orientation: createQuaternionFromYaw(yaw)
  }
}

export const normalizePatrolPoints = (points = []) => points.map((point, index) => normalizePatrolPoint(
  point,
  points[index + 1] || null,
  points[index - 1] || null
))

export const rotatePointByQuaternion = (point, quaternion = {}) => {
  const vector = {
    x: Number(point.x || 0),
    y: Number(point.y || 0),
    z: Number(point.z || 0)
  }
  const qx = Number(quaternion.x || 0)
  const qy = Number(quaternion.y || 0)
  const qz = Number(quaternion.z || 0)
  const qw = Number(quaternion.w || 1)
  const dotUV = qx * vector.x + qy * vector.y + qz * vector.z
  const dotUU = qx * qx + qy * qy + qz * qz
  const cross = {
    x: qy * vector.z - qz * vector.y,
    y: qz * vector.x - qx * vector.z,
    z: qx * vector.y - qy * vector.x
  }

  return {
    x: 2 * dotUV * qx + (qw * qw - dotUU) * vector.x + 2 * qw * cross.x,
    y: 2 * dotUV * qy + (qw * qw - dotUU) * vector.y + 2 * qw * cross.y,
    z: 2 * dotUV * qz + (qw * qw - dotUU) * vector.z + 2 * qw * cross.z
  }
}

export const applyTransformToPoint = (point, transform = {}) => {
  const rotated = rotatePointByQuaternion(point, transform.rotation)
  const translation = transform.translation || {}

  return {
    x: rotated.x + Number(translation.x || 0),
    y: rotated.y + Number(translation.y || 0),
    z: rotated.z + Number(translation.z || 0)
  }
}

export const invertQuaternion = (quaternion = {}) => ({
  x: -Number(quaternion.x || 0),
  y: -Number(quaternion.y || 0),
  z: -Number(quaternion.z || 0),
  w: Number(quaternion.w || 1)
})

export const composeQuaternions = (left = {}, right = {}) => {
  const ax = Number(left.x || 0)
  const ay = Number(left.y || 0)
  const az = Number(left.z || 0)
  const aw = Number(left.w || 1)
  const bx = Number(right.x || 0)
  const by = Number(right.y || 0)
  const bz = Number(right.z || 0)
  const bw = Number(right.w || 1)

  return {
    x: aw * bx + ax * bw + ay * bz - az * by,
    y: aw * by - ax * bz + ay * bw + az * bx,
    z: aw * bz + ax * by - ay * bx + az * bw,
    w: aw * bw - ax * bx - ay * by - az * bz
  }
}

export const invertTransform = (transform = {}) => {
  const rotation = invertQuaternion(transform.rotation)
  const translation = rotatePointByQuaternion({
    x: -Number((transform.translation || {}).x || 0),
    y: -Number((transform.translation || {}).y || 0),
    z: -Number((transform.translation || {}).z || 0)
  }, rotation)

  return { translation, rotation }
}

export const composeTransforms = (left = {}, right = {}) => ({
  translation: applyTransformToPoint(right.translation || {}, left),
  rotation: composeQuaternions(left.rotation, right.rotation)
})

export const buildTransformFromStamped = (transformStamped = {}, options = {}) => ({
  translation: {
    x: Number((((transformStamped.transform || {}).translation || {}).x) || 0),
    y: Number((((transformStamped.transform || {}).translation || {}).y) || 0),
    z: Number((((transformStamped.transform || {}).translation || {}).z) || 0)
  },
  rotation: {
    x: Number((((transformStamped.transform || {}).rotation || {}).x) || 0),
    y: Number((((transformStamped.transform || {}).rotation || {}).y) || 0),
    z: Number((((transformStamped.transform || {}).rotation || {}).z) || 0),
    w: Number((((transformStamped.transform || {}).rotation || {}).w) || 1)
  },
  timestampMs: rosTimeToMillis((transformStamped.header || {}).stamp),
  isStatic: !!options.isStatic
})

const TF_HISTORY_LIMIT = 120

const getTransformSamples = (entry) => {
  if (!entry) {
    return []
  }

  if (Array.isArray(entry.samples)) {
    return entry.samples
  }

  if (entry.translation && entry.rotation) {
    return [entry]
  }

  return []
}

const selectTransformSample = (entry, timestampMs = null) => {
  const samples = getTransformSamples(entry)
  if (!samples.length) {
    return null
  }

  if (timestampMs === null || timestampMs === undefined || entry.isStatic) {
    return samples[samples.length - 1]
  }

  let nearestSample = null
  let nearestDelta = Infinity

  for (let index = 0; index < samples.length; index += 1) {
    const sample = samples[index]
    if (!Number.isFinite(sample.timestampMs)) {
      continue
    }

    const delta = Math.abs(sample.timestampMs - timestampMs)
    if (delta < nearestDelta) {
      nearestSample = sample
      nearestDelta = delta
    }
  }

  return nearestSample || samples[samples.length - 1]
}

export const updateTransformGraph = (graph, transforms = [], options = {}) => {
  const maxSamples = Number(options.maxSamples || TF_HISTORY_LIMIT)
  const isStatic = !!options.isStatic

  transforms.forEach(item => {
    const parent = normalizeFrameId((item.header || {}).frame_id)
    const child = normalizeFrameId(item.child_frame_id)
    if (!parent || !child) {
      return
    }

    const key = `${parent}->${child}`
    const nextSample = buildTransformFromStamped(item, { isStatic })

    if (isStatic) {
      graph[key] = {
        isStatic: true,
        samples: [nextSample]
      }
      return
    }

    const samples = getTransformSamples(graph[key]).slice()
    const lastSample = samples[samples.length - 1]

    if (
      lastSample &&
      Number.isFinite(lastSample.timestampMs) &&
      Number.isFinite(nextSample.timestampMs) &&
      lastSample.timestampMs === nextSample.timestampMs
    ) {
      samples[samples.length - 1] = nextSample
    } else {
      samples.push(nextSample)
    }

    if (samples.length > 1) {
      samples.sort((left, right) => {
        const leftStamp = Number.isFinite(left.timestampMs) ? left.timestampMs : Number.MAX_SAFE_INTEGER
        const rightStamp = Number.isFinite(right.timestampMs) ? right.timestampMs : Number.MAX_SAFE_INTEGER
        return leftStamp - rightStamp
      })
    }

    if (samples.length > maxSamples) {
      samples.splice(0, samples.length - maxSamples)
    }

    graph[key] = {
      isStatic: false,
      samples
    }
  })
}

export const resolveTransform = (graph, fromFrame, toFrame, options = {}) => {
  const start = normalizeFrameId(fromFrame)
  const target = normalizeFrameId(toFrame)
  const timestampMs = typeof options === 'number' ? options : options.timestampMs
  if (!start || !target) {
    return null
  }
  if (start === target) {
    return {
      translation: { x: 0, y: 0, z: 0 },
      rotation: { x: 0, y: 0, z: 0, w: 1 }
    }
  }

  const visited = new Set([start])
  const queue = [{ frame: start, transform: null }]

  while (queue.length) {
    const current = queue.shift()
    const frame = current.frame

    const keys = Object.keys(graph)
    for (let index = 0; index < keys.length; index += 1) {
      const key = keys[index]
      const [parent, child] = key.split('->')
      let nextFrame = null
      let edgeTransform = null

      if (parent === frame) {
        nextFrame = child
        edgeTransform = selectTransformSample(graph[key], timestampMs)
      } else if (child === frame) {
        nextFrame = parent
        const sample = selectTransformSample(graph[key], timestampMs)
        edgeTransform = sample ? invertTransform(sample) : null
      }

      if (!nextFrame || !edgeTransform || visited.has(nextFrame)) {
        continue
      }

      const nextTransform = current.transform
        ? composeTransforms(current.transform, edgeTransform)
        : edgeTransform

      if (nextFrame === target) {
        return nextTransform
      }

      visited.add(nextFrame)
      queue.push({ frame: nextFrame, transform: nextTransform })
    }
  }

  return null
}

export const routerObj={
  'home':'首页',
  'map':'地图管理',
  'newMap':"新建地图",
  'editMap':"编辑地图",
  'seeMap':"查看地图",
  'site':"设置",
  'utility':"应用功能",
  'goPoint':"去位置点",
  'telecontrol':"遥控",
  'screen': "展示大屏",
}


// 判断点位到线段是否太近；return: checkError - true 则表示点太近；false 则表示点位正常
export function checkDistance(_this, point, lines) {
  if (!_this || !point || !Array.isArray(lines) || !lines.length) return false;
  const distances = lines.map(line =>
    getDistanceToLine(
      [point.x, point.y],
      [line.start.x, line.start.y, line.end.x, line.end.y]
    )
  );
  distances.sort((a, b) => a - b);
  const v = distances[0]; // 最短距离
  if (v <= POINT_SAFE_DISTANCE) {
    _this.$message("距离禁行线过近，无法设置点位");
    return true;
  }
  return false;
}
// 计算点 [x, y] 到线段 [x1, y1, x2, y2] 的最小距离
export function getDistanceToLine(point, line) {
  const [x1, y1, x2, y2] = line;
  const [x, y] = point;
  // 1. 计算线段的长度平方 (避免开根号，提升性能)
  // len_sq = (x2 - x1)^2 + (y2 - y1)^2
  let A = x - x1;
  let B = y - y1;
  let C = x2 - x1;
  let D = y2 - y1;
  let dot = A * C + B * D;
  let len_sq = C * C + D * D;
  let param = -1;
  // 避免除以 0 (即线段两端点重合的情况)
  if (len_sq !== 0) {
    param = dot / len_sq;
  }
  let xx, yy;
  // 2. 判断垂足位置
  if (param < 0) {
    // 垂足在 A 点外侧，最近点是 A
    xx = x1;
    yy = y1;
  } else if (param > 1) {
    // 垂足在 B 点外侧，最近点是 B
    xx = x2;
    yy = y2;
  } else {
    // 垂足在线段内部，计算垂足坐标
    xx = x1 + param * C;
    yy = y1 + param * D;
  }
  // 3. 计算点 (x,y) 到最近点 (xx, yy) 的欧几里得距离
  let dx = x - xx;
  let dy = y - yy;
  return Math.sqrt(dx * dx + dy * dy);
}
