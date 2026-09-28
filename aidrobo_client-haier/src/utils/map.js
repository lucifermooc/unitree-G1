export function clampMapOffset(value, maxOffset) {
  const min = Math.min(0, maxOffset);
  const max = Math.max(0, maxOffset);
  return Math.max(min, Math.min(value, max));
}

// checkEraseLines(leftTopX, leftTopY) {
//   const eraserSize = this.eraserSize;
//   const rx = leftTopX; // 橡皮擦左上点x
//   const ry = leftTopY; // 橡皮擦左上点y
//
//   if(this.linearCurveArr) {
//     let hasIntersect = false;
//     this.linearCurveArr = this.linearCurveArr.filter(line => {
//       const [{x: x1, y: y1}, {x: x2, y: y2}] = line;
//       const isIntersect = lineRectIntersects([x1, y1, x2, y2], [rx, ry, eraserSize, eraserSize,], 3);
//       hasIntersect = hasIntersect || isIntersect;
//       return !isIntersect;
//     })
//     // 相交的线则擦除掉，需要重绘
//     if(hasIntersect) {
//       let ctx = this.operate_txc;
//       ctx.clearRect(0, 0, this.d_width, this.d_height);
//       this.initBarrier()
//       this.$store.state.eraserArr.forEach(item => {
//         const eraseX = item.x - (this.eraserSize/2);
//         const eraseY = item.y - (this.eraserSize/2);
//         ctx.fillRect(eraseX, eraseY, this.eraserSize, this.eraserSize);
//       })
//     }
//   }
// }

/**
 * 检测线段与矩形是否相交
 * @param {number} x1, y1 - 线段起点
 * @param {number} x2, y2 - 线段终点
 * @param {number} rx, ry - 矩形左上角
 * @param {number} rw, rh - 矩形宽高
 * @param {number} lineWidth - 线段宽度（可选，增加检测精度）
 * @returns {boolean}
 */
export function lineRectIntersects(
  [x1, y1, x2, y2],
  [rx, ry, rw, rh],
  lineWidth = 0
) {
  const halfWidth = lineWidth / 2;

  // 1. 计算矩形中心点
  const rectCenterX = rx + rw / 2;
  const rectCenterY = ry + rh / 2;

  // 2. 计算矩形的"扩展半径"（从中心到角的距离）
  const rectHalfW = rw / 2;
  const rectHalfH = rh / 2;

  // 3. 点到线段的最短距离
  const dist = pointToLineSegmentDistance(rectCenterX, rectCenterY, x1, y1, x2, y2);

  // 4. 如果矩形中心到线段的距离 < (线段半宽 + 矩形半对角线)，可能相交
  const maxDist = halfWidth + Math.sqrt(rectHalfW * rectHalfW + rectHalfH * rectHalfH);

  if (dist > maxDist) {
    return false; // 肯定不相交
  }

  // 5. 精确检测：检查矩形四个角和四条边
  return (
    // 检查矩形的四个角是否在线段的"胶囊体"内
    pointToLineSegmentDistance(rx, ry, x1, y1, x2, y2) <= halfWidth ||
    pointToLineSegmentDistance(rx + rw, ry, x1, y1, x2, y2) <= halfWidth ||
    pointToLineSegmentDistance(rx, ry + rh, x1, y1, x2, y2) <= halfWidth ||
    pointToLineSegmentDistance(rx + rw, ry + rh, x1, y1, x2, y2) <= halfWidth ||

    // 检查线段两端点是否在矩形内
    pointInRect(x1, y1, rx, ry, rw, rh) ||
    pointInRect(x2, y2, rx, ry, rw, rh) ||

    // 检查线段是否与矩形四条边相交
    lineSegmentsIntersect(x1, y1, x2, y2, rx, ry, rx + rw, ry) ||
    lineSegmentsIntersect(x1, y1, x2, y2, rx + rw, ry, rx + rw, ry + rh) ||
    lineSegmentsIntersect(x1, y1, x2, y2, rx + rw, ry + rh, rx, ry + rh) ||
    lineSegmentsIntersect(x1, y1, x2, y2, rx, ry + rh, rx, ry)
  );
}

/**
 * 计算点到线段的最短距离
 */
function pointToLineSegmentDistance(px, py, x1, y1, x2, y2) {
  const dx = x2 - x1;
  const dy = y2 - y1;

  if (dx === 0 && dy === 0) {
    // 线段退化为点
    return Math.sqrt((px - x1) * (px - x1) + (py - y1) * (py - y1));
  }

  // 计算投影参数 t
  let t = ((px - x1) * dx + (py - y1) * dy) / (dx * dx + dy * dy);
  t = Math.max(0, Math.min(1, t)); // 限制在 [0, 1] 范围内

  // 计算投影点
  const projX = x1 + t * dx;
  const projY = y1 + t * dy;

  // 返回距离
  return Math.sqrt((px - projX) * (px - projX) + (py - projY) * (py - projY));
}

/**
 * 判断点是否在矩形内
 */
function pointInRect(px, py, rx, ry, rw, rh) {
  return px >= rx && px <= rx + rw && py >= ry && py <= ry + rh;
}

/**
 * 判断两条线段是否相交
 */
function lineSegmentsIntersect(x1, y1, x2, y2, x3, y3, x4, y4) {
  const denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1);

  if (denom === 0) {
    return false; // 平行或共线
  }

  const ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom;
  const ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom;

  return ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1;
}
