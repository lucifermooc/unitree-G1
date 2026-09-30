/* G1 控制台：只包含《G1_Communication_Documentation》里的功能 + 语义地图。
 * 全部通过 rosbridge（ws://<机器人IP>:9090）调用机器人上的 ROS 话题 / 服务，接口名和参数与原前端一致。 */
"use strict";

const $ = (id) => document.getElementById(id);
const SRV = {
  modeSet: ["/mode_set", "aid_robot_msgs/srv/StatusChange"],
  saveMap: ["/aid_save_map", "aid_robot_msgs/srv/MapOperation"],
  addMap: ["/add_map", "aid_robot_msgs/srv/MapOperationAdd"],
  mapList: ["/get_map_list", "aid_robot_msgs/srv/MapList"],
  deleteMap: ["/delete_map", "aid_robot_msgs/srv/OperationDelete"],
  updateMap: ["/update_map", "aid_robot_msgs/srv/OperationUpdate"],
  mapImage: ["/get_map_image", "aid_robot_msgs/srv/MapImage"],
  setCurrent: ["/set_current_map_id", "aid_robot_msgs/srv/SetCurrentMap"],
  getCurrent: ["/get_current_map_id", "aid_robot_msgs/srv/GetCurrentMap"],
  addPoint: ["/add_point", "aid_robot_msgs/srv/OperationAdd"],
  updatePoint: ["/update_point", "aid_robot_msgs/srv/OperationUpdate"],
  deletePoint: ["/delete_point", "aid_robot_msgs/srv/OperationDelete"],
  pointList: ["/get_map_point_list", "aid_robot_msgs/srv/MapLinkedDataList"],
  patrol: ["/patrol_control", "aid_robot_msgs/srv/PatrolControl"],
  setForbidden: ["/set_forbidden", "aid_robot_msgs/srv/ForbiddenSet"],
  getForbidden: ["/get_forbidden", "aid_robot_msgs/srv/ForbiddenGet"],
  drawForbidden: ["/aid_draw_forbidden_line", "aid_robot_msgs/srv/DrawPicture"],
  mapEditor: ["/map_editor", "aid_robot_msgs/srv/DrawPicture"],
  semSearch: ["/semantic_map/search", "aid_robot_msgs/srv/SetString"],
  semGo: ["/semantic_map/go", "aid_robot_msgs/srv/SetString"],
  semRebuild: ["/semantic_map/rebuild", "std_srvs/srv/Trigger"],
  semHistory: ["/semantic_map/history", "std_srvs/srv/Trigger"],
};
const TASK_STATUS = ["空闲", "执行中", "成功", "失败", "暂停", "已取消"];
const TASK_TYPE = ["单点导航", "巡逻"];
const MODE_NAME = { idle: "空闲", mapping: "建图", continue_mapping: "继续建图", localization: "定位", patrol: "导航",
  remote_control: "遥控" };

const S = {
  ros: null, connected: false,
  map: null,          // {img, width, height, resolution, ox, oy}
  mapId: 0, mapName: "",
  maps: [], points: [], nogo: [],
  robot: null,        // {x, y, yaw}
  robotStatus: "", task: null, battery: null,
  view: { scale: 1, ox: 0, oy: 0 },
  tab: "maps",
  tool: null,         // point | nogo | eraser | relocate
  selectedId: null, pending: null, drag: null,
  nogoStart: null, eraser: [],
  patrolSel: [], semHitId: null, mappingPreview: false, semLog: [],
};
window.app = S;  // 方便调试和自动化测试

// ---------------- 通用工具 ----------------
const round = (v, n = 3) => Math.round(v * 10 ** n) / 10 ** n;
const yawToQuat = (yaw) => ({ x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) });
const quatToYaw = (q) => Math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
const deg = (rad) => Math.round((rad * 180) / Math.PI);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function toast(text) {
  const t = $("toast");
  t.textContent = text; t.classList.add("show");
  clearTimeout(toast.timer); toast.timer = setTimeout(() => t.classList.remove("show"), 2500);
}
function log(text, cls = "") {
  const li = document.createElement("li");
  li.className = cls;
  li.textContent = `${new Date().toLocaleTimeString()} ${text}`;
  $("log").prepend(li);
  while ($("log").children.length > 80) $("log").lastChild.remove();
}
function short(obj) {
  const s = typeof obj === "string" ? obj : JSON.stringify(obj);
  return s.length > 160 ? s.slice(0, 157) + "..." : s;
}

// ---------------- rosbridge ----------------
function call([name, type], args = {}, timeoutMs = 60000) {
  return new Promise((resolve, reject) => {
    if (!S.connected) { reject(new Error("未连接机器人")); return; }
    const srv = new ROSLIB.Service({ ros: S.ros, name, serviceType: type });
    const timer = setTimeout(() => reject(new Error(`${name} 超时`)), timeoutMs);
    srv.callService(new ROSLIB.ServiceRequest(args), (res) => {
      clearTimeout(timer);
      log(`${name} ${short(args)} → ${short(res)}`, "ok");
      resolve(res);
    }, (err) => {
      clearTimeout(timer);
      log(`${name} ${short(args)} ✗ ${err}`, "err");
      reject(new Error(`${name}：${err}`));
    });
  });
}
function publish(name, type, msg) {
  new ROSLIB.Topic({ ros: S.ros, name, messageType: type }).publish(new ROSLIB.Message(msg));
  log(`publish ${name} ${short(msg)}`);
}
const topics = {};
function subscribe(name, type, cb, opts = {}) {
  unsubscribe(name);
  topics[name] = new ROSLIB.Topic({ ros: S.ros, name, messageType: type, ...opts });
  topics[name].subscribe(cb);
}
function unsubscribe(name) {
  if (topics[name]) { topics[name].unsubscribe(); delete topics[name]; }
}
async function guard(fn) {  // 按钮回调：统一报错
  try { return await fn(); } catch (e) { toast(e.message); log(e.message, "err"); }
}

function connect(host) {
  if (S.ros) { S.ros.close(); }
  const url = host.startsWith("ws") ? host : `ws://${host}:9090`;
  try { localStorage.setItem("rosIP", host); } catch (e) { /* 忽略 */ }
  S.ros = new ROSLIB.Ros();
  S.ros.on("connection", onConnected);
  S.ros.on("close", () => { setConn(false); });
  S.ros.on("error", () => { setConn(false); toast(`连不上 ${url}`); });
  S.ros.connect(url);
  log(`连接 ${url}`);
}
function setConn(on) {
  S.connected = on;
  $("stConn").classList.toggle("on", on);
  $("stConn").lastElementChild.textContent = on ? "已连接" : "未连接";
}
async function onConnected() {
  setConn(true);
  toast("已连接");
  subscribe("/base_link_pose", "geometry_msgs/msg/PoseStamped", (m) => {
    S.robot = { x: m.pose.position.x, y: m.pose.position.y, yaw: quatToYaw(m.pose.orientation) };
    draw();
  }, { throttle_rate: 200, queue_length: 1 });
  // G1 的 battery_state_bridge：percentage 为 0~1；charge/capacity 是 NaN，充电状态看 power_supply_status。
  // 用 CBOR 编码订阅：JSON 表示不了 NaN，CBOR 可以。
  subscribe("/battery_state", "sensor_msgs/msg/BatteryState", (m) => {
    const p = m.percentage > 1 ? m.percentage : m.percentage * 100;
    const st = { 1: " 充电中", 4: " 已充满" }[m.power_supply_status] || "";
    $("stBattery").textContent = Number.isFinite(p) ? `${Math.round(p)}%${st}` : "--";
  }, { throttle_rate: 1000, compression: "cbor" });
  subscribe("/robot_status", "std_msgs/msg/String", (m) => {
    S.robotStatus = m.data;
    const [slam, control] = m.data.split("+");
    $("stMode").textContent = `${MODE_NAME[slam] || slam} / ${MODE_NAME[control] || control}`;
    document.querySelectorAll(".modes button").forEach((b) =>
      b.classList.toggle("active", b.dataset.mode === slam || b.dataset.mode === control));
    updateMappingInfo();
  });
  subscribe("/task_status", "aid_robot_msgs/msg/AidTaskStatus", (m) => {
    S.task = m;
    const text = `${TASK_TYPE[m.task_type] || m.task_type} · ${TASK_STATUS[m.status] || m.status}`;
    $("stTask").textContent = m.status === 0 ? "空闲" : text;
    $("navTask").textContent = $("stTask").textContent;
  });
  subscribe("/semantic_map/debug", "std_msgs/msg/String", (m) => addSemLog([JSON.parse(m.data)]));
  await guard(loadMaps);
  await guard(loadCurrentMap);
  await guard(loadSemLog);
}

// ---------------- 地图数据 ----------------
function gridToMap(grid) {
  // map_manager_server 把 JPEG 的 base64 字符串逐字节放进 OccupancyGrid.data（与原前端 changeStr 一致）
  const d = grid.data;
  const b64 = typeof d === "string" ? atob(d) : new TextDecoder().decode(new Uint8Array(d));
  return new Promise((resolve, reject) => {
    const img = new Image();
    img.onload = () => resolve({
      img, width: grid.info.width, height: grid.info.height, resolution: grid.info.resolution,
      ox: grid.info.origin.position.x, oy: grid.info.origin.position.y,
    });
    img.onerror = () => reject(new Error("地图图片解码失败"));
    img.src = "data:image/jpg;base64," + b64;
  });
}
async function loadMaps() {
  const res = await call(SRV.mapList);
  S.maps = res.success ? JSON.parse(res.map_list || "[]") : [];
  renderMaps();
}
async function loadCurrentMap() {
  const cur = await call(SRV.getCurrent);
  S.mapId = cur.success ? cur.map_id : 0;
  S.mapName = cur.success ? cur.map_name : "";
  $("stMap").textContent = S.mapName || "未选择";
  renderMaps();
  if (!S.mapId) { S.map = null; S.points = []; S.nogo = []; renderAll(); return; }
  await reloadMapImage();
  await loadPoints();
  await loadNogo();
}
async function reloadMapImage(fit = true) {
  const res = await call(SRV.mapImage, { id: S.mapId });
  if (!res.success) throw new Error("获取地图图片失败");
  const keepView = !fit && S.map;
  S.map = await gridToMap(res.map);
  if (!keepView) fitView();
  draw();
}
async function loadPoints() {
  const res = await call(SRV.pointList, { map_id: S.mapId, data_type: "waypoint_node" });
  S.points = [];
  for (const row of JSON.parse(res.message || "[]")) {
    try {
      const d = typeof row.point_list === "string" ? JSON.parse(row.point_list) : row.point_list;
      S.points.push({ id: row.id, frame_id: row.frame_id || "map", name: d.name || "", description: d.description || "",
        x: +d.position.x, y: +d.position.y, yaw: quatToYaw(d.orientation) });
    } catch (e) { log(`点位 ${row.id} 数据格式不对，跳过`, "err"); }
  }
  S.patrolSel = S.patrolSel.filter((id) => S.points.some((p) => p.id === id));
  renderAll();
}
async function loadNogo() {
  const res = await call(SRV.getForbidden, { map_id: S.mapId });
  S.nogo = (res.lines || []).map((l) => ({ x1: l.start.x, y1: l.start.y, x2: l.end.x, y2: l.end.y }));
  renderNogo(); draw();
}
function pointData(p) {  // 写进数据库的 JSON：原有字段 + 语义描述。坐标按 double 原样存，不取整
  return JSON.stringify({ position: { x: p.x, y: p.y, z: 0 }, orientation: yawToQuat(p.yaw),
    name: p.name, description: p.description });
}

// ---------------- 地图管理 ----------------
function renderMaps() {
  const ul = $("mapList"); ul.innerHTML = "";
  if (!S.maps.length) { ul.innerHTML = '<li class="s">还没有地图</li>'; return; }
  for (const m of S.maps) {
    const li = document.createElement("li");
    li.dataset.id = m.id;
    li.className = m.id === S.mapId ? "current" : "";
    const time = m.create_timestamp ? new Date(m.create_timestamp * 1000).toLocaleString() : "";
    li.innerHTML = `<div class="grow"><div class="t"></div><div class="s"></div></div><div class="actions">
      <button data-act="use">使用</button><button data-act="rename">重命名</button>
      <button data-act="delete" class="danger">删除</button></div>`;
    li.querySelector(".t").textContent = `${m.name}${m.id === S.mapId ? "（当前）" : ""}`;
    li.querySelector(".s").textContent = `id ${m.id} · ${time}`;
    li.querySelector('[data-act="use"]').onclick = () => guard(() => useMap(m));
    li.querySelector('[data-act="rename"]').onclick = () => startRename(li, m);
    li.querySelector('[data-act="delete"]').onclick = (e) => confirmThen(e.target, `确认删除`, () => guard(() => deleteMap(m)));
    ul.appendChild(li);
  }
}
function confirmThen(btn, text, fn) {  // 两次点击确认（不用弹窗）
  if (btn.dataset.armed) { fn(); return; }
  const old = btn.innerHTML;  // 模式按钮里有 <small>，按 HTML 还原
  btn.dataset.armed = "1"; btn.textContent = text;
  setTimeout(() => { delete btn.dataset.armed; btn.innerHTML = old; }, 3000);
}
async function useMap(m) {
  const r = await call(SRV.setCurrent, { id: m.id });
  if (!r.success) throw new Error("设置当前地图失败");
  const mode = await call(SRV.modeSet, { action: "localization" });
  toast(mode.message === "ok" ? `已切换到"${m.name}"，进入定位模式` : `已设为当前地图，但切换定位失败：${mode.message}`);
  await loadCurrentMap();
}
function startRename(li, m) {
  const t = li.querySelector(".t");
  const input = document.createElement("input");
  input.value = m.name; input.setAttribute("aria-label", "新名称");
  t.replaceWith(input); input.focus(); input.select();
  const done = async (save) => {
    const name = input.value.trim();
    if (save && name && name !== m.name) {
      await guard(async () => {
        const r = await call(SRV.updateMap, { id: m.id, data: JSON.stringify({ name }), data_type: "map" });
        if (!r.success) throw new Error(`重命名失败：${r.message}`);
        toast("已重命名");
        if (m.id === S.mapId) { S.mapName = name; $("stMap").textContent = name; }
      });
    }
    await guard(loadMaps);
  };
  input.addEventListener("keydown", (e) => { if (e.key === "Enter") done(true); if (e.key === "Escape") done(false); });
  input.addEventListener("blur", () => done(true), { once: true });
}
async function deleteMap(m) {
  const r = await call(SRV.deleteMap, { id: m.id, data_type: "map" });
  if (!r.success) throw new Error(`删除失败：${r.message}`);
  toast(`已删除"${m.name}"`);
  await loadMaps();
  if (m.id === S.mapId) await loadCurrentMap();
}

// ---------------- 建图 ----------------
function updateMappingInfo() {
  const mapping = S.robotStatus.startsWith("mapping");
  $("mappingInfo").textContent = mapping ? "建图中……推着/遥控机器人把场地走一遍，然后输入名称保存。" : "当前不在建图模式。";
  if (mapping && !S.mappingPreview) startPreview();
  if (!mapping && S.mappingPreview) stopPreview();
}
function startPreview() {
  S.mappingPreview = true;
  S.selectedId = null; S.pending = null;
  subscribe("/map_base64", "nav_msgs/msg/OccupancyGrid", async (m) => {
    const first = !S.map || !S.mappingPreview;
    S.map = await gridToMap(m);
    if (first || !S.previewFitted) { fitView(); S.previewFitted = true; }
    draw();
  }, { compression: "png" });
  renderAll();
}
function stopPreview() {
  S.mappingPreview = false; S.previewFitted = false;
  unsubscribe("/map_base64");
}
async function startMapping() {
  const r = await call(SRV.modeSet, { action: "mapping" });
  if (r.message !== "ok") throw new Error(`进入建图失败：${r.message}`);
  S.map = null; startPreview(); draw();
  toast("开始建图（新地图）");
}
async function continueMapping() {
  if (!S.mapId) throw new Error("先在“地图”里使用一张地图，再继续建图");
  const r = await call(SRV.modeSet, { action: "continue_mapping" });
  if (r.message !== "ok") throw new Error(`继续建图失败：${r.message}`);
  if (!$("mapName").value.trim()) $("mapName").value = `${S.mapName}-续建`;
  startPreview(); draw();  // 先显示当前地图，收到实时地图后替换
  toast(`在“${S.mapName}”上继续建图`);
}
async function cancelMapping() {
  const r = await call(SRV.modeSet, { action: "idle" });
  stopPreview();
  toast(r.message === "ok" ? "已取消建图，地图未保存（进入定位时会重新加载当前地图）" : `取消失败：${r.message}`);
  await loadCurrentMap();
}
async function saveMapping() {
  const name = $("mapName").value.trim();
  if (!name) throw new Error("先填地图名称");
  if (S.maps.some((m) => m.name === name)) throw new Error(`已经有叫"${name}"的地图了`);
  const file = `/maps/${Date.now()}`;
  $("btnMapSave").disabled = true;
  try {
    const s = await call(SRV.saveMap, { map_file_name: file }, 120000);
    if (!s.success) throw new Error(`保存地图失败：${s.message}`);
    const d = await call(SRV.addMap, { map_name: name, map_file: file });
    if (!d.success) throw new Error(`地图已保存但写入数据库失败：${d.message}`);
  } finally { $("btnMapSave").disabled = false; }
  stopPreview();
  $("mapName").value = "";
  toast(`地图"${name}"已保存。到"地图"里点"使用"开始用它`);
  await loadMaps();
  await loadCurrentMap();
}

// ---------------- 点位 ----------------
function currentPoint() { return S.pending || S.points.find((p) => p.id === S.selectedId) || null; }
function selectPoint(id) { S.pending = null; S.selectedId = id; renderAll(); }
function newPoint(x, y, yaw) {
  S.selectedId = null;
  S.pending = { id: null, name: "", description: "", x, y, yaw };
  renderAll(); $("pName").focus();
}
function cellWarning(x, y) {
  if (!S.map) return "";
  const res = S.map.resolution;
  const u = Math.floor((x - S.map.ox) / res), v = Math.floor(S.map.height - (y - S.map.oy) / res);
  if (u < 0 || v < 0 || u >= S.map.width || v >= S.map.height) return "⚠ 在地图范围外";
  const c = mapPixel(u, v);
  if (c === "occ") return "⚠ 这里是障碍物，机器人到不了";
  // 激光只扫到墙和家具的表面，内部在地图上是"未探索"；所以先看周围有没有障碍物
  const r = Math.round(0.2 / res);
  for (let dv = -r; dv <= r; dv++) {
    for (let du = -r; du <= r; du++) {
      if (du * du + dv * dv > r * r) continue;
      const uu = u + du, vv = v + dv;
      if (uu >= 0 && vv >= 0 && uu < S.map.width && vv < S.map.height && mapPixel(uu, vv) === "occ") {
        return "⚠ 离障碍物太近（0.2 米内），机器人可能到不了";
      }
    }
  }
  if (c === "unknown") return "⚠ 这里是未探索区域，机器人可能到不了";
  return "";
}
let pixelCache = null;
function mapPixel(u, v) {
  // 地图图片（JPEG）里：障碍物黑色，空闲 RGB(127,145,200)，未探索 RGB(82,108,170)。JPEG 会把细墙边缘晕开，
  // 所以按"离哪个参考色最近"来判断，而不是要求颜色完全相等。
  if (!pixelCache || pixelCache.img !== S.map.img) {
    const c = document.createElement("canvas");
    c.width = S.map.width; c.height = S.map.height;
    const ctx = c.getContext("2d"); ctx.drawImage(S.map.img, 0, 0);
    pixelCache = { img: S.map.img, data: ctx.getImageData(0, 0, c.width, c.height).data };
  }
  const i = (v * S.map.width + u) * 4, d = pixelCache.data;
  const dist = (r, g, b) => (d[i] - r) ** 2 + (d[i + 1] - g) ** 2 + (d[i + 2] - b) ** 2;
  const occ = dist(0, 0, 0), free = dist(127, 145, 200), unknown = dist(82, 108, 170);
  if (occ < free && occ < unknown) return "occ";
  return unknown < free ? "unknown" : "free";
}
function fillPointForm() {
  const p = currentPoint();
  $("pointForm").hidden = !p;
  if (!p) return;
  $("pointFormTitle").textContent = S.pending ? "新点位" : `编辑点位（id ${p.id}）`;
  $("pName").value = p.name; $("pDesc").value = p.description;
  // 朝向只为显示保留 2 位小数；没改过就沿用原值（见 readPointForm），避免保存时被显示值覆盖
  $("pX").value = p.x; $("pY").value = p.y; $("pYaw").value = $("pYaw").dataset.shown = round((p.yaw * 180) / Math.PI, 2);
  $("pWarn").textContent = cellWarning(p.x, p.y);
  $("btnPointDelete").hidden = $("btnPointGo").hidden = !!S.pending;
}
function readPointForm(p) {
  p.name = $("pName").value.trim(); p.description = $("pDesc").value.trim();
  const x = parseFloat($("pX").value), y = parseFloat($("pY").value), yawDeg = parseFloat($("pYaw").value);
  if (Number.isFinite(x)) p.x = x;
  if (Number.isFinite(y)) p.y = y;
  if (Number.isFinite(yawDeg) && $("pYaw").value !== $("pYaw").dataset.shown) p.yaw = (yawDeg * Math.PI) / 180;
}
async function savePoint() {
  const p = currentPoint();
  readPointForm(p);
  if (!p.name) throw new Error("名称不能为空");
  if (S.points.some((q) => q.name === p.name && q.id !== p.id)) throw new Error(`已经有叫"${p.name}"的点位了`);
  if (!S.mapId) throw new Error("先选择地图");
  if (S.pending) {
    const r = await call(SRV.addPoint, { map_id: S.mapId, frame_id: "map", data: pointData(p), data_type: "waypoint_node" });
    if (!r.success) throw new Error(`新增点位失败：${r.message}`);
    S.pending = null;
    await loadPoints();
    const added = S.points.filter((q) => q.name === p.name).pop();
    S.selectedId = added ? added.id : null;
  } else {
    const r = await call(SRV.updatePoint, { id: p.id, data: pointData(p), data_type: "waypoint_node" });
    if (!r.success) throw new Error(`修改点位失败：${r.message}`);
    await loadPoints();
  }
  renderAll();
  toast(`已保存"${p.name}"`);
}
async function deletePoint(p) {
  const r = await call(SRV.deletePoint, { id: p.id, data_type: "waypoint_node" });
  if (!r.success) throw new Error(`删除失败：${r.message}`);
  S.selectedId = null;
  await loadPoints();
  toast(`已删除"${p.name}"`);
}
function renderPoints() {
  const ul = $("pointList"); ul.innerHTML = "";
  for (const p of S.points) {
    const li = document.createElement("li");
    li.dataset.id = p.id;
    li.className = p.id === S.selectedId ? "sel" : "";
    li.innerHTML = '<div class="grow"><div class="t"></div><div class="s"></div></div>';
    li.querySelector(".t").textContent = p.name + (cellWarning(p.x, p.y) ? " ⚠" : "");
    li.querySelector(".s").textContent = `${p.description ? p.description + " · " : ""}(${round(p.x, 2)}, ${round(p.y, 2)}) ${deg(p.yaw)}°`;
    li.querySelector(".grow").onclick = () => { selectPoint(p.id); centerOn(p.x, p.y); };
    ul.appendChild(li);
  }
  if (!S.points.length) ul.innerHTML = '<li class="s">当前地图还没有点位</li>';
  // 导航下拉框和巡逻列表
  const sel = $("navPoint"), keep = sel.value;
  sel.innerHTML = S.points.map((p) => `<option value="${p.id}"></option>`).join("");
  [...sel.options].forEach((o, i) => { o.textContent = S.points[i].name; });
  if (S.points.some((p) => String(p.id) === keep)) sel.value = keep;
  const pl = $("patrolList"); pl.innerHTML = "";
  for (const p of S.points) {
    const li = document.createElement("li");
    const order = S.patrolSel.indexOf(p.id);
    li.innerHTML = `<label class="grow"><input type="checkbox"> <span class="order"></span> <span class="n"></span></label>`;
    const cb = li.querySelector("input");
    cb.checked = order >= 0; cb.dataset.id = p.id;
    li.querySelector(".order").textContent = order >= 0 ? order + 1 : "";
    li.querySelector(".n").textContent = p.name;
    cb.onchange = () => {
      S.patrolSel = cb.checked ? [...S.patrolSel, p.id] : S.patrolSel.filter((id) => id !== p.id);
      renderPoints(); draw();
    };
    pl.appendChild(li);
  }
}

// ---------------- 导航 / 巡逻 ----------------
async function ensurePatrolMode() {  // 与原前端导航页一致：先进入 patrol 模式
  const control = S.robotStatus.split("+")[1];
  if (control !== "patrol") {
    const r = await call(SRV.modeSet, { action: "patrol" });
    if (r.message !== "ok") throw new Error("进入导航模式失败（需要先在'地图'里使用一张地图进入定位）");
  }
  if (S.task && (S.task.status === 1 || S.task.status === 4)) {  // 有任务在跑时新目标会被忽略，先取消
    await call(SRV.patrol, { cmd: "cancel" });
    await sleep(500);
  }
}
function poseStamped(x, y, yaw) {
  return { header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
    pose: { position: { x, y, z: 0 }, orientation: yawToQuat(yaw) } };
}
async function navigateTo(p) {
  await ensurePatrolMode();
  publish("/nav_to_pose", "geometry_msgs/msg/PoseStamped", poseStamped(p.x, p.y, p.yaw));
  toast(`前往"${p.name}"`);
}
async function startPatrol() {
  const pts = S.patrolSel.map((id) => S.points.find((p) => p.id === id)).filter(Boolean);
  if (pts.length < 2) throw new Error("至少勾选两个点位");
  await ensurePatrolMode();
  publish("/patrol_path", "nav_msgs/msg/Path", {
    header: { frame_id: "map", stamp: { sec: 0, nanosec: 0 } },
    poses: pts.map((p) => poseStamped(p.x, p.y, p.yaw)),
  });
  toast(`开始巡逻：${pts.map((p) => p.name).join(" → ")}`);
}
async function taskControl(cmd) {
  const r = await call(SRV.patrol, { cmd });
  if (!r.success) throw new Error(`${cmd} 失败：${r.message}`);
}

// ---------------- 语义地图 ----------------
function renderSemantic(res) {
  const box = $("semResult"); box.innerHTML = "";
  const head = document.createElement("div");
  if (res.found) {
    head.innerHTML = '<span class="best"></span>';
    head.firstChild.textContent = `→ ${res.best.name}（分数 ${res.best.score.toFixed(3)}）${res.navigating ? "，出发！" : ""}`;
    S.semHitId = res.best.id;
    centerOn(res.best.x, res.best.y);
  } else {
    head.textContent = "没有匹配的点位，机器人不会动。";
    S.semHitId = null;
  }
  box.appendChild(head);
  if (res.candidates && res.candidates.length) {
    const table = document.createElement("table");
    for (const c of res.candidates) {
      const tr = table.insertRow();
      tr.insertCell().textContent = c.name;
      tr.insertCell().textContent = c.score.toFixed(3);
      tr.insertCell().textContent = c.description;
    }
    box.appendChild(table);
  }
  draw();
}
async function semantic(go) {
  const q = $("semQuery").value.trim();
  if (!q) throw new Error("先输入一句话");
  $("semResult").textContent = "搜索中……（第一次要加载模型，可能要等一会儿）";  // 先清掉上一次的结果
  S.semHitId = null;
  try {
    if (go) await ensurePatrolMode();
  } catch (e) { $("semResult").textContent = `失败：${e.message}`; throw e; }
  const data = JSON.stringify({ text: q, source: "web" });  // source 只用于调试记录
  const r = await call(go ? SRV.semGo : SRV.semSearch, { data }, 180000);
  if (!r.success) { $("semResult").textContent = `失败：${r.message}`; throw new Error(r.message); }
  renderSemantic(JSON.parse(r.message));
}

// 调试记录：语义地图节点每处理一次请求就在 /semantic_map/debug 发一条，连上时先用 /semantic_map/history 补齐之前的
const semKey = (r) => `${r.time}|${r.entry}|${r.raw}`;
function addSemLog(records) {
  const seen = new Set(S.semLog.map(semKey));
  for (const r of records) if (!seen.has(semKey(r))) { S.semLog.push(r); seen.add(semKey(r)); }
  S.semLog.sort((a, b) => (a.time < b.time ? -1 : 1));
  if (S.semLog.length > 200) S.semLog = S.semLog.slice(-200);
  renderSemLog();
}
async function loadSemLog() {
  const r = await call(SRV.semHistory);
  if (r.success) addSemLog(JSON.parse(r.message || "[]"));
}
function rawNode(visible) {  // 服务端已把看不见的字符写成 ␠ \n ⟨U+200B⟩，这里给它们加底色
  const span = document.createElement("span");
  for (const part of visible.split(/(␠|\\[nrt]|⟨U\+[0-9A-F]+⟩)/)) {
    if (!part) continue;
    const t = document.createElement("span");
    t.textContent = part;
    if (/^(␠|\\[nrt]|⟨U\+[0-9A-F]+⟩)$/.test(part)) t.className = "inv";
    span.appendChild(t);
  }
  return span;
}
const SOURCE_NAME = { search: "服务 search", go: "服务 go", topic: "话题 text_in", web: "网页" };
function renderSemLog() {
  const ul = $("semDebug"); ul.innerHTML = "";
  const onlyIssues = $("semDebugIssues").checked;
  for (const r of S.semLog.slice().reverse()) {
    const bad = !r.ok || (r.issues && r.issues.length) || !r.found;
    if (onlyIssues && !bad) continue;
    const li = document.createElement("li");
    const meta = document.createElement("div"); meta.className = "meta";
    meta.textContent = `${r.time.slice(11, 23)} · 来源 ${SOURCE_NAME[r.source] || r.source}` +
      `${r.source !== r.entry ? `（经 ${SOURCE_NAME[r.entry] || r.entry}）` : ""} · ${r.length} 字 · ${r.elapsed_ms} ms`;
    const raw = document.createElement("div"); raw.className = "raw";
    raw.append("原文：「", rawNode(r.raw_visible), "」");
    li.append(meta, raw);
    for (const i of r.issues || []) {
      const d = document.createElement("div"); d.className = "issue"; d.textContent = `⚠ ${i}`; li.appendChild(d);
    }
    const res = document.createElement("div"); res.className = "res";
    const scores = (r.candidates || []).map((c) => `${c.name} ${c.score.toFixed(3)}`).join("，");
    if (!r.ok) { res.classList.add("bad"); res.textContent = `✗ 出错：${r.error}`; }
    else if (r.found) {
      const b = document.createElement("span"); b.className = "best"; b.textContent = `→ ${r.best}`;
      res.append(b, `${r.navigating ? "（已发导航）" : ""}  得分：${scores}（阈值 ${r.threshold}）`);
    } else {
      res.classList.add("bad");
      res.textContent = `✗ 没找到：${scores || "没有点位"}${scores ? `，最高分低于阈值 ${r.threshold}` : ""}`;
    }
    li.appendChild(res);
    ul.appendChild(li);
  }
  if (!ul.children.length) ul.innerHTML = '<li class="meta">还没有记录</li>';
}
function sendAsrTest() {
  const text = $("semAsrText").value;
  if (!text.trim()) throw new Error("先输入一句话");
  publish("/semantic_map/text_in", "std_msgs/msg/String", { data: text });  // 原样发送，不 trim，方便测试首尾空白
  toast("已发到 /semantic_map/text_in，结果看下面的调试记录");
}

// ---------------- 禁行线 / 橡皮擦 ----------------
function renderNogo() {
  const ul = $("nogoList"); ul.innerHTML = "";
  S.nogo.forEach((l, i) => {
    const li = document.createElement("li");
    li.innerHTML = '<span class="s"></span><button class="danger">删除</button>';
    li.firstChild.textContent = `${i + 1}. (${round(l.x1, 2)}, ${round(l.y1, 2)}) → (${round(l.x2, 2)}, ${round(l.y2, 2)})`;
    li.lastChild.onclick = () => { S.nogo.splice(i, 1); renderNogo(); draw(); };
    ul.appendChild(li);
  });
  if (!S.nogo.length) ul.innerHTML = '<li class="s">没有禁行线</li>';
}
async function saveNogo() {
  if (!S.mapId) throw new Error("先选择地图");
  const lines = S.nogo.map((l) => ({ start: { x: l.x1, y: l.y1, z: 0 }, end: { x: l.x2, y: l.y2, z: 0 } }));
  const r = await call(SRV.setForbidden, { map_id: S.mapId, frame_id: "map", lines });
  if (!r.success) throw new Error(`保存禁行线失败：${r.message}`);
  // 与原前端一致：再画到导航用的禁行地图上（需要定位模式下已加载地图）
  const d = await call(SRV.drawForbidden, { frame_id: "map", type: "line", map_id: S.mapId, data: lines, rectangle_array: [] })
    .catch((e) => ({ success: false, message: e.message }));
  toast(d.success ? `已保存 ${lines.length} 条禁行线` : `已存入数据库，但导航地图未更新：${d.message}`);
  await loadNogo();
}
async function applyEraser() {
  if (!S.eraser.length) throw new Error("先在地图上涂抹");
  const gray = +$("eraserGray").value;
  const r = await call(SRV.mapEditor, { frame_id: "map", type: "eraser", map_id: S.mapId, data: [],
    rectangle_array: S.eraser.map((e) => ({ center_point: { x: e.x, y: e.y, z: 0 }, side_length: e.size, grayscale: gray })) });
  if (!r.success) throw new Error(`橡皮擦失败：${r.message}`);
  S.eraser = [];
  await reloadMapImage(false);
  toast("地图已更新");
}

// ---------------- 重定位 / 模式 ----------------
function relocate(x, y, yaw) {
  const pose = poseStamped(x, y, yaw);
  const cov = new Array(36).fill(0); cov[0] = 0.25; cov[7] = 0.25; cov[35] = 0.06853891945200942;
  publish("/initialpose", "geometry_msgs/msg/PoseWithCovarianceStamped",
    { header: pose.header, pose: { pose: pose.pose, covariance: cov } });
  publish("/start_init_pose", "geometry_msgs/msg/PoseStamped", pose);
  toast("已发送重定位");
}
async function setMode(action) {
  const r = await call(SRV.modeSet, { action });
  toast(r.message === "ok" ? `已切换到${MODE_NAME[action]}` : `切换失败：${r.message}`);
  if (action === "mapping" && r.message === "ok") { S.map = null; startPreview(); }
  if (action === "continue_mapping" && r.message === "ok") startPreview();
}

// ---------------- 画布 ----------------
const canvas = $("map"), ctx = canvas.getContext("2d");
const css = (name) => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const w2s = (x, y) => [((x - S.map.ox) / S.map.resolution) * S.view.scale + S.view.ox,
  (S.map.height - (y - S.map.oy) / S.map.resolution) * S.view.scale + S.view.oy];
const s2w = (sx, sy) => [S.map.ox + ((sx - S.view.ox) / S.view.scale) * S.map.resolution,
  S.map.oy + (S.map.height - (sy - S.view.oy) / S.view.scale) * S.map.resolution];
window.w2s = w2s;

function resize() {
  const r = canvas.getBoundingClientRect(), dpr = window.devicePixelRatio || 1;
  canvas.width = r.width * dpr; canvas.height = r.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  draw();
}
function fitView() {
  if (!S.map) return;
  const r = canvas.getBoundingClientRect();
  S.view.scale = Math.min(r.width / S.map.width, r.height / S.map.height) * 0.94;
  S.view.ox = (r.width - S.map.width * S.view.scale) / 2;
  S.view.oy = (r.height - S.map.height * S.view.scale) / 2;
  draw();
}
function centerOn(x, y) {
  if (!S.map) return;
  const r = canvas.getBoundingClientRect(), [sx, sy] = w2s(x, y);
  S.view.ox += r.width / 2 - sx; S.view.oy += r.height / 2 - sy; draw();
}
function arrow(x, y, yaw, color, label, big) {
  const [sx, sy] = w2s(x, y), len = big ? 26 : 20;
  const ex = sx + Math.cos(yaw) * len, ey = sy - Math.sin(yaw) * len;
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = 2.5;
  ctx.beginPath(); ctx.moveTo(sx, sy); ctx.lineTo(ex, ey); ctx.stroke();
  const a = Math.atan2(ey - sy, ex - sx);
  ctx.beginPath(); ctx.moveTo(ex, ey);
  ctx.lineTo(ex - 8 * Math.cos(a - 0.45), ey - 8 * Math.sin(a - 0.45));
  ctx.lineTo(ex - 8 * Math.cos(a + 0.45), ey - 8 * Math.sin(a + 0.45)); ctx.fill();
  ctx.beginPath(); ctx.arc(sx, sy, big ? 7 : 5.5, 0, Math.PI * 2);
  ctx.fillStyle = css("--panel"); ctx.fill(); ctx.lineWidth = 3; ctx.stroke();
  if (label) {
    ctx.font = "600 12px system-ui, sans-serif";
    const w = ctx.measureText(label).width;
    ctx.fillStyle = css("--overlay"); ctx.fillRect(sx + 9, sy + 5, w + 8, 18);
    ctx.fillStyle = color; ctx.fillText(label, sx + 13, sy + 18);
  }
}
function draw() {
  const r = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);
  $("mapEmpty").hidden = !!S.map;
  if (!S.map) return;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(S.map.img, S.view.ox, S.view.oy, S.map.width * S.view.scale, S.map.height * S.view.scale);
  if (!S.mappingPreview) {
    // 禁行线
    ctx.strokeStyle = css("--nogo"); ctx.lineWidth = 3; ctx.setLineDash([8, 5]);
    for (const l of S.nogo) {
      const [a, b] = w2s(l.x1, l.y1), [c, d] = w2s(l.x2, l.y2);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(c, d); ctx.stroke();
    }
    if (S.nogoStart && S.hover) {
      const [a, b] = w2s(...S.nogoStart);
      ctx.beginPath(); ctx.moveTo(a, b); ctx.lineTo(...S.hover); ctx.stroke();
    }
    ctx.setLineDash([]);
    // 橡皮擦涂抹预览
    ctx.fillStyle = "rgba(255, 200, 0, .45)";
    for (const e of S.eraser) {
      const [a, b] = w2s(e.x - e.size / 2, e.y + e.size / 2), s = (e.size / S.map.resolution) * S.view.scale;
      ctx.fillRect(a, b, s, s);
    }
    // 巡逻路线
    if (S.tab === "nav" && S.patrolSel.length > 1) {
      const pts = S.patrolSel.map((id) => S.points.find((p) => p.id === id)).filter(Boolean);
      ctx.strokeStyle = css("--accent"); ctx.lineWidth = 2; ctx.setLineDash([4, 4]);
      ctx.beginPath();
      pts.concat(pts[0]).forEach((p, i) => { const [a, b] = w2s(p.x, p.y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
      ctx.stroke(); ctx.setLineDash([]);
    }
    // 点位
    for (const p of S.points) {
      const color = p.id === S.selectedId ? css("--point-sel") : p.id === S.semHitId ? css("--hit") : css("--point");
      arrow(p.x, p.y, p.yaw, color, p.name, p.id === S.selectedId || p.id === S.semHitId);
    }
    if (S.pending) arrow(S.pending.x, S.pending.y, S.pending.yaw, css("--point-sel"), S.pending.name || "新点位", true);
  }
  if (S.drag && S.drag.kind === "pose") {
    const [x, y] = S.drag.start;
    arrow(x, y, dragYaw(S.drag), S.tool === "relocate" ? css("--robot") : css("--point-sel"), null, true);
  }
  // 机器人
  if (S.robot) {
    const [sx, sy] = w2s(S.robot.x, S.robot.y), a = -S.robot.yaw;
    ctx.save(); ctx.translate(sx, sy); ctx.rotate(a);
    ctx.fillStyle = css("--robot"); ctx.strokeStyle = css("--panel"); ctx.lineWidth = 2;
    ctx.beginPath(); ctx.moveTo(14, 0); ctx.lineTo(-9, 9); ctx.lineTo(-5, 0); ctx.lineTo(-9, -9); ctx.closePath();
    ctx.fill(); ctx.stroke(); ctx.restore();
  }
}
function evPos(e) { const r = canvas.getBoundingClientRect(); return [e.clientX - r.left, e.clientY - r.top]; }
function hitPoint(sx, sy) {
  let best = null, bd = 12;
  for (const p of S.points) {
    const [px, py] = w2s(p.x, p.y), d = Math.hypot(px - sx, py - sy);
    if (d < bd) { best = p; bd = d; }
  }
  return best;
}
function dragYaw(d) {
  const dx = d.cur[0] - d.startScreen[0], dy = d.cur[1] - d.startScreen[1];
  return Math.hypot(dx, dy) < 8 ? d.defaultYaw : Math.atan2(-dy, dx);
}
function paintEraser(sx, sy) {
  const [x, y] = s2w(sx, sy), size = +$("eraserSize").value;
  const last = S.eraser[S.eraser.length - 1];
  if (!last || Math.hypot(last.x - x, last.y - y) > size / 2) S.eraser.push({ x: round(x), y: round(y), size });
}
canvas.addEventListener("contextmenu", (e) => e.preventDefault());
canvas.addEventListener("pointerdown", (e) => {
  if (!S.map) return;
  canvas.setPointerCapture(e.pointerId);
  const [sx, sy] = evPos(e);
  const pan = () => { S.drag = { kind: "pan", last: [sx, sy] }; };
  if (e.button !== 0 || S.mappingPreview) return pan();
  const [x, y] = s2w(sx, sy);
  if (S.tool === "nogo") {
    if (!S.nogoStart) S.nogoStart = [round(x), round(y)];
    else { S.nogo.push({ x1: S.nogoStart[0], y1: S.nogoStart[1], x2: round(x), y2: round(y) }); S.nogoStart = null; renderNogo(); }
    draw(); return;
  }
  if (S.tool === "eraser") { S.drag = { kind: "erase" }; paintEraser(sx, sy); draw(); return; }
  if (S.tool === "point") {
    const hit = hitPoint(sx, sy);
    if (hit) { selectPoint(hit.id); S.drag = { kind: "move", p: hit, last: [sx, sy], moved: false }; return; }
  }
  if (S.tool === "point" || S.tool === "relocate") {
    const def = S.tool === "relocate" && S.robot ? S.robot.yaw : 0;
    S.drag = { kind: "pose", start: [x, y], startScreen: [sx, sy], cur: [sx, sy], defaultYaw: def };
    draw(); return;
  }
  pan();
});
canvas.addEventListener("pointermove", (e) => {
  if (!S.map) return;
  const [sx, sy] = evPos(e), [x, y] = s2w(sx, sy);
  S.hover = [sx, sy];
  $("mapCoord").textContent = `x=${x.toFixed(2)}  y=${y.toFixed(2)} m`;
  const d = S.drag;
  if (!d) { if (S.nogoStart) draw(); return; }
  if (d.kind === "pan") { S.view.ox += sx - d.last[0]; S.view.oy += sy - d.last[1]; d.last = [sx, sy]; }
  else if (d.kind === "pose") d.cur = [sx, sy];
  else if (d.kind === "erase") paintEraser(sx, sy);
  else if (d.kind === "move") {
    if (!d.moved && Math.hypot(sx - d.last[0], sy - d.last[1]) < 4) return;
    d.moved = true; d.p.x = x; d.p.y = y; fillPointForm();
  }
  draw();
});
canvas.addEventListener("pointerup", () => {
  const d = S.drag; S.drag = null;
  if (!d) return;
  if (d.kind === "pose") {
    const [x, y] = d.start, yaw = dragYaw(d);
    if (S.tool === "relocate") { relocate(round(x), round(y), round(yaw, 4)); setTool(null); }
    else newPoint(x, y, yaw);
  }
  if (d.kind === "move" && d.moved) guard(savePoint);
  draw();
});
canvas.addEventListener("wheel", (e) => {
  if (!S.map) return;
  e.preventDefault();
  const [sx, sy] = evPos(e), k = Math.exp(-e.deltaY * 0.0015);
  const ns = Math.min(Math.max(S.view.scale * k, 0.2), 60);
  S.view.ox = sx - ((sx - S.view.ox) * ns) / S.view.scale;
  S.view.oy = sy - ((sy - S.view.oy) * ns) / S.view.scale;
  S.view.scale = ns; draw();
}, { passive: false });

// ---------------- 标签页 / 工具 ----------------
const HINTS = {
  point: "按住拖动新建点位（拖的方向 = 到达后的朝向）；点圆点编辑，拖圆点移动。右键拖动平移，滚轮缩放。",
  nogo: "依次点击起点和终点画一条禁行线。",
  eraser: "点击或拖动涂抹，然后点右边的“应用橡皮擦”。",
  relocate: "在机器人实际所在位置按住，朝它面对的方向拖一下再松开。",
};
function setTool(tool) {
  S.tool = tool; S.nogoStart = null;
  $("btnNogoDraw").classList.toggle("active", tool === "nogo");
  $("btnEraser").classList.toggle("active", tool === "eraser");
  $("btnRelocate").classList.toggle("active", tool === "relocate");
  $("mapHint").textContent = S.mappingPreview ? "建图中：左边是实时地图" : (HINTS[tool] || "拖动平移，滚轮缩放。");
  draw();
}
function setTab(tab) {
  S.tab = tab;
  document.querySelectorAll("#tabs button").forEach((b) => b.classList.toggle("active", b.dataset.tab === tab));
  document.querySelectorAll(".tab").forEach((s) => { s.hidden = s.id !== `tab-${tab}`; });
  setTool(tab === "points" ? "point" : null);
}
function renderAll() { renderPoints(); fillPointForm(); renderNogo(); draw(); }

// ---------------- 事件绑定 ----------------
document.querySelectorAll("#tabs button").forEach((b) => { b.onclick = () => setTab(b.dataset.tab); });
$("connForm").addEventListener("submit", (e) => { e.preventDefault(); connect($("host").value.trim() || "localhost"); });
$("btnFit").onclick = fitView;
$("btnCenterRobot").onclick = () => S.robot && centerOn(S.robot.x, S.robot.y);
$("btnMapsRefresh").onclick = () => guard(loadMaps);
$("btnMapStart").onclick = (e) => confirmThen(e.currentTarget, "再点一次：清空并新建", () => guard(startMapping));
$("btnMapContinue").onclick = () => guard(continueMapping);
$("btnMapCancel").onclick = () => guard(cancelMapping);
$("btnMapSave").onclick = () => guard(saveMapping);
$("pointForm").addEventListener("submit", (e) => { e.preventDefault(); guard(savePoint); });
for (const id of ["pName", "pDesc", "pX", "pY", "pYaw"]) {
  $(id).addEventListener("input", () => { const p = currentPoint(); if (p) { readPointForm(p); $("pWarn").textContent = cellWarning(p.x, p.y); draw(); } });
}
$("btnPointCancel").onclick = () => { S.pending = null; S.selectedId = null; guard(loadPoints); };
$("btnPointDelete").onclick = (e) => confirmThen(e.target, "再点一次删除", () => guard(() => deletePoint(currentPoint())));
$("btnPointGo").onclick = () => guard(() => navigateTo(currentPoint()));
$("btnPointHere").onclick = () => {
  if (!S.robot) { toast("还没有收到机器人位置（/base_link_pose）"); return; }
  setTab("points"); newPoint(S.robot.x, S.robot.y, S.robot.yaw);
};
$("btnSemSearch").onclick = () => guard(() => semantic(false));
$("btnSemGo").onclick = () => guard(() => semantic(true));
$("semQuery").addEventListener("keydown", (e) => { if (e.key === "Enter") guard(() => semantic(false)); });
$("btnSemAsr").onclick = () => guard(sendAsrTest);
$("btnSemDebugRefresh").onclick = () => guard(loadSemLog);
$("semDebugIssues").onchange = renderSemLog;
$("btnSemRebuild").onclick = () => guard(async () => {
  const r = await call(SRV.semRebuild, {}, 180000);
  if (!r.success) throw new Error(r.message);
  toast(`语义库已重建：${JSON.parse(r.message).count} 个点位`);
});
$("btnNavGo").onclick = () => guard(() => navigateTo(S.points.find((p) => String(p.id) === $("navPoint").value)));
$("btnPause").onclick = () => guard(() => taskControl("pause"));
$("btnResume").onclick = () => guard(() => taskControl("resume"));
$("btnCancel").onclick = () => guard(() => taskControl("cancel"));
$("btnPatrolStart").onclick = () => guard(startPatrol);
$("btnNogoDraw").onclick = () => setTool(S.tool === "nogo" ? null : "nogo");
$("btnNogoSave").onclick = () => guard(saveNogo);
$("btnEraser").onclick = () => setTool(S.tool === "eraser" ? null : "eraser");
$("btnEraserApply").onclick = () => guard(applyEraser);
$("btnEraserClear").onclick = () => { S.eraser = []; draw(); };
document.querySelectorAll(".modes button").forEach((b) => {
  b.onclick = () => (b.dataset.mode === "mapping"  // 新建图会清掉当前实时地图，要确认
    ? confirmThen(b, "再点一次：清空并新建", () => guard(() => setMode("mapping")))
    : guard(() => setMode(b.dataset.mode)));
});
$("btnRelocate").onclick = () => setTool(S.tool === "relocate" ? null : "relocate");
$("btnRelocateZero").onclick = () => relocate(0, 0, 0);
window.addEventListener("resize", resize);

// 启动：地址默认用上次连接的，或页面所在主机（网页和 rosbridge 在同一台机器上时直接可用）
let savedHost = "";
try { savedHost = localStorage.getItem("rosIP") || ""; } catch (e) { /* 忽略 */ }
const params = new URLSearchParams(location.search);
$("host").value = params.get("host") || savedHost || location.hostname || "localhost";
resize(); setTab("maps");
if (params.get("host") || location.hostname) connect($("host").value);
