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
  costmap: null,      // {canvas, width, height, resolution, ox, oy}：导航时的 local costmap
  globalPlan: null, localPlan: null, arc: null,  // 导航时的路径，map 系 [[x, y], ...]
  loc: null, locLevel: "",  // /lightning/loc_status 最近一条（JSON）和当前级别 ok / warn / bad
  goal: null, navStart: 0, cmdV: null, odomV: null,  // 当前导航目标 {x, y, yaw, name}、本次任务开始时间、速度
  arrivals: [], settle: null,  // 到点误差记录（新的在前）；正在等站稳的那一条
  nav2Log: [], failure: null,  // Nav2 节点的告警日志；最近一次导航失败 {time, since}
  seen: {}, bw: null,  // 每个话题最后收到的时间；带宽统计
  health: { nodes: null, pubs: {}, busy: false },
  radius: null, radiusBusy: false,  // 两张 costmap 的避障半径 {节点: {radius, inflation} | null}
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
function call([name, type], args = {}, timeoutMs = 60000, quiet = false) {  // quiet：定时轮询用，不写操作记录
  return new Promise((resolve, reject) => {
    if (!S.connected) { reject(new Error("未连接机器人")); return; }
    const srv = new ROSLIB.Service({ ros: S.ros, name, serviceType: type });
    const timer = setTimeout(() => reject(new Error(`${name} 超时`)), timeoutMs);
    srv.callService(new ROSLIB.ServiceRequest(args), (res) => {
      clearTimeout(timer);
      if (!quiet) log(`${name} ${short(args)} → ${short(res)}`, "ok");
      resolve(res);
    }, (err) => {
      clearTimeout(timer);
      if (!quiet) log(`${name} ${short(args)} ✗ ${err}`, "err");
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
  S.bw = { since: Date.now(), total: 0, topics: {}, hist: [] };
  S.seen = {};
  if (S.ros.socket && S.ros.socket.addEventListener) S.ros.socket.addEventListener("message", (ev) => meter(ev.data));
  log(`连接 ${url}`);
}
function setConn(on) {
  S.connected = on;
  $("stConn").classList.toggle("on", on);
  $("stConn").lastElementChild.textContent = on ? "已连接" : "未连接";
  if (!on) { updateSubs(); S.loc = null; renderLoc(); }
  S.radius = null; renderRadius();
}
async function onConnected() {
  setConn(true);
  toast("已连接");
  subscribe("/base_link_pose", "geometry_msgs/msg/PoseStamped", (m) => {
    S.robot = { x: m.pose.position.x, y: m.pose.position.y, yaw: quatToYaw(m.pose.orientation) };
    checkSettled();
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
    updateSubs();
    renderLoc();
  });
  subscribe("/task_status", "aid_robot_msgs/msg/AidTaskStatus", (m) => {
    const prev = S.task ? S.task.status : null;
    S.task = m;
    if (m.status !== prev) onTaskStatus(prev, m);
    const text = `${TASK_TYPE[m.task_type] || m.task_type} · ${TASK_STATUS[m.status] || m.status}`;
    $("stTask").textContent = m.status === 0 ? "空闲" : text;
    $("navTask").textContent = $("stTask").textContent;
  });
  subscribe("/lightning/loc_status", "std_msgs/msg/String", onLocStatus, { queue_length: 5 });
  // 导航目标：网页、语义地图、ASR 发的单点目标都走 /nav_to_pose。一直订阅（很小），否则刚切导航模式时发的目标会漏掉
  subscribe("/nav_to_pose", "geometry_msgs/msg/PoseStamped", (m) => {
    const g = { x: m.pose.position.x, y: m.pose.position.y, yaw: quatToYaw(m.pose.orientation) };
    const p = S.points.find((q) => Math.hypot(q.x - g.x, q.y - g.y) < 0.02);
    S.goal = { ...g, name: p ? p.name : `(${round(g.x, 2)}, ${round(g.y, 2)})` };
    renderNavInfo();
  });
  subscribe("/semantic_map/debug", "std_msgs/msg/String", (m) => addSemLog([JSON.parse(m.data)]));
  updateSubs();
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

// ---------------- 避障半径（接口文档 3.4） ----------------
// 两张 costmap 的 robot_radius 支持运行时改（Nav2 动态参数），用节点自带的 get_parameters / set_parameters 服务。
// Nav2 重启后恢复 nav2_params.yaml 的值，所以导航页打开时每 5 秒读一次。
// 膨胀半径只读不改（2026-10-25）：原来按"半径 + 0.20"一起改，设 0.40 时膨胀变 0.60，梯度带只剩 0.2 m，
// 离障碍 0.6 m 外代价全为 0，规划器不再把路径推向通道中线，路径贴墙、走着走着就 collision ahead。膨胀半径以配置文件为准。
const RADIUS = { min: 0.40, max: 0.55, minBand: 0.10,  // 膨胀半径至少比半径大 0.10，否则不让改
  nodes: ["/local_costmap/local_costmap", "/global_costmap/global_costmap"],
  names: ["robot_radius", "inflation_layer.inflation_radius"] };
const PARAM_DOUBLE = 3;  // rcl_interfaces/msg/ParameterType
function radiusError(r) {
  if (!Number.isFinite(r)) return "请输入数字";
  if (r < RADIUS.min - 1e-9 || r > RADIUS.max + 1e-9) return `只允许 ${RADIUS.min.toFixed(2)}～${RADIUS.max.toFixed(2)} 米`;
  return "";
}
async function loadRadius(quiet = true) {
  if (S.radiusBusy || !S.connected) return;
  if (!inLoc()) {  // 不在定位模式时 Nav2 一定没跑，不去调不存在的服务
    S.radius = Object.fromEntries(RADIUS.nodes.map((n) => [n, null]));
    renderRadius();
    return;
  }
  S.radiusBusy = true;
  try {
    const out = {};
    for (const node of RADIUS.nodes) {
      try {
        const r = await call([`${node}/get_parameters`, "rcl_interfaces/srv/GetParameters"], { names: RADIUS.names }, 5000, quiet);
        const [rad, infl] = r.values.map((v) => (v.type === PARAM_DOUBLE ? v.double_value : null));
        out[node] = rad === null ? null : { radius: rad, inflation: infl };
      } catch (e) { out[node] = null; }  // Nav2 没在运行时服务不存在
    }
    S.radius = out;
  } finally { S.radiusBusy = false; }
  renderRadius();
}
function renderRadius() {
  const el = $("radiusNow"), vals = S.radius ? RADIUS.nodes.map((n) => S.radius[n]) : [];
  let text = "--", bad = false;
  if (!S.connected) text = "--";
  else if (!S.radius) text = "读取中…";
  else if (vals.every((v) => !v)) { text = "读不到（Nav2 没在运行，需要定位模式）"; bad = true; }
  else if (vals.some((v) => !v) || vals[0].radius !== vals[1].radius || vals[0].inflation !== vals[1].inflation) {
    const f = (v) => (v ? `${v.radius.toFixed(2)}（膨胀 ${v.inflation === null ? "?" : v.inflation.toFixed(2)}）` : "读不到");
    text = `local ${f(vals[0])} / global ${f(vals[1])}：两张图不一致，请重新应用一次`; bad = true;
  } else {
    const { radius: r, inflation: i } = vals[0];
    text = `${r.toFixed(2)} 米（膨胀 ${i === null ? "?" : i.toFixed(2)} 米，通道净宽约需 ${(2 * r + 0.15).toFixed(2)}～${(2 * r + 0.25).toFixed(2)} 米）`;
    if ($("radiusInput").value === "" && document.activeElement !== $("radiusInput")) $("radiusInput").value = r.toFixed(2);
  }
  el.textContent = text;
  el.classList.toggle("bad", bad);
  checkRadiusInput();
}
function checkRadiusInput() {
  const s = $("radiusInput").value.trim(), err = s === "" ? "" : radiusError(Number(s));
  $("radiusWarn").textContent = err;
  $("btnRadiusApply").disabled = !S.connected || s === "" || !!err;
}
async function applyRadius() {
  const r = Number($("radiusInput").value);
  const err = radiusError(r);
  if (err) throw new Error(`避障半径${err}`);
  const v = round(r, 2);
  await loadRadius(false);
  for (const node of RADIUS.nodes) {
    const cur = S.radius && S.radius[node];
    if (!cur || cur.inflation === null) throw new Error(`${node} 读不到当前膨胀半径，没有修改`);
    if (cur.inflation < v + RADIUS.minBand - 1e-9) {
      throw new Error(`${node} 膨胀半径 ${cur.inflation.toFixed(2)} 米，半径最大只能设到 ${(cur.inflation - RADIUS.minBand).toFixed(2)} 米`);
    }
  }
  for (const node of RADIUS.nodes) {
    const res = await call([`${node}/set_parameters`, "rcl_interfaces/srv/SetParameters"], { parameters: [
      { name: "robot_radius", value: { type: PARAM_DOUBLE, double_value: v } },
    ] }, 10000);
    const bad = res.results.find((x) => !x.successful);
    if (bad) throw new Error(`${node} 设置失败：${bad.reason || "未知原因"}`);
  }
  toast(`避障半径已设为 ${v.toFixed(2)} 米`);
  await loadRadius(false);
}

// ---------------- 按条件订阅 ----------------
// want() 为真时订阅，为假时退订并调 off() 清掉数据。模式、连接、勾选框、标签页变化时调 updateSubs()。
// local costmap 是 map 系的滚动窗口（6×6 m、0.05 m），值已被 nav2 换成 0~100：100 致命、99 内切带、1~98 膨胀、-1 未知。
// 路径：/plan 是 SmacPlanner2D 的全局路径（按 global costmap 规划，含 Smac 内置平滑；只在重规划时才发新的——2026-10-26 前 BT 只在
// 路径失效时重规划，之后是每 2 s 或失效时——所以 /plan 可能是机器人还在后面时规划的，起点落在身后；控制器直接跟它，剩余距离也用它算，导航时一直订阅）；
// 注意画在它下面的是 local costmap（实时），不是规划器当时看到的 global costmap，两者可以对不上；
// /received_global_plan 是 RPP 实际在跟的那一段（裁到 costmap 范围内）；/lookahead_collision_arc 是 RPP 按当前速度预测的未来轨迹。
const inNav = () => S.robotStatus.split("+")[1] === "patrol";
const inLoc = () => S.robotStatus.split("+")[0] === "localization";
const healthOpen = () => S.tab === "health";
const SUBS = [
  { topic: "/local_costmap/costmap", type: "nav_msgs/msg/OccupancyGrid", want: () => inNav() && $("navCostmap").checked,
    on: (m) => { S.costmap = costmapImage(m); draw(); }, off: () => { S.costmap = null; },
    opts: { throttle_rate: 500, queue_length: 1, compression: "cbor" } },  // CBOR 下 int8[] 直接是 Int8Array
  { topic: "/plan", type: "nav_msgs/msg/Path", want: inNav,
    on: (m) => { S.globalPlan = pathPoints(m); renderNavInfo(); draw(); }, off: () => { S.globalPlan = null; },
    opts: { throttle_rate: 500, queue_length: 1, compression: "cbor" } },
  { topic: "/received_global_plan", type: "nav_msgs/msg/Path", want: () => inNav() && $("navPaths").checked,
    on: (m) => { S.localPlan = pathPoints(m); draw(); }, off: () => { S.localPlan = null; },
    opts: { throttle_rate: 500, queue_length: 1, compression: "cbor" } },
  { topic: "/lookahead_collision_arc", type: "nav_msgs/msg/Path", want: () => inNav() && $("navPaths").checked,
    on: (m) => { S.arc = pathPoints(m); draw(); }, off: () => { S.arc = null; },
    opts: { throttle_rate: 200, queue_length: 1, compression: "cbor" } },
  // collision_monitor 没开时桥接直接吃 /cmd_vel（机器人上没有 /cmd_vel_safe）；实际速度看腿式里程计
  { topic: "/cmd_vel", type: "geometry_msgs/msg/Twist", want: inNav,
    on: (m) => { S.cmdV = { vx: m.linear.x, wz: m.angular.z, t: Date.now() }; }, off: () => { S.cmdV = null; },
    opts: { throttle_rate: 200, queue_length: 1 } },
  { topic: "/odom", type: "nav_msgs/msg/Odometry", want: inNav,
    on: (m) => { S.odomV = { vx: m.twist.twist.linear.x, vy: m.twist.twist.linear.y, wz: m.twist.twist.angular.z, t: Date.now() }; },
    off: () => { S.odomV = null; }, opts: { throttle_rate: 200, queue_length: 1, compression: "cbor" } },
  // 日志不能节流（会丢），queue 放大一点；只留 Nav2 相关节点的告警，见 onRosout
  { topic: "/rosout", type: "rcl_interfaces/msg/Log", want: inNav, on: onRosout, opts: { queue_length: 100 } },
  // 状态页：传感器只订阅轻量话题判断"还在不在发"（D435 停发时节点和发布者都还在，只能看数据）
  { topic: "/livox/imu", type: "sensor_msgs/msg/Imu", want: healthOpen, on: () => {}, opts: { throttle_rate: 1000, queue_length: 1 } },
  { topic: "/camera/camera/depth/camera_info", type: "sensor_msgs/msg/CameraInfo", want: healthOpen, on: () => {},
    opts: { throttle_rate: 1000, queue_length: 1, compression: "cbor" } },
];
function updateSubs() {
  for (const o of SUBS) {
    const want = S.connected && o.want(), t = topics[o.topic];
    if (want && (!t || t.ros !== S.ros)) {  // 重连后旧连接上的订阅已失效，要重订
      subscribe(o.topic, o.type, o.on, o.opts);
    } else if (!want && t) {
      unsubscribe(o.topic);
      if (o.off) o.off();
      draw();
    }
  }
}
function pathPoints(path) {  // 统一换到 map 系；RPP 有的调试话题在 base_link 系，按当前机器人位姿换算
  const frame = (path.header.frame_id || "map").replace(/^\//, "");
  const pts = path.poses.map((p) => [p.pose.position.x, p.pose.position.y]);
  if (frame === "map") return pts;
  if (frame !== "base_link" || !S.robot) return null;
  const { x, y, yaw } = S.robot, c = Math.cos(yaw), s = Math.sin(yaw);
  return pts.map(([px, py]) => [x + c * px - s * py, y + s * px + c * py]);
}
function costmapImage(grid) {
  const { width, height, resolution } = grid.info, d = grid.data;
  const c = document.createElement("canvas");
  c.width = width; c.height = height;
  const cctx = c.getContext("2d"), img = cctx.createImageData(width, height), px = img.data;
  for (let j = 0; j < height; j++) {
    for (let i = 0; i < width; i++) {
      const v = d[j * width + i];
      if (v <= 0) continue;  // 空闲、未知不画
      const k = ((height - 1 - j) * width + i) * 4;  // 栅格第 0 行在下（y 最小），图片第 0 行在上
      const [r, g, b, a] = v >= 100 ? [220, 20, 60, 230] : v >= 99 ? [150, 60, 220, 180]
        : [30, 140, 255, 30 + Math.round((v / 98) * 130)];
      px[k] = r; px[k + 1] = g; px[k + 2] = b; px[k + 3] = a;
    }
  }
  cctx.putImageData(img, 0, 0);
  return { canvas: c, width, height, resolution, ox: grid.info.origin.position.x, oy: grid.info.origin.position.y };
}

// ---------------- 定位状态（/lightning/loc_status） ----------------
// lightning 的一致性守护每次激光定位（只在 LIO 关键帧）后发一条 JSON，机器人不动时可能很久不更新，所以不按"多久没收到"判异常。
function onLocStatus(m) {
  try { S.loc = JSON.parse(m.data); } catch (e) { return; }
  S.loc.recvAt = Date.now();
  renderLoc();
}
function locLevel(st) {
  if (st.need_reloc) return ["bad", "需要重定位"];
  if (st.frozen) return ["warn", "守护冻结"];
  if (st.state === "NDT_SUSPECT") return ["warn", "地图匹配可疑"];
  if (st.state === "LIO_SUSPECT") return ["warn", "LIO 可疑"];
  if (st.state !== "GOOD") return ["warn", st.state];
  if (!st.loc_valid) return ["warn", "激光定位无效"];
  return ["ok", "正常"];
}
function renderLoc() {
  const pill = $("stLoc"), text = pill.lastElementChild;
  let level = "off", label = "--", title = "定位一致性守护（/lightning/loc_status）";
  if (S.connected && inLoc()) {
    if (!S.loc) { label = "等待数据"; title += "：还没收到（只在 LIO 关键帧发布，机器人走动后才会有）"; }
    else {
      const st = S.loc;
      [level, label] = locLevel(st);
      const age = Math.round((Date.now() - st.recvAt) / 1000);
      title += `\n状态 ${st.state}${st.frozen ? "（冻结：只跟 LIO，不信地图匹配）" : ""}${st.need_reloc ? "，需要重定位" : ""}` +
        `\n输出相对 LIO 漂移 ${st.drift_m.toFixed(3)} m / ${st.drift_deg.toFixed(2)}°` +
        `\nNDT 残差 ${st.ndt_residual_m.toFixed(3)} m / ${st.ndt_residual_deg.toFixed(2)}°，分值 ${st.score.toFixed(2)}` +
        `\n回退 ${st.rollbacks} 次（上次 ${st.last_rollback_m.toFixed(2)} m），腿式里程计${st.odom_online ? "在线" : "离线"}` +
        `\n${age} 秒前更新`;
    }
  }
  pill.className = `pill lv-${level}`;
  text.textContent = `定位 ${label}`;
  pill.title = title;
  if (level === "bad" && S.locLevel !== "bad") {
    toast("⚠ 定位需要重定位：守护长时间冻结，当前位置可能不准");
    log("定位守护：NEED RELOC", "err");
  } else if (level === "warn" && S.locLevel === "ok") log(`定位守护：${label}`, "err");
  else if (level === "ok" && (S.locLevel === "warn" || S.locLevel === "bad")) log("定位守护：恢复正常", "ok");
  S.locLevel = level;
  $("mapAlert").hidden = level !== "bad";
  $("mapAlert").textContent = level === "bad"
    ? "⚠ 定位需要重定位（守护冻结太久）。先暂停导航，到“模式”里在地图上拖动设置位姿。" : "";
}

// ---------------- 导航信息 / 到点误差 ----------------
const wrapAngle = (a) => Math.atan2(Math.sin(a), Math.cos(a));
function goalError(p, g) {  // 按目标朝向拆成纵向（沿目标朝向，正 = 冲过头）和横向（正 = 偏左）
  const dx = p.x - g.x, dy = p.y - g.y, c = Math.cos(g.yaw), s = Math.sin(g.yaw);
  return { dist: Math.hypot(dx, dy), along: c * dx + s * dy, lateral: -s * dx + c * dy, yaw: wrapAngle(p.yaw - g.yaw) };
}
function onTaskStatus(prev, m) {
  const running = (st) => st === 1 || st === 4;
  if (m.status === 1 && !running(prev)) {  // 新任务开始
    S.navStart = Date.now(); S.failure = null; S.settle = null; renderFail();
  }
  // 单点导航"成功"时记一次（判到达），之后等位姿稳定再记一次（站稳）。巡逻时后端每到一个点不发成功，统计不到
  if (m.status === 2 && prev === 1 && m.task_type === 0 && S.goal && S.robot) {
    const e = { time: Date.now(), goal: S.goal, judged: goalError(S.robot, S.goal), settled: null,
      dur: S.navStart ? (Date.now() - S.navStart) / 1000 : null };
    S.arrivals.unshift(e); S.arrivals.length = Math.min(S.arrivals.length, 10);
    S.settle = { e, hist: [] };
    renderArrivals();
  }
  if (m.status === 3 && prev !== 3) {
    S.failure = { time: Date.now(), since: S.navStart || Date.now() - 60000 };
    renderFail();
    const f = failureHints()[0];
    toast(`导航失败${f ? "：" + f : ""}`);
  }
  renderNavInfo();
}
function checkSettled() {  // 位姿 2 秒内变化 <1 cm、<0.5° 算站稳；20 秒还不稳就按当时的位置记，并标出来
  const st = S.settle;
  if (!st || !S.robot) return;
  const now = Date.now();
  st.hist.push({ t: now, ...S.robot });
  st.hist = st.hist.filter((h) => now - h.t <= 2000);
  const still = now - st.hist[0].t >= 1800 && st.hist.every((h) =>
    Math.hypot(h.x - S.robot.x, h.y - S.robot.y) < 0.01 && Math.abs(wrapAngle(h.yaw - S.robot.yaw)) < 0.0087);
  if (still || now - st.e.time > 20000) {
    st.e.settled = goalError(S.robot, st.e.goal);
    st.e.unstable = !still;
    S.settle = null;
    renderArrivals();
  }
}
function planRemaining() {  // 沿 /plan 从离机器人最近的点算到终点
  const pl = S.globalPlan, r = S.robot;
  if (!pl || pl.length < 2 || !r) return null;
  let bi = 0, bd = Infinity;
  pl.forEach(([x, y], i) => { const d = Math.hypot(x - r.x, y - r.y); if (d < bd) { bd = d; bi = i; } });
  let sum = bd;
  for (let i = bi; i < pl.length - 1; i++) sum += Math.hypot(pl[i + 1][0] - pl[i][0], pl[i + 1][1] - pl[i][1]);
  return sum;
}
function renderNavInfo() {
  const running = S.task && (S.task.status === 1 || S.task.status === 4);
  const g = S.goal;
  $("niGoal").textContent = g ? `${g.name}  ${deg(g.yaw)}°` : "--";
  const rem = running ? planRemaining() : null;
  const straight = g && S.robot ? Math.hypot(g.x - S.robot.x, g.y - S.robot.y) : null;
  $("niRemain").textContent = rem != null ? `${rem.toFixed(2)} m（直线 ${straight != null ? straight.toFixed(2) : "--"} m）`
    : straight != null && running ? `直线 ${straight.toFixed(2)} m` : "--";
  const fresh = (v) => v && Date.now() - v.t < 1500;
  $("niCmd").textContent = fresh(S.cmdV) ? `${S.cmdV.vx.toFixed(2)} m/s，${S.cmdV.wz.toFixed(2)} rad/s` : "--";
  $("niOdom").textContent = fresh(S.odomV)
    ? `${S.odomV.vx.toFixed(2)} m/s${Math.abs(S.odomV.vy) >= 0.02 ? `（横移 ${S.odomV.vy.toFixed(2)}）` : ""}，${S.odomV.wz.toFixed(2)} rad/s` : "--";
  $("niTime").textContent = running && S.navStart ? `${Math.round((Date.now() - S.navStart) / 1000)} s` : "--";
}
const cm = (v) => `${(v * 100).toFixed(1)}`;
function renderArrivals() {
  const t = $("arrivals");
  t.innerHTML = "<tr><th>时间</th><th>目标</th><th>判到达</th><th>站稳</th><th>纵向</th><th>横向</th><th>航向</th><th>用时</th></tr>";
  for (const e of S.arrivals) {
    const tr = t.insertRow(), s = e.settled;
    const cells = [new Date(e.time).toLocaleTimeString(), e.goal.name, `${cm(e.judged.dist)}`,
      s ? `${cm(s.dist)}${e.unstable ? "?" : ""}` : "等站稳…", s ? cm(s.along) : "", s ? cm(s.lateral) : "",
      s ? `${(s.yaw * 180 / Math.PI).toFixed(1)}°` : "", e.dur != null ? `${Math.round(e.dur)} s` : ""];
    for (const c of cells) tr.insertCell().textContent = c;
    if (e.unstable) tr.title = "20 秒内没站稳，按 20 秒时的位置记";
  }
  if (!S.arrivals.length) { const c = t.insertRow().insertCell(); c.colSpan = 8; c.textContent = "还没有记录（单点导航成功后自动记录）"; }
  const done = S.arrivals.filter((e) => e.settled).map((e) => e.settled);
  $("arrStats").textContent = done.length
    ? `站稳误差 ${done.length} 次：平均 ${cm(done.reduce((a, s) => a + s.dist, 0) / done.length)} cm，` +
      `最大 ${cm(Math.max(...done.map((s) => s.dist)))} cm；航向最大 ${Math.max(...done.map((s) => Math.abs(s.yaw) * 180 / Math.PI)).toFixed(1)}°`
    : "";
}

// ---------------- Nav2 告警 / 失败原因（/rosout） ----------------
const NAV2_NODE = /^(bt_navigator|controller_server|planner_server|behavior_server|smoother_server|velocity_smoother|local_costmap|global_costmap|lifecycle_manager_navigation|waypoint_manage|g1_cmdvel_to_sport|collision_monitor)/;
// 常见 Nav2 日志 → 中文说明。按顺序匹配，取第一个
const NAV2_HINTS = [
  [/collision ahead|detected collision/i, "RPP 预测前方 1 秒内会撞上障碍物，停车"],
  [/failed to make progress/i, "控制器判定长时间没有前进（被挡住、卡住或速度太低）"],
  [/start occupied|start.*(lethal|in collision)|robot is out of bounds|starting point.*(occupied|lethal)/i, "起点在障碍物里（机器人被 costmap 包住了，看 local/global costmap）"],
  [/goal.*(occupied|lethal|in collision)/i, "目标点在障碍物上"],
  [/out of bounds|outside.*map|goal coordinates/i, "目标或起点在地图范围外"],
  [/failed to (create|compute) (a )?plan|no valid path|failed to plan|exceeded maximum iterations|planning algorithm/i, "全局规划失败（目标被障碍物或禁行区隔开，或在障碍里）"],
  [/goal.*rejected|rejected.*goal/i, "导航目标被拒绝"],
  [/transform|extrapolat|tf .*timeout|lookup.*(would require|into the future|past)/i, "TF 超时或太旧（定位或传感器延迟）"],
  [/costmap.*(not current|timed out|timeout)|sensor origin|no current/i, "costmap 数据过时（传感器数据没按时到）"],
  [/running (spin|wait|backup|clear|drive)|clearing.*costmap|clear.*costmap/i, "触发了恢复行为（原地转 / 等待 / 清图）"],
  [/no.*controller|controller.*(exception|failed)|failed to compute.*velocity|computevelocitycommands/i, "控制器算不出速度"],
  [/aborting|aborted|goal failed|navigation failed/i, "导航被中止"],
];
const nav2Hint = (msg) => { const h = NAV2_HINTS.find(([re]) => re.test(msg)); return h ? h[1] : ""; };
function onRosout(m) {
  if (!NAV2_NODE.test(m.name)) return;
  if (m.level < 30 && !/^behavior_server/.test(m.name)) return;  // WARN 以上；恢复行为（behavior_server）的 INFO 也留着
  const now = Date.now(), last = S.nav2Log[S.nav2Log.length - 1];
  if (last && last.name === m.name && last.msg === m.msg && now - last.time < 10000) { last.count++; last.time = now; }
  else S.nav2Log.push({ time: now, first: now, level: m.level, name: m.name, msg: m.msg, count: 1 });
  if (S.nav2Log.length > 60) S.nav2Log.splice(0, S.nav2Log.length - 60);
  renderNav2Log();
  if (S.failure && now - S.failure.time < 5000) renderFail();  // 日志可能比 /task_status 晚到一点
}
function failureLines() {
  const f = S.failure;
  return f ? S.nav2Log.filter((l) => l.time >= f.since - 1000 && l.first <= f.time + 5000) : [];
}
function failureHints() {
  return [...new Set(failureLines().slice().reverse().map((l) => nav2Hint(l.msg)).filter(Boolean))];
}
function renderFail() {
  const box = $("navFail"), f = S.failure;
  box.hidden = !f;
  if (!f) return;
  box.innerHTML = "";
  const h = document.createElement("div"); h.className = "t";
  h.textContent = `导航失败（${new Date(f.time).toLocaleTimeString()}）`;
  box.appendChild(h);
  const hints = failureHints(), lines = failureLines().slice(-8);
  for (const t of hints) { const d = document.createElement("div"); d.textContent = `• ${t}`; box.appendChild(d); }
  if (!lines.length) {
    const d = document.createElement("div"); d.className = "s";
    d.textContent = "这次任务期间没收到 Nav2 的告警日志：可能是目标被拒绝、导航还没起来，或者是后端（waypoint_manage）的问题，看下面的操作记录。";
    box.appendChild(d);
  }
  for (const l of lines) box.appendChild(nav2LogNode(l));
}
function nav2LogNode(l) {
  const d = document.createElement("div"); d.className = `s lv${l.level}`;
  d.textContent = `${new Date(l.time).toLocaleTimeString()} [${l.name}] ${l.msg}${l.count > 1 ? `（×${l.count}）` : ""}`;
  return d;
}
function renderNav2Log() {
  const ul = $("nav2Log"); ul.innerHTML = "";
  for (const l of S.nav2Log.slice(-12).reverse()) {
    const li = document.createElement("li");
    li.appendChild(nav2LogNode(l));
    const hint = nav2Hint(l.msg);
    if (hint) { const d = document.createElement("div"); d.className = "hint"; d.textContent = `→ ${hint}`; li.appendChild(d); }
    ul.appendChild(li);
  }
  if (!ul.children.length) ul.innerHTML = '<li class="meta">导航模式下 Nav2 的告警会显示在这里</li>';
}

// ---------------- 带宽统计 ----------------
// 统计 rosbridge 发到本页的每一条消息：JSON 按字符数（基本都是 ASCII）、CBOR 按字节数，按话题分类。
// 顺带记下每个话题最后收到的时间（状态页的"最近收到"）。
function jsonTopic(s) {
  const m = /"(topic|service)"\s*:\s*"([^"]+)"/.exec(s.slice(0, 400));
  if (m) return m[1] === "service" ? `服务 ${m[2]}` : m[2];
  return /"op"\s*:\s*"png"/.test(s.slice(0, 40)) ? "png 压缩（建图预览）" : "其他";
}
function cborTopic(u8) {  // CBOR map 里找键 "topic"（0x65 + 5 字节），后面是一个短文本串
  const n = Math.min(u8.length - 8, 80);
  for (let i = 0; i < n; i++) {
    if (u8[i] === 0x65 && u8[i + 1] === 0x74 && u8[i + 2] === 0x6f && u8[i + 3] === 0x70 && u8[i + 4] === 0x69 && u8[i + 5] === 0x63) {
      let j = i + 6, len = u8[j] - 0x60;
      if (u8[j] === 0x78) { len = u8[j + 1]; j++; } else if (len < 0 || len > 23) return "其他（CBOR）";
      return String.fromCharCode(...u8.subarray(j + 1, j + 1 + len));
    }
  }
  return "其他（CBOR）";
}
function meter(data) {
  if (!S.bw) return;
  let bytes = 0, topic = "其他";
  if (typeof data === "string") { bytes = data.length; topic = jsonTopic(data); }
  else if (data instanceof ArrayBuffer) { bytes = data.byteLength; topic = cborTopic(new Uint8Array(data)); }
  else if (data && data.size != null) { bytes = data.size; }
  const t = S.bw.topics[topic] || (S.bw.topics[topic] = { bytes: 0, msgs: 0 });
  t.bytes += bytes; t.msgs++; S.bw.total += bytes;
  S.seen[topic] = Date.now();
}
function bwSample() {  // 每秒记一次累计值，速率按最近 5 秒算
  if (!S.bw) return;
  const snap = { t: Date.now(), total: S.bw.total, topics: {} };
  for (const [k, v] of Object.entries(S.bw.topics)) snap.topics[k] = { bytes: v.bytes, msgs: v.msgs };
  S.bw.hist.push(snap);
  if (S.bw.hist.length > 6) S.bw.hist.shift();
}
function bwRates() {
  const h = S.bw && S.bw.hist;
  if (!h || h.length < 2) return null;
  const a = h[0], b = h[h.length - 1], dt = (b.t - a.t) / 1000;
  const topics = Object.entries(b.topics).map(([k, v]) => {
    const o = a.topics[k] || { bytes: 0, msgs: 0 };
    return { topic: k, kbps: (v.bytes - o.bytes) / 1024 / dt, hz: (v.msgs - o.msgs) / dt, total: v.bytes };
  }).sort((x, y) => y.kbps - x.kbps || y.total - x.total);
  return { kbps: (b.total - a.total) / 1024 / dt, topics };
}

// ---------------- 状态页：节点 / 数据流 / 带宽 ----------------
// 节点按启动它的 launch 分组：常驻 = robot.launch.py + 语义地图服务；定位 = lightning；导航 = g1_navigation.launch.py
const NODE_GROUPS = [
  { name: "常驻", want: () => true, nodes: ["/rosbridge_websocket", "/rosapi", "/robot_status_manager_node", "/launch_manager_node",
    "/map_manager_node", "/waypoint_manage_node", "/robot_pose_pub_node", "/livox_lidar_publisher", "/camera/camera", "/point_filter",
    "/g1_robot_state_publisher", "/g1_battery_state_bridge", "/semantic_map_server"] },
  { name: "定位", want: inLoc, nodes: ["/lightning_slam"] },
  { name: "导航", want: inNav, nodes: ["/planner_server", "/controller_server", "/bt_navigator", "/behavior_server",
    "/velocity_smoother", "/local_costmap/local_costmap", "/global_costmap/global_costmap", "/lifecycle_manager_navigation",
    "/g1_cmdvel_to_sport", "/g1_sport_to_odom", "/scan_range_filter", "/d435_mark_filter"] },
];
// sub：本页有没有订阅它（订阅了才有"最近收到"）
const TOPIC_CHECKS = [
  { topic: "/livox/lidar", label: "MID360 点云", want: () => true },
  { topic: "/livox/imu", label: "MID360 IMU", want: () => true, sub: true },
  { topic: "/camera/camera/depth/color/points", label: "D435 点云", want: () => true },
  { topic: "/camera/camera/depth/camera_info", label: "D435 深度流", want: () => true, sub: true },
  { topic: "/lightning/registered_scan", label: "LIO 单帧", want: inLoc },
  { topic: "/base_link_pose", label: "定位位姿", want: () => true, sub: true },
  { topic: "/lightning/loc_status", label: "定位守护", tip: "只在 LIO 关键帧发布，机器人不动时可能很久不更新", want: inLoc, sub: true, slow: true },
  { topic: "/battery_state", label: "电池", want: () => true, sub: true },
  { topic: "/odom", label: "腿式里程计", want: inNav, sub: true },
  { topic: "/local_costmap/costmap", label: "local costmap", want: () => inNav() && $("navCostmap").checked, sub: true },
  { topic: "/cmd_vel", label: "速度指令", tip: "Nav2 在跑任务时才发", want: inNav, sub: true, slow: true },
];
async function refreshHealth() {
  if (!S.connected || S.health.busy) return;
  S.health.busy = true;
  try {
    const r = await call(["/rosapi/nodes", "rosapi_msgs/srv/Nodes"], {}, 5000, true);
    S.health.nodes = new Set(r.nodes || []);
    await Promise.all(TOPIC_CHECKS.map(async (c) => {
      const p = await call(["/rosapi/publishers", "rosapi_msgs/srv/Publishers"], { topic: c.topic }, 5000, true).catch(() => null);
      S.health.pubs[c.topic] = p ? (p.publishers || []).length : null;
    }));
    S.health.at = Date.now(); S.health.err = "";
  } catch (e) {
    S.health.err = `rosapi 查询失败：${e.message}`;
  } finally { S.health.busy = false; }
  renderHealth();
}
function ageText(t) {
  if (!t) return ["—", ""];
  const a = (Date.now() - t) / 1000;
  return [a < 1.5 ? "刚刚" : `${Math.round(a)} 秒前`, a < 3 ? "ok" : a < 10 ? "warn" : "bad"];
}
function renderHealth() {
  if (S.tab !== "health") return;
  const H = S.health;
  $("healthErr").textContent = !S.connected ? "未连接机器人" : H.err || "";
  // 数据流
  const tt = $("topicTable");
  tt.innerHTML = "<tr><th>数据</th><th>话题</th><th>发布者</th><th>最近收到</th></tr>";
  for (const c of TOPIC_CHECKS) {
    const need = c.want(), tr = tt.insertRow(), pubs = H.pubs[c.topic];
    tr.className = need ? "" : "dim";
    const lc = tr.insertCell(); lc.textContent = c.label + (c.tip ? " ⓘ" : ""); lc.title = c.tip || "";
    tr.insertCell().textContent = c.topic;
    const pc = tr.insertCell();
    pc.textContent = pubs == null ? "?" : pubs;
    if (need && pubs === 0) pc.className = "bad";
    const ac = tr.insertCell();
    if (c.sub) {
      const [txt, lv] = ageText(S.seen[c.topic]);
      ac.textContent = txt;
      if (need && !c.slow) ac.className = lv || (pubs > 0 ? "bad" : "");
    } else { ac.textContent = "不订阅"; ac.title = "数据量大，不经 rosbridge 订阅，只查发布者"; }
  }
  // 节点
  const nt = $("nodeTable");
  nt.innerHTML = "";
  if (H.nodes) {
    for (const g of NODE_GROUPS) {
      const need = g.want(), missing = g.nodes.filter((n) => !H.nodes.has(n));
      const tr = nt.insertRow(); tr.className = need ? "" : "dim";
      tr.insertCell().textContent = `${g.name}${need ? "" : "（当前模式不需要）"}`;
      const c = tr.insertCell();
      c.textContent = missing.length ? `缺 ${missing.length} 个：${missing.join("  ")}` : `${g.nodes.length} 个都在`;
      if (need) c.className = missing.length ? "bad" : "ok";
    }
    const known = new Set(NODE_GROUPS.flatMap((g) => g.nodes));
    const others = [...H.nodes].filter((n) => !known.has(n) && !/^\/(_|transform_listener_impl|launch_ros)/.test(n));
    const tr = nt.insertRow(); tr.className = "dim";
    tr.insertCell().textContent = "其他";
    tr.insertCell().textContent = others.join("  ") || "无";
  }
  $("healthAt").textContent = H.at ? `节点和发布者 ${new Date(H.at).toLocaleTimeString()} 查询（每 3 秒）` : "";
  // 带宽
  const r = bwRates(), bt = $("bwTable");
  $("bwTotal").textContent = r
    ? `下行 ${r.kbps.toFixed(1)} KB/s（最近 5 秒平均），连接以来共 ${(S.bw.total / 1048576).toFixed(2)} MB / ${Math.round((Date.now() - S.bw.since) / 1000)} s`
    : "统计中…";
  bt.innerHTML = "<tr><th>话题</th><th>KB/s</th><th>条/秒</th><th>累计</th></tr>";
  for (const t of r ? r.topics : []) {
    const tr = bt.insertRow();
    tr.insertCell().textContent = t.topic;
    tr.insertCell().textContent = t.kbps.toFixed(1);
    tr.insertCell().textContent = t.hz.toFixed(1);
    tr.insertCell().textContent = t.total > 1048576 ? `${(t.total / 1048576).toFixed(1)} MB` : `${(t.total / 1024).toFixed(0)} KB`;
  }
}

// 每秒：带宽采样、导航信息（速度、用时）、定位标签的"几秒前"、状态页
let tickN = 0;
setInterval(() => {
  tickN++;
  bwSample();
  if (S.tab === "nav") { renderNavInfo(); if (tickN % 5 === 0) loadRadius(); }
  if (S.loc) renderLoc();
  if (S.tab === "health") { renderHealth(); if (tickN % 3 === 0) refreshHealth(); }
}, 1000);

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
function polyline(pts, color, width, dash = []) {
  if (!pts || pts.length < 2) return;
  ctx.strokeStyle = color; ctx.lineWidth = width; ctx.setLineDash(dash); ctx.lineJoin = ctx.lineCap = "round";
  ctx.beginPath();
  pts.forEach(([x, y], i) => { const [a, b] = w2s(x, y); i ? ctx.lineTo(a, b) : ctx.moveTo(a, b); });
  ctx.stroke(); ctx.setLineDash([]);
}
function draw() {
  const r = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);
  $("mapEmpty").hidden = !!S.map;
  if (!S.map) return;
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(S.map.img, S.view.ox, S.view.oy, S.map.width * S.view.scale, S.map.height * S.view.scale);
  if (!S.mappingPreview && S.costmap) {
    const cm = S.costmap, k = (cm.resolution / S.map.resolution) * S.view.scale;
    const [a, b] = w2s(cm.ox, cm.oy + cm.height * cm.resolution);
    ctx.drawImage(cm.canvas, a, b, cm.width * k, cm.height * k);
    ctx.strokeStyle = css("--muted"); ctx.lineWidth = 1; ctx.setLineDash([3, 3]);
    ctx.strokeRect(a, b, cm.width * k, cm.height * k); ctx.setLineDash([]);
  }
  if (!S.mappingPreview && $("navPaths").checked) {  // 规划路径：全局在下，局部和预测轨迹叠在上面
    polyline(S.globalPlan, css("--plan"), 3, [8, 5]);
    polyline(S.localPlan, css("--local-plan"), 3);
    polyline(S.arc, css("--arc"), 4);
  }
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
  updateSubs();
  if (tab === "health") { renderHealth(); guard(refreshHealth); }
  if (tab === "nav") loadRadius();
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
$("navCostmap").onchange = $("navPaths").onchange = () => { updateSubs(); draw(); };
$("radiusInput").addEventListener("input", checkRadiusInput);
$("radiusInput").addEventListener("keydown", (e) => { if (e.key === "Enter" && !$("btnRadiusApply").disabled) guard(applyRadius); });
$("btnRadiusApply").onclick = () => guard(applyRadius);
$("btnRadiusRefresh").onclick = () => guard(() => loadRadius(false));
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
resize(); setTab("maps"); renderLoc(); renderArrivals(); renderNav2Log(); renderRadius();
if (params.get("host") || location.hostname) connect($("host").value);
