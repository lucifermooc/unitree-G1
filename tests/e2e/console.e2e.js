// G1 控制台端到端测试：在浏览器里像人一样操作新前端，并通过后端服务核对结果。
// 需要先启动仿真（ros2 launch g1_sim sim.launch.py web_dir:=<仓库>/web），Qdrant 在运行。
//   node tests/e2e/console.e2e.js            （环境变量 URL 默认 http://localhost:8080/?host=localhost）
const { chromium } = require("playwright");
const fs = require("fs");

const URL = process.env.URL || "http://localhost:8080/?host=localhost";
const SHOT_DIR = process.env.SHOT_DIR || ".";
const CHROME = process.env.CHROME || undefined;
const SEM_LOG = process.env.SEM_LOG || "";   // 语义地图调试记录文件（仿真里在 <sim_home>/maps/semantic_map_log.jsonl）
const results = [];
function check(ok, name, detail = "") {
  results.push({ ok, name, detail });
  console.log(`${ok ? "PASS" : "FAIL"} ${name}${detail ? " — " + detail : ""}`);
}

(async () => {
  const browser = await chromium.launch(CHROME ? { executablePath: CHROME } : {});
  const page = await browser.newPage({ viewport: { width: 1400, height: 860 } });
  const jsErrors = [];
  page.on("pageerror", (e) => jsErrors.push(e.message));
  page.setDefaultTimeout(30000);

  // ---------- 小工具 ----------
  const text = (id) => page.locator(`#${id}`).innerText();
  const S = (fn, arg) => page.evaluate(fn, arg);
  const srv = (key, args = {}) => S(([k, a]) => call(SRV[k], a), [key, args]);  // 直接调后端核对
  const until = async (fn, what, timeout = 30000) => {
    const t0 = Date.now();
    while (Date.now() - t0 < timeout) {
      try { if (await fn()) return true; } catch (e) { /* 继续等 */ }
      await page.waitForTimeout(300);
    }
    throw new Error(`等待超时：${what}`);
  };
  const step = async (name, fn) => {
    try { await fn(); } catch (e) { check(false, name, e.message.split("\n")[0]); }
  };
  const tab = (t) => page.click(`#tabs button[data-tab="${t}"]`);
  const toScreen = async (x, y) => {
    const box = await page.locator("#map").boundingBox();
    const [sx, sy] = await S(([x, y]) => w2s(x, y), [x, y]);
    return [box.x + sx, box.y + sy];
  };
  const dragOnMap = async (x, y, yawDeg = 0) => {
    await page.click("#btnFit");   // 语义搜索会把视图移到命中的点位，先缩放到整张地图
    const [sx, sy] = await toScreen(x, y);
    const a = (yawDeg * Math.PI) / 180;
    await page.mouse.move(sx, sy); await page.mouse.down();
    await page.mouse.move(sx + Math.cos(a) * 25, sy - Math.sin(a) * 25, { steps: 4 });
    await page.mouse.move(sx + Math.cos(a) * 50, sy - Math.sin(a) * 50, { steps: 4 });
    await page.mouse.up();
  };
  const clickMap = async (x, y) => { const [sx, sy] = await toScreen(x, y); await page.mouse.click(sx, sy); };
  const robot = () => S(() => app.robot);
  const near = async (x, y, tol) => { const r = await robot(); return r && Math.hypot(r.x - x, r.y - y) < tol; };
  const dbPoints = async () => {
    const cur = await srv("getCurrent");
    const r = await srv("pointList", { map_id: cur.map_id, data_type: "waypoint_node" });
    return JSON.parse(r.message).map((row) => ({ id: row.id, ...JSON.parse(row.point_list) }));
  };
  const addPoint = async (name, desc, x, y, yawDeg) => {
    await dragOnMap(x, y, yawDeg);
    await page.fill("#pName", name);
    await page.fill("#pDesc", desc);
    await page.click("#btnPointSave");
    await until(async () => (await dbPoints()).some((p) => p.name === name), `点位 ${name} 写入数据库`);
  };

  // ---------- 1. 连接与状态栏 ----------
  await page.goto(URL);
  await step("连接 rosbridge 并显示状态", async () => {
    await until(async () => (await text("stConn")).includes("已连接"), "连接");
    await until(async () => /\d+%/.test(await text("stBattery")), "电量（BatteryState 含 NaN，CBOR 订阅）");
    await until(async () => (await text("stMode")) !== "--", "模式");
    check(true, "连接 rosbridge 并显示状态",
      `电量 ${await text("stBattery")}，模式 ${await text("stMode")}，任务 ${await text("stTask")}，地图 ${await text("stMap")}`);
  });

  // ---------- 2. 建图 ----------
  await step("建图：开始 → 实时预览 → 保存 → 写入数据库", async () => {
    await tab("mapping");
    await page.click("#btnMapSave");  // 空名称
    await until(async () => (await text("toast")).includes("先填地图名称"), "空名称提示");
    await page.click("#btnMapStart");
    await until(async () => (await text("stMode")).startsWith("建图"), "进入建图模式");
    await until(() => S(() => !!app.map && app.mappingPreview), "收到 /map_base64 预览");
    const p0 = await robot();
    await page.waitForTimeout(55000);   // 等模拟机器人把办公室走一遍
    const p1 = await robot();
    await page.screenshot({ path: `${SHOT_DIR}/e2e_mapping.png` });
    check(p0 && p1 && (p0.x !== p1.x || p0.y !== p1.y), "建图时机器人位置在更新（/base_link_pose）",
      `(${p0 && p0.x.toFixed(1)}, ${p0 && p0.y.toFixed(1)}) → (${p1 && p1.x.toFixed(1)}, ${p1 && p1.y.toFixed(1)})`);
    await page.fill("#mapName", "一楼办公室");
    await page.click("#btnMapSave");
    await until(async () => JSON.parse((await srv("mapList")).map_list).some((m) => m.name === "一楼办公室"), "地图写入数据库");
    await until(async () => (await text("stMode")).startsWith("空闲"), "保存后退出建图");
    check(true, "建图：开始 → 实时预览 → 保存 → 写入数据库");
    // 不在建图模式时再保存应失败（后端返回 must change mode to mapping first）
    await page.fill("#mapName", "不应该存在");
    await page.click("#btnMapSave");
    await until(async () => (await text("toast")).includes("保存地图失败"), "非建图模式保存报错");
    check(!JSON.parse((await srv("mapList")).map_list).some((m) => m.name === "不应该存在"), "非建图模式下保存会被拒绝");
    await page.fill("#mapName", "");
  });

  // ---------- 3. 使用地图 ----------
  await step("使用地图：设为当前 + 进入定位 + 显示地图", async () => {
    await tab("maps");
    await page.locator("#mapList li", { hasText: "一楼办公室" }).locator('[data-act="use"]').click();
    await until(async () => (await text("stMap")) === "一楼办公室", "当前地图");
    await until(async () => (await text("stMode")).startsWith("定位"), "定位模式");
    await until(() => S(() => app.map && app.map.width === 480 && !app.mappingPreview), "地图图片");
    const warn = await S(() => cellWarning(12.5, 4.0));
    check(warn === "", "使用地图：设为当前 + 进入定位 + 显示地图", `茶水间位置已探索：${warn === "" ? "是" : warn}`);
  });

  // ---------- 4. 点位 ----------
  const POINTS = [
    ["茶水间", "有饮水机和咖啡机，可以接水、喝水、泡茶", 12.5, 4.0, 90],
    ["会议室", "开会、讨论和做汇报的地方", 7.0, 2.6, 90],
    ["卫生间", "上厕所、洗手", 5.0, -4.0, -90],
    ["前台", "访客登记、取快递", 0.5, 2.8, 180],
    ["仓库", "存放纸箱和工具", 11.0, -2.6, -90],
  ];
  await step("点位：在地图上拖动新建（含描述和朝向）", async () => {
    await tab("points");
    for (const p of POINTS) await addPoint(...p);
    const db = await dbPoints();
    const tea = db.find((p) => p.name === "茶水间");
    const yaw = Math.atan2(2 * tea.orientation.w * tea.orientation.z, 1 - 2 * tea.orientation.z ** 2) * 180 / Math.PI;
    check(db.length === 5 && Math.hypot(tea.position.x - 12.5, tea.position.y - 4.0) < 0.1 && Math.abs(yaw - 90) < 3
      && tea.description.includes("饮水机"), "点位：在地图上拖动新建（含描述和朝向）",
      `数据库 ${db.length} 个；茶水间 (${tea.position.x}, ${tea.position.y}) 朝向 ${yaw.toFixed(0)}°`);
    check((await page.locator("#pointList li").count()) === 5, "点位列表显示 5 个");
  });
  await step("点位：修改描述 / 拖动移动 / 障碍物提示 / 删除 / 当前位置新建", async () => {
    await page.locator("#pointList li", { hasText: "会议室" }).locator(".grow").click();
    await page.fill("#pDesc", "开会、讨论、做汇报，有投影仪");
    await page.click("#btnPointSave");
    await until(async () => (await dbPoints()).find((p) => p.name === "会议室").description.includes("投影仪"), "描述更新");
    check(true, "修改点位描述（/update_point）");

    const [sx, sy] = await toScreen(11.0, -2.6);
    const [tx, ty] = await toScreen(12.0, -2.5);
    await page.mouse.move(sx, sy); await page.mouse.down(); await page.mouse.move(tx, ty, { steps: 8 }); await page.mouse.up();
    await until(async () => { const p = (await dbPoints()).find((q) => q.name === "仓库"); return Math.abs(p.position.x - 12.0) < 0.1; }, "移动后坐标");
    check(true, "拖动点位移动位置（/update_point）");

    await dragOnMap(9.0, 1.5, 0);   // 走廊北墙上（桌子内部激光扫不到，地图上是"未探索"，所以选墙）
    let warn = await text("pWarn");
    check(warn.includes("障碍物"), "点在墙上会提示障碍物 / 离障碍物太近", warn);
    await page.click("#btnPointCancel");
    await dragOnMap(7.0, 4.2, 0);   // 会议桌中间
    warn = await text("pWarn");
    check(warn.includes("未探索"), "点在桌子内部会提示未探索", warn);
    await page.click("#btnPointCancel");

    await addPoint("临时点", "马上删掉", 3.0, 0.0, 0);
    await page.locator("#pointList li", { hasText: "临时点" }).locator(".grow").click();
    await page.click("#btnPointDelete"); await page.click("#btnPointDelete");
    await until(async () => !(await dbPoints()).some((p) => p.name === "临时点"), "删除点位");
    check(true, "删除点位（两次确认，/delete_point）");

    await page.click("#btnPointHere");
    const r = await robot();
    const fx = +(await page.inputValue("#pX")), fy = +(await page.inputValue("#pY"));
    check(Math.hypot(fx - r.x, fy - r.y) < 0.05, "在机器人当前位置新建：表单填入当前位置", `(${fx}, ${fy})`);
    await page.click("#btnPointCancel");
  });

  // ---------- 5. 语义地图 ----------
  const semantic = async (q, go = false) => {
    await page.fill("#semQuery", q);
    await page.click(go ? "#btnSemGo" : "#btnSemSearch");
    await until(async () => !(await text("semResult")).includes("搜索中"), `语义结果：${q}`, 180000);
    return text("semResult");
  };
  await step("语义搜索：一句话找到点位", async () => {
    for (const [q, expect] of [["我想喝水", "茶水间"], ["一会儿要开会，投影仪在哪", "会议室"], ["快递到了", "前台"]]) {
      const r = await semantic(q);
      check(r.startsWith(`→ ${expect}`), `语义搜索"${q}"`, r.split("\n")[0]);
    }
    const r = await semantic("今天股票涨了吗");
    check(r.startsWith("没有匹配"), "无关的话不匹配任何点位", r.split("\n")[0]);
  });
  await step("语义地图随点位自动更新", async () => {
    await addPoint("打印室", "有打印机和复印机，打印、复印、扫描文件", 16.0, -4.5, -90);
    const r = await semantic("我要打印文件");
    check(r.startsWith("→ 打印室"), "新增点位后语义库自动重建", r.split("\n")[0]);
  });
  await step("语义地图调试记录：看得到别人发来的原文和得分", async () => {
    const entries = () => S(() => app.semLog);
    const last = async (pred, what) => {
      await until(async () => (await entries()).some(pred), what, 180000);
      return (await entries()).filter(pred).pop();
    };
    // 网页自己的搜索也会记下来（来源 web）
    const w = await last((r) => r.source === "web" && r.raw === "我想喝水", "网页搜索的记录");
    check(w.ok && w.best === "茶水间" && w.candidates.length === 3 && w.issues.length === 0,
      "网页搜索写进调试记录（来源、原文、前 3 名得分）", w.candidates.map((c) => `${c.name}:${c.score}`).join(" "));
    // 模拟 ASR：往话题发一句带首尾空格和零宽空格的话
    const bad = " 我想\u200b喝水 ";
    await page.fill("#semAsrText", bad);
    await page.click("#btnSemAsr");
    const t = await last((r) => r.entry === "topic" && r.raw === bad, "话题输入的记录");
    check(t.ok && t.best === "茶水间" && t.issues.some((i) => i.includes("首尾有空白")) &&
      t.issues.some((i) => i.includes("零宽空格")) && t.raw_visible === "␠我想⟨U+200B⟩喝水␠",
      "ASR 话题输入：照常匹配，同时标出原文里的首尾空格和零宽空格", `${t.raw_visible} ${t.issues.join("；")}`);
    await until(async () => (await page.locator("#semDebug li .inv").count()) >= 3, "不可见字符高亮");
    const first = await page.locator("#semDebug li").first().innerText();
    check(first.includes("话题 text_in") && first.includes("零宽空格") && first.includes("→ 茶水间"),
      "网页调试面板显示来源、原文问题和匹配结果", first.replace(/\n/g, " | "));
    // 别的程序调服务，并用 JSON 标明来源
    const r = await srv("semSearch", { data: JSON.stringify({ text: "快递到了", source: "asr" }) });
    const res = JSON.parse(r.message);
    check(r.success && res.source === "asr" && res.best && res.best.name === "前台",
      "服务调用可带来源（{text, source}），应答里也有来源和问题列表", `${res.source} → ${res.best && res.best.name}`);
    const a = await last((x) => x.source === "asr" && x.entry === "search", "asr 来源的记录");
    check(a.text === "快递到了", "调试记录区分来源 asr（经服务 search）");
    // 空内容：出错也要留记录
    await S(() => publish("/semantic_map/text_in", "std_msgs/msg/String", { data: "\u200b  " }));
    const e = await last((x) => x.entry === "topic" && x.ok === false, "出错的记录");
    check(e.error === "empty query" && e.issues.includes("清理后是空的"), "发来空内容时记录出错原因", e.issues.join("；"));
    await page.check("#semDebugIssues");
    const n = await page.locator("#semDebug li").count();
    await page.uncheck("#semDebugIssues");
    check(n >= 2 && n < (await page.locator("#semDebug li").count()), "只看有问题的记录", `${n} 条`);
    // 记录文件
    if (SEM_LOG) {
      const lines = fs.readFileSync(SEM_LOG, "utf8").trim().split("\n").map((l) => JSON.parse(l));
      check(lines.some((l) => l.raw === bad && l.source === "topic") && lines.some((l) => l.source === "asr"),
        "调试记录同时写进文件（每行一条 JSON）", `${lines.length} 行`);
    }
  });
  await step("一句话导航：我要上厕所 → 卫生间", async () => {
    const r = await semantic("我要上厕所", true);
    check(r.includes("卫生间") && r.includes("出发"), "语义导航发出目标", r.replace(/\n/g, " | "));
    await until(async () => (await text("stTask")).includes("执行中"), "任务执行中", 15000);
    await until(async () => (await text("stTask")).includes("成功"), "导航成功", 90000);
    const rb = await robot();
    check(await near(5.0, -4.0, 0.3), "机器人到达卫生间", `位置 (${rb.x.toFixed(2)}, ${rb.y.toFixed(2)})`);
  });

  // ---------- 6. 单点导航 + 暂停/继续/取消 ----------
  await step("单点导航：暂停 / 继续 / 取消 / 到达", async () => {
    await tab("nav");
    const id = (await dbPoints()).find((p) => p.name === "茶水间").id;
    await page.selectOption("#navPoint", String(id));
    await page.click("#btnNavGo");
    await until(async () => (await text("stTask")) === "单点导航 · 执行中", "开始导航");
    await page.waitForTimeout(2500);
    await page.click("#btnPause");
    await until(async () => (await text("stTask")).includes("暂停"), "暂停");
    const a = await robot(); await page.waitForTimeout(1500); const b = await robot();
    check(Math.hypot(a.x - b.x, a.y - b.y) < 0.05, "暂停后机器人停住", `1.5 s 内移动 ${Math.hypot(a.x - b.x, a.y - b.y).toFixed(3)} m`);
    await page.click("#btnResume");
    await until(async () => (await text("stTask")).includes("执行中"), "继续");
    await page.waitForTimeout(1500);
    await page.click("#btnCancel");
    await until(async () => (await text("stTask")).includes("已取消"), "取消");
    check(true, "暂停 → 继续 → 取消，任务状态正确");
    await page.click("#btnNavGo");
    await until(async () => (await text("stTask")).includes("成功"), "到达茶水间", 90000);
    check(await near(12.5, 4.0, 0.3), "取消后重新导航并到达茶水间");
  });

  // ---------- 7. 巡逻 ----------
  await step("巡逻：勾选顺序 → 开始 → 依次经过 → 取消", async () => {
    for (const name of ["前台", "会议室", "茶水间"]) {
      await page.locator("#patrolList li", { hasText: name }).locator("input").check();
    }
    const orderOf = (name) => page.locator("#patrolList li", { hasText: name }).locator(".order").innerText();
    const orders = [await orderOf("前台"), await orderOf("会议室"), await orderOf("茶水间")];
    check(orders.join(",") === "1,2,3", "巡逻顺序编号按勾选顺序", orders.join(","));
    await page.click("#btnPatrolStart");
    await until(async () => (await text("stTask")) === "巡逻 · 执行中", "巡逻开始");
    await until(() => near(0.5, 2.8, 0.35), "到达第 1 个点（前台）", 90000);
    await until(() => near(7.0, 2.6, 0.35), "到达第 2 个点（会议室）", 90000);
    check(true, "巡逻依次经过前台、会议室");
    await page.click("#btnCancel");
    await until(async () => (await text("stTask")).includes("已取消"), "取消巡逻");
    check(true, "取消巡逻");
    await page.screenshot({ path: `${SHOT_DIR}/e2e_patrol.png` });
  });

  // ---------- 8. 禁行线 ----------
  await step("禁行线：画线 → 保存到数据库 → 画到导航地图", async () => {
    await tab("edit");
    await page.click("#btnNogoDraw");
    await clickMap(9.0, -1.2); await clickMap(9.0, 1.2);
    check((await page.locator("#nogoList li").count()) === 1, "画出 1 条禁行线");
    await page.click("#btnNogoSave");
    await until(async () => (await srv("getForbidden", { map_id: (await srv("getCurrent")).map_id })).lines.length === 1, "数据库");
    const t = await text("toast");
    check(t.includes("已保存 1 条禁行线"), "禁行线保存成功（/set_forbidden + /aid_draw_forbidden_line）", t);
    await page.click("#btnNogoDraw");
  });

  // ---------- 9. 橡皮擦 ----------
  await step("橡皮擦：画成障碍 → 写回地图 → 擦回空地", async () => {
    await page.selectOption("#eraserGray", "100");
    await page.click("#btnEraser");
    await clickMap(3.0, 0.8);
    await page.click("#btnEraserApply");
    await until(() => S(() => cellWarning(3.0, 0.8).includes("障碍物")), "地图出现障碍", 15000);
    check(true, "橡皮擦画成障碍后地图更新（/map_editor）");
    await page.selectOption("#eraserGray", "0");
    await clickMap(3.0, 0.8);
    await page.click("#btnEraserApply");
    await until(() => S(() => cellWarning(3.0, 0.8) === ""), "擦回空地", 15000);
    check(true, "橡皮擦擦回空地");
    await page.click("#btnEraser");
  });

  // ---------- 10. 刷新页面后数据还在 ----------
  await step("刷新页面后地图、点位、禁行线仍在", async () => {
    await page.reload();
    await until(async () => (await text("stMap")) === "一楼办公室", "重新连接");
    await until(() => S(() => app.points.length === 6 && app.nogo.length === 1 && !!app.map), "重新加载",
      15000).catch(async (e) => { throw new Error(e.message + " " + JSON.stringify(await S(() =>
        ({ points: app.points.map((p) => p.name), nogo: app.nogo.length, map: !!app.map })))); });
    check(true, "刷新页面后地图、点位、禁行线仍在");
    await until(() => S(() => app.semLog.some((r) => r.source === "asr")), "刷新后从 /semantic_map/history 补回调试记录");
    check(true, "刷新页面后调试记录仍在（/semantic_map/history）");
  });

  // ---------- 11. 模式与重定位 ----------
  await step("模式切换与重定位", async () => {
    await tab("mode");
    await page.click('.modes button[data-mode="remote_control"]');
    await until(async () => (await text("stMode")).includes("遥控"), "遥控");
    await page.click('.modes button[data-mode="patrol"]');
    await until(async () => (await text("stMode")) === "定位 / 导航", "导航");
    check(true, "模式切换：遥控 / 导航（/mode_set + /robot_status）");
    await page.click("#btnRelocateZero");
    await until(() => near(0, 0, 0.05), "重定位到原点", 5000);
    await page.click("#btnRelocate");
    await dragOnMap(3.0, -0.5, 180);
    await until(async () => { const r = await robot(); return Math.hypot(r.x - 3.0, r.y + 0.5) < 0.1 && Math.abs(Math.abs(r.yaw) - Math.PI) < 0.1; }, "拖动重定位", 5000);
    check(true, "重定位：回原点 / 在地图上拖动设置（/initialpose + /start_init_pose，仿真）");
  });

  // ---------- 12. 接口文档与后端不一致的地方（记录，不算失败） ----------
  await step("核对：PatrolControl 是否支持 patrol_count", async () => {
    const r = await S(() => call(SRV.patrol, { cmd: "cancel", patrol_count: -1, patrol_duration: -1.0 })
      .then(() => "accepted").catch((e) => "rejected: " + e.message));
    console.log(`INFO 带 patrol_count/patrol_duration 调用 /patrol_control：${r}`);
  });

  // ---------- 13. 地图重命名 / 删除 ----------
  await step("地图重命名与删除", async () => {
    await tab("maps");
    const li = page.locator("#mapList li", { hasText: "一楼办公室" });
    await li.locator('[data-act="rename"]').click();
    await page.keyboard.press("Control+A");
    await page.keyboard.type("一楼办公室-改");
    await page.keyboard.press("Enter");
    await until(async () => JSON.parse((await srv("mapList")).map_list).some((m) => m.name === "一楼办公室-改"), "重命名");
    await until(async () => (await text("stMap")) === "一楼办公室-改", "状态栏更新");
    check(true, "地图重命名（/update_map）");
    await page.screenshot({ path: `${SHOT_DIR}/e2e_final.png` });
    const del = page.locator("#mapList li", { hasText: "一楼办公室-改" }).locator('[data-act="delete"]');
    await del.click(); await del.click();
    await until(async () => JSON.parse((await srv("mapList")).map_list).length === 0, "删除地图");
    await until(async () => (await text("stMap")) === "未选择", "当前地图清空");
    check(true, "删除地图（两次确认，/delete_map）");
  });

  check(jsErrors.length === 0, "页面没有 JS 错误", jsErrors.join(" | "));
  await browser.close();
  const failed = results.filter((r) => !r.ok);
  console.log(`\n共 ${results.length} 项，通过 ${results.length - failed.length}，失败 ${failed.length}`);
  process.exit(failed.length ? 1 : 0);
})();
