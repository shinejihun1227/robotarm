# server_codeplant_multi_control_v2.py
# Codeplant Robot Arm Central Server v2
# - ESP32 STA 자동등록 확인용 패널 강화
# - 등록된 로봇팔 목록/선택/상태 확인 UI 추가
# - 05/06 Web Functions Proxy: 선택한 로봇팔에게만 명령 전송

from flask import Flask, request, jsonify, render_template_string
import requests
import time
import re
import socket

app = Flask(__name__)

ROBOTS = {}
REGISTER_LOG = []

ALLOWED_ENDPOINTS = {
    "neutral_raw", "home_calibrated", "status",
    "calib_joint", "calib_grip_rel",
    "joint", "joint0", "scan_start", "scan_stop", "scan_status",
    "move", "live_xyz", "grip",
}

HTML = r"""
<!DOCTYPE html>
<html lang="ko">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Codeplant Robot Arm Central Server</title>
<style>
*{box-sizing:border-box;margin:0;padding:0}
:root{
  --mint:#dff5f3;--mint2:#c7ebe8;--teal:#20c5c7;--tealDark:#11999e;
  --ink:#16181d;--sub:#5f6b72;--paper:#fff;--soft:#f7fbfb;--line:#d5e8e7;
  --red:#ff4d5e;--orange:#ff9f1c;--green:#18a66a;--blue:#2563eb;
  --violet:#6d5dfc;--navy:#172033;--slate:#607080
}
body{font-family:Arial,'Noto Sans KR',sans-serif;background:linear-gradient(180deg,var(--mint),#eefaf9);color:var(--ink);padding:18px;min-height:100vh}
.wrap{max-width:1220px;margin:0 auto}
.title{background:var(--paper);border:10px solid var(--mint2);border-radius:12px;padding:24px 22px;box-shadow:0 10px 28px rgba(20,90,92,.10);position:relative;overflow:hidden;margin-bottom:16px}
.title:after{content:'';position:absolute;left:0;right:0;bottom:0;height:6px;background:linear-gradient(90deg,var(--tealDark),var(--red),var(--orange))}
h1{font-size:2.05rem;line-height:1.05;letter-spacing:-.06em;font-weight:900;color:var(--ink)}
.subtitle{margin-top:7px;color:var(--sub);font-weight:800}
.grid{display:grid;grid-template-columns:390px 1fr;gap:16px;align-items:start}
.card{background:var(--paper);border:1px solid var(--line);border-radius:12px;padding:16px;box-shadow:0 6px 20px rgba(20,80,80,.07);margin-bottom:14px}
.card h2{font-size:1.04rem;font-weight:900;letter-spacing:-.04em;border-left:8px solid var(--red);padding:4px 0 5px 10px;margin-bottom:12px}
.card h2:after{content:'';display:block;width:44px;height:3px;background:var(--teal);margin-top:7px;border-radius:99px}
.small{font-size:.78rem;color:var(--sub);line-height:1.45;margin-top:8px}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-top:8px;align-items:center}
button{border:0;border-radius:9px;padding:10px 12px;font-weight:900;color:#fff;cursor:pointer;box-shadow:0 4px 0 rgba(0,0,0,.15);letter-spacing:-.02em}
button:active{transform:translateY(1px);box-shadow:0 2px 0 rgba(0,0,0,.18)}
button:disabled{opacity:.55;cursor:not-allowed}
.btn-main{background:linear-gradient(135deg,var(--tealDark),var(--blue))}
.btn-ok{background:linear-gradient(135deg,var(--green),#35d58a)}
.btn-warn{background:linear-gradient(135deg,var(--orange),#ff6b35)}
.btn-stop{background:linear-gradient(135deg,var(--red),#d9283f)}
.btn-special{background:linear-gradient(135deg,var(--navy),var(--violet))}
.btn-gray{background:linear-gradient(135deg,var(--slate),#93a1ad)}
label{display:flex;justify-content:space-between;align-items:center;margin:9px 0 4px;font-size:.84rem;font-weight:900;color:#30363a}
label span{color:var(--red)}
input,select,textarea{width:100%;background:#fff;color:var(--ink);border:1.5px solid #b9d2d2;border-radius:8px;padding:9px;font-weight:800}
input[type=number]{max-width:118px}
input[type=checkbox]{width:auto}
.formline{display:flex;align-items:center;gap:8px;flex-wrap:wrap;margin:6px 0}
.formline input{width:90px}
.statusBar{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:12px}
.stat{background:#eefafa;border:1px solid #c8dddd;border-radius:10px;padding:10px}
.stat b{display:block;font-size:.78rem;color:var(--sub)}
.stat span{font-size:1.05rem;font-weight:900;color:var(--ink)}
.robotList{margin-top:12px}
.empty{background:#fff7e8;border:1px solid #ffd391;border-left:6px solid var(--orange);border-radius:10px;padding:12px;color:#5d3a00;font-size:.84rem;line-height:1.45}
.robot{border:1.5px solid #d5e7e7;border-radius:10px;padding:12px;background:var(--soft);margin-bottom:10px}
.robotTop{display:flex;align-items:flex-start;gap:9px}
.robot input.robotCheck{transform:scale(1.2);margin-top:5px}
.robot h3{font-size:1rem;font-weight:900;letter-spacing:-.03em}
.robot .ip{font-family:Consolas,monospace;font-size:.78rem;color:var(--sub);margin-top:4px;word-break:break-all}
.badge{display:inline-block;margin-top:8px;padding:4px 8px;border-radius:99px;background:#e5fbf8;color:#097f82;font-size:.72rem;font-weight:900}
.badge.off{background:#f3f4f6;color:#6b7280}
.selectedBox,.serverBox{background:#eefafa;border:1.5px solid #c8dddd;border-left:6px solid var(--teal);border-radius:9px;padding:10px;font-family:Consolas,monospace;font-size:.83rem;color:#1d292d;min-height:42px;white-space:pre-wrap}
.tabs{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:12px}
.tab{background:#edf7f7;color:#243033;border:1px solid #c8dddd;box-shadow:none}
.tab.active{background:linear-gradient(135deg,var(--tealDark),var(--blue));color:#fff}
.panel{display:none}
.panel.active{display:block}
.controlNote{background:#fff7e8;border:1px solid #ffd391;border-left:6px solid var(--orange);border-radius:9px;padding:10px;font-size:.8rem;color:#5d3a00;line-height:1.45;margin:8px 0}
.log{background:var(--navy);color:#e9feff;border:3px solid var(--teal);border-radius:10px;padding:12px;min-height:240px;max-height:420px;overflow:auto;font-size:.79rem;white-space:pre-wrap;font-family:Consolas,monospace}
.inlineBtn{padding:8px 9px;font-size:.8rem}
.twoCol{display:grid;grid-template-columns:1fr 1fr;gap:12px}
.tableBox{overflow:auto}
table{width:100%;border-collapse:collapse;font-size:.8rem}
th,td{border-bottom:1px solid #e0eeee;padding:8px;text-align:left}
th{color:#526268;background:#f7fbfb;font-weight:900}
td.mono{font-family:Consolas,monospace}
@media(max-width:980px){.grid{grid-template-columns:1fr}.statusBar{grid-template-columns:1fr}.twoCol{grid-template-columns:1fr}h1{font-size:1.55rem}}
</style>
</head>
<body>
<div class="wrap">
  <div class="title">
    <h1>Codeplant Robot Arm<br>Central Server</h1>
    <div class="subtitle">STA Robot Selection · Group Control · 05 Web Functions Proxy</div>
    <div class="statusBar">
      <div class="stat"><b>Server</b><span id="serverAddr">checking...</span></div>
      <div class="stat"><b>Registered Robots</b><span id="robotCount">0</span></div>
      <div class="stat"><b>Selected Targets</b><span id="selectedCount">0</span></div>
    </div>
  </div>

  <div class="grid">
    <aside>
      <div class="card">
        <h2>Connection Check</h2>
        <div class="serverBox" id="serverBox">서버 상태 확인 중...</div>
        <p class="small">
          ESP32 Shell에서 <b>중앙 서버 등록 요청 완료</b>가 떠도, 이 목록에 안 뜨면
          서버 IP, 방화벽, 핫스팟 기기 간 통신, 실행 중인 서버 파일을 확인해야 합니다.
        </p>
      </div>

      <div class="card">
        <h2>Registered Robot List</h2>
        <div class="row">
          <button class="btn-main inlineBtn" onclick="loadRobots(true)">Refresh</button>
          <button class="btn-ok inlineBtn" onclick="selectAll()">Select All</button>
          <button class="btn-gray inlineBtn" onclick="clearSelected()">Clear</button>
        </div>
        <div id="robotList" class="robotList"></div>
      </div>

      <div class="card">
        <h2>Selected Targets</h2>
        <div id="selectedBox" class="selectedBox">선택된 로봇팔 없음</div>
      </div>

      <div class="card">
        <h2>Registration Log</h2>
        <div class="row"><button class="btn-gray inlineBtn" onclick="loadRegisterLog()">Refresh Log</button></div>
        <pre id="regLog" class="selectedBox" style="max-height:180px;overflow:auto">등록 로그 없음</pre>
      </div>
    </aside>

    <main>
      <div class="card">
        <h2>Control Menu</h2>
        <div class="tabs">
          <button class="tab active" onclick="showPanel('p0',this)">00 Home</button>
          <button class="tab" onclick="showPanel('p1',this)">01 Offset</button>
          <button class="tab" onclick="showPanel('p2',this)">02 Angle</button>
          <button class="tab" onclick="showPanel('p3',this)">03/04 XYZ</button>
          <button class="tab" onclick="showPanel('p4',this)">Grip</button>
          <button class="tab" onclick="showPanel('p5',this)">Advanced</button>
        </div>

        <section id="p0" class="panel active">
          <h2>00 Neutral / Home / Status</h2>
          <div class="row">
            <button class="btn-warn" onclick="group('neutral_raw',{})">Raw Neutral</button>
            <button class="btn-ok" onclick="group('home_calibrated',{})">Calibrated Home</button>
          </div>
          <label>Status Target</label>
          <select id="statusTarget">
            <option value="all">전체 관절정보</option><option value="base">base</option><option value="shoulder">shoulder</option><option value="elbow">elbow</option><option value="wrist_r">wrist_r</option><option value="wrist_p">wrist_p</option><option value="grip">grip</option>
          </select>
          <div class="row">
            <button class="btn-gray" onclick="group('status',{target:val('statusTarget')})">Status</button>
          </div>
        </section>

        <section id="p1" class="panel">
          <h2>01 Offset Calibration</h2>
          <div class="controlNote">Offset Calibration은 로봇마다 조립 오차가 다르므로 <b>1대만 선택</b>하고 진행하는 것을 권장합니다.</div>
          <div class="twoCol">
            <div>
              <label>Joint</label>
              <select id="calibJoint"><option value="base">base</option><option value="shoulder">shoulder</option><option value="elbow">elbow</option><option value="wrist_r">wrist_r</option><option value="wrist_p">wrist_p</option></select>
              <label>software abs(deg) <span id="calibAbsVal">90</span></label>
              <input id="calibAbs" type="range" min="0" max="180" value="90" step="1" oninput="document.getElementById('calibAbsVal').textContent=this.value">
              <div class="row"><button class="btn-main" onclick="group('calib_joint',{name:val('calibJoint'),abs:num('calibAbs')})">Apply Calib Joint</button></div>
              <p class="small">추천 offset 계산: software_abs - 90</p>
            </div>
            <div>
              <label>Grip rel(deg) <span id="calibGripVal">0</span></label>
              <input id="calibGrip" type="range" min="-60" max="60" value="0" step="1" oninput="document.getElementById('calibGripVal').textContent=this.value">
              <div class="row"><button class="btn-main" onclick="group('calib_grip_rel',{rel:num('calibGrip')})">Apply Grip Rel</button></div>
              <p class="small">grip은 offset보다 열림/닫힘 방향 관계 확인이 핵심입니다.</p>
            </div>
          </div>
        </section>

        <section id="p2" class="panel">
          <h2>02 Angle Test - Jog / Scan</h2>
          <label>Joint</label>
          <select id="angleJoint"><option value="base">base</option><option value="shoulder">shoulder</option><option value="elbow">elbow</option><option value="wrist_r">wrist_r</option><option value="wrist_p">wrist_p</option><option value="grip">grip</option></select>
          <div class="row">
            <button class="btn-stop" onclick="group('joint',{name:val('angleJoint'),delta:-10})">-10°</button>
            <button class="btn-stop" onclick="group('joint',{name:val('angleJoint'),delta:-5})">-5°</button>
            <button class="btn-ok" onclick="group('joint0',{name:val('angleJoint')})">0°</button>
            <button class="btn-main" onclick="group('joint',{name:val('angleJoint'),delta:5})">+5°</button>
            <button class="btn-main" onclick="group('joint',{name:val('angleJoint'),delta:10})">+10°</button>
          </div>
          <div class="row">
            <button class="btn-special" onclick="group('scan_start',{name:val('angleJoint'),direction:'up'})">Start Up</button>
            <button class="btn-special" onclick="group('scan_start',{name:val('angleJoint'),direction:'down'})">Start Down</button>
            <button class="btn-stop" onclick="group('scan_stop',{})">Stop / Record</button>
            <button class="btn-gray" onclick="group('scan_status',{})">Scan Status</button>
          </div>
        </section>

        <section id="p3" class="panel">
          <h2>03 / 04 XYZ Control</h2>
          <div class="formline">X <input id="x" type="number" value="18" step="1">Y <input id="y" type="number" value="0" step="1">Z <input id="z" type="number" value="18" step="1"></div>
          <div class="formline">v_max <input id="vmax" type="number" value="60" step="10">a_max <input id="amax" type="number" value="120" step="20"></div>
          <div class="row">
            <button class="btn-special" onclick="moveXYZ('linear')">03 Linear</button>
            <button class="btn-main" onclick="moveXYZ('smooth')">04 Smooth</button>
            <button class="btn-gray" onclick="group('live_xyz',xyzPayload())">Live XYZ Apply Once</button>
          </div>
          <div class="controlNote">여러 대를 동시에 움직일 때는 Live Slider 방식보다 <b>값 입력 후 Move 버튼</b> 방식이 안정적입니다.</div>
        </section>

        <section id="p4" class="panel">
          <h2>Gripper Control</h2>
          <div class="row">
            <button class="btn-ok" onclick="group('grip',{v:30})">Open</button>
            <button class="btn-stop" onclick="group('grip',{v:0})">Close</button>
            <button class="btn-gray" onclick="group('grip',{v:0})">Neutral</button>
          </div>
          <label>Custom grip rel(deg)</label>
          <input id="gripCustom" type="number" value="30" step="1">
          <div class="row"><button class="btn-main" onclick="group('grip',{v:num('gripCustom')})">Apply Custom Grip</button></div>
        </section>

        <section id="p5" class="panel">
          <h2>Advanced Direct Endpoint</h2>
          <label>Endpoint</label>
          <select id="directEndpoint"><option value="neutral_raw">neutral_raw</option><option value="home_calibrated">home_calibrated</option><option value="status">status</option><option value="calib_joint">calib_joint</option><option value="calib_grip_rel">calib_grip_rel</option><option value="joint">joint</option><option value="joint0">joint0</option><option value="scan_start">scan_start</option><option value="scan_stop">scan_stop</option><option value="scan_status">scan_status</option><option value="move">move</option><option value="live_xyz">live_xyz</option><option value="grip">grip</option></select>
          <label>JSON Payload</label>
          <textarea id="directPayload" rows="5">{"target":"all"}</textarea>
          <div class="row"><button class="btn-main" onclick="directSend()">Send Direct</button></div>
        </section>
      </div>

      <div class="card">
        <h2>Log</h2>
        <div class="row">
          <button class="btn-gray inlineBtn" onclick="clearLog()">Clear Log</button>
          <button class="btn-main inlineBtn" onclick="loadRobots(true)">Check Robots Now</button>
        </div>
        <pre id="log" class="log">Ready</pre>
      </div>
    </main>
  </div>
</div>

<script>
let robots = {};
let selected = new Set();

function val(id){return document.getElementById(id).value}
function num(id){return Number(document.getElementById(id).value)}
function nowStr(){return new Date().toLocaleTimeString()}
function log(obj){
  const text=typeof obj==='string'?obj:JSON.stringify(obj,null,2);
  document.getElementById('log').textContent=nowStr()+"\n"+text;
}
function clearLog(){document.getElementById('log').textContent='Ready'}

function showPanel(id,btn){
  document.querySelectorAll('.panel').forEach(p=>p.classList.remove('active'));
  document.querySelectorAll('.tab').forEach(t=>t.classList.remove('active'));
  document.getElementById(id).classList.add('active');
  btn.classList.add('active');
}

async function api(path,data={},method='POST'){
  try{
    const opt={method,headers:{'Content-Type':'application/json'}};
    if(method!=='GET') opt.body=JSON.stringify(data);
    const res=await fetch(path,opt);
    return await res.json();
  }catch(e){
    return {ok:false,msg:String(e)};
  }
}

function selectedIds(){
  const boxes=document.querySelectorAll('input.robotCheck:checked');
  selected=new Set(Array.from(boxes).map(b=>b.value));
  renderSelected();
  return Array.from(selected);
}

function renderSelected(){
  const arr=Array.from(selected);
  document.getElementById('selectedBox').textContent=arr.length ? arr.join(', ') : '선택된 로봇팔 없음';
  document.getElementById('selectedCount').textContent=String(arr.length);
}

function selectAll(){Object.keys(robots).forEach(id=>selected.add(id));renderRobotList()}
function clearSelected(){selected.clear();renderRobotList()}

async function loadServerInfo(){
  const res = await api('/api/server_info',{},'GET');
  if(res.ok){
    document.getElementById('serverAddr').textContent = res.host + ':' + res.port;
    document.getElementById('serverBox').textContent =
      '서버 실행 중\\n접속 주소: http://' + res.host + ':' + res.port +
      '\\n로컬 주소: http://127.0.0.1:' + res.port;
  }else{
    document.getElementById('serverBox').textContent = '서버 정보 확인 실패: ' + res.msg;
  }
}

async function loadRobots(showLog=false){
  const res=await api('/api/robots',{},'GET');
  if(!res.ok){
    document.getElementById('robotList').innerHTML='<div class="empty">/api/robots 요청 실패<br>'+res.msg+'</div>';
    log(res);
    return;
  }

  robots=res.robots||{};
  document.getElementById('robotCount').textContent=String(Object.keys(robots).length);

  // 등록된 로봇이 하나뿐이고 아직 선택이 없으면 자동 선택
  if(Object.keys(robots).length===1 && selected.size===0){
    selected.add(Object.keys(robots)[0]);
  }

  renderRobotList();
  if(showLog) log({ok:true,msg:'robot list refreshed',robots});
}

function renderRobotList(){
  const box=document.getElementById('robotList');
  const ids=Object.keys(robots).sort();

  if(ids.length===0){
    box.innerHTML='<div class="empty"><b>등록된 로봇팔이 없습니다.</b><br>1) server_codeplant_multi_control_v2.py 실행 유지<br>2) ESP32에서 06_sta_robot_arm_client.py 재실행<br>3) ESP32 Shell의 서버 응답 200 OK 확인<br>4) 이 화면에서 Refresh</div>';
    selected.clear();
    renderSelected();
    return;
  }

  box.innerHTML='';
  ids.forEach(id=>{
    const r=robots[id];
    const checked=selected.has(id)?'checked':'';
    const ago = r.age_sec == null ? '-' : (Math.round(r.age_sec)+'s ago');
    const div=document.createElement('div');
    div.className='robot';
    div.innerHTML=`
      <div class="robotTop">
        <input class="robotCheck" type="checkbox" value="${id}" ${checked} onchange="selectedIds()">
        <div style="flex:1">
          <h3>${r.name||id}</h3>
          <div class="ip">${id} · ${r.ip}:${r.port}</div>
          <span class="badge">registered</span>
          <span class="badge off">last ${ago}</span>
          <div class="row">
            <button class="btn-ok inlineBtn" onclick="single('${id}','home_calibrated',{})">Home</button>
            <button class="btn-gray inlineBtn" onclick="single('${id}','status',{target:'all'})">Status</button>
            <button class="btn-main inlineBtn" onclick="openDirect('${r.ip}',${r.port})">Open</button>
          </div>
        </div>
      </div>`;
    box.appendChild(div);
  });
  renderSelected();
}

function openDirect(ip,port){
  window.open('http://'+ip+':'+port,'_blank');
}

async function loadRegisterLog(){
  const res=await api('/api/register_log',{},'GET');
  if(!res.ok){document.getElementById('regLog').textContent='등록 로그 요청 실패: '+res.msg;return}
  if(!res.logs.length){document.getElementById('regLog').textContent='등록 로그 없음';return}
  document.getElementById('regLog').textContent=res.logs.map(x=>`${x.time} ${x.robot_id} ${x.ip}:${x.port} ${x.name}`).join('\\n');
}

async function single(robotId,endpoint,payload){
  const res=await api(`/api/command/${robotId}/${endpoint}`,payload);
  log(res);
}

async function group(endpoint,payload){
  const ids=selectedIds();
  if(ids.length===0){
    log('명령을 보낼 로봇팔을 먼저 Robot Selection에서 체크하세요.');
    return;
  }
  const res=await api(`/api/group/${endpoint}`,{robot_ids:ids,payload});
  log(res);
}

function xyzPayload(){return{x:num('x'),y:num('y'),z:num('z'),v_max:num('vmax'),a_max:num('amax')}}
function moveXYZ(mode){const p=xyzPayload();p.mode=mode;group('move',p)}

function directSend(){
  let payload={};
  try{payload=JSON.parse(document.getElementById('directPayload').value||'{}')}
  catch(e){log('JSON Payload 형식 오류: '+e);return}
  group(val('directEndpoint'),payload)
}

loadServerInfo();
loadRobots(true);
loadRegisterLog();
setInterval(()=>loadRobots(false),3000);
setInterval(loadRegisterLog,5000);
</script>
</body>
</html>
"""


def _get_lan_ip():
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        s.close()
        return ip
    except Exception:
        return "127.0.0.1"


def _safe_endpoint(endpoint: str) -> str:
    endpoint = (endpoint or "").strip().strip("/")
    if endpoint not in ALLOWED_ENDPOINTS:
        raise ValueError(f"endpoint not allowed: {endpoint}")
    if not re.fullmatch(r"[a-zA-Z0-9_]+", endpoint):
        raise ValueError(f"invalid endpoint: {endpoint}")
    return endpoint


def send_to_robot(robot_id, endpoint, payload=None, timeout=5):
    try:
        endpoint = _safe_endpoint(endpoint)
    except Exception as e:
        return {"ok": False, "target": robot_id, "msg": str(e)}

    if robot_id not in ROBOTS:
        return {"ok": False, "target": robot_id, "msg": "robot not found"}

    robot = ROBOTS[robot_id]
    payload = payload or {}
    url = f"http://{robot['ip']}:{robot['port']}/{endpoint}"

    try:
        r = requests.post(url, json=payload, timeout=timeout)
        try:
            robot_response = r.json()
        except Exception:
            robot_response = r.text
        return {
            "ok": r.ok,
            "target": robot_id,
            "url": url,
            "status_code": r.status_code,
            "robot_response": robot_response,
        }
    except Exception as e:
        return {"ok": False, "target": robot_id, "url": url, "msg": str(e)}


@app.get("/")
def index():
    return render_template_string(HTML)


@app.get("/api/server_info")
def server_info():
    return jsonify({"ok": True, "host": _get_lan_ip(), "port": 8000, "robots": len(ROBOTS)})


@app.post("/api/register")
def register():
    data = request.get_json() or {}
    robot_id = data.get("robot_id")
    if not robot_id:
        return jsonify({"ok": False, "msg": "robot_id missing"}), 400

    item = {
        "robot_id": robot_id,
        "name": data.get("name", robot_id),
        "ip": data.get("ip", request.remote_addr),
        "port": int(data.get("port", 80)),
        "last_seen": time.time(),
    }
    ROBOTS[robot_id] = item

    log_item = {
        "time": time.strftime("%H:%M:%S"),
        "robot_id": robot_id,
        "name": item["name"],
        "ip": item["ip"],
        "port": item["port"],
    }
    REGISTER_LOG.append(log_item)
    del REGISTER_LOG[:-30]

    print("registered:", item)
    return jsonify({"ok": True, "msg": f"{robot_id} registered", "robot": item})


@app.get("/api/robots")
def robots():
    now = time.time()
    out = {}
    for rid, r in ROBOTS.items():
        item = dict(r)
        item["age_sec"] = now - r.get("last_seen", now)
        out[rid] = item
    return jsonify({"ok": True, "robots": out})


@app.get("/api/register_log")
def register_log():
    return jsonify({"ok": True, "logs": REGISTER_LOG})


@app.post("/api/command/<robot_id>/<endpoint>")
def command(robot_id, endpoint):
    payload = request.get_json() or {}
    return jsonify(send_to_robot(robot_id, endpoint, payload))


@app.post("/api/group/<endpoint>")
def command_group(endpoint):
    data = request.get_json() or {}
    robot_ids = data.get("robot_ids") or []
    payload = data.get("payload") or {}

    if not isinstance(robot_ids, list):
        return jsonify({"ok": False, "msg": "robot_ids must be list"}), 400
    if not robot_ids:
        return jsonify({"ok": False, "msg": "no robots selected"}), 400

    results = {robot_id: send_to_robot(robot_id, endpoint, payload) for robot_id in robot_ids}
    return jsonify({"ok": True, "endpoint": endpoint, "payload": payload, "results": results})


@app.post("/api/all/<endpoint>")
def command_all(endpoint):
    payload = request.get_json() or {}
    results = {robot_id: send_to_robot(robot_id, endpoint, payload) for robot_id in list(ROBOTS.keys())}
    return jsonify({"ok": True, "endpoint": endpoint, "payload": payload, "results": results})


if __name__ == "__main__":
    print("=" * 60)
    print("Codeplant Robot Arm Central Server v2")
    print("Local: http://127.0.0.1:8000")
    print(f"LAN  : http://{_get_lan_ip()}:8000")
    print("=" * 60)
    app.run(host="0.0.0.0", port=8000, debug=True)

