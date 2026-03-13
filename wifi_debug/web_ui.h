#ifndef WEB_UI_H
#define WEB_UI_H

const char PAGE_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>Umbreon</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{font-family:system-ui,-apple-system,sans-serif;background:#0f172a;color:#e2e8f0;font-size:14px}
header{background:#1e293b;padding:10px 14px;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:6px;border-bottom:1px solid #334155}
.logo{font-size:18px;font-weight:700;letter-spacing:1px;background:linear-gradient(135deg,#f59e0b,#ef4444);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.hdr-r{display:flex;align-items:center;gap:10px;font-size:12px}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block}
.dot.on{background:#22c55e}.dot.off{background:#ef4444}
.badge{font-size:11px;font-weight:700;padding:2px 8px;border-radius:4px}
.badge.run{background:#15803d}.badge.stop{background:#b91c1c}
#frames{color:#64748b}
section{margin:8px;padding:12px;background:#1e293b;border-radius:8px;border:1px solid #334155}
h2{font-size:13px;color:#94a3b8;margin-bottom:8px;cursor:pointer;user-select:none}
h2::before{content:'\25b8 '}
h2.open::before{content:'\25be '}
.grid4{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.sbox{background:#0f172a;padding:8px;border-radius:6px;text-align:center}
.sbox .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.sbox .l{font-size:10px;color:#64748b;margin-top:2px}
.info-row{display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap;font-variant-numeric:tabular-nums}
.info-row .l{color:#64748b;font-size:12px}.info-row .v{font-weight:600;font-size:13px;min-width:50px}
.btns{display:flex;gap:6px;flex-wrap:wrap}
button{background:#334155;color:#e2e8f0;border:none;padding:10px 16px;border-radius:6px;font-size:13px;cursor:pointer;touch-action:manipulation;-webkit-tap-highlight-color:transparent}
button:active{background:#475569}
.b-go{background:#15803d;font-weight:700}.b-go:active{background:#166534}
.b-no{background:#b91c1c;font-weight:700}.b-no:active{background:#991b1b}
.b-bl{background:#1d4ed8}.b-bl:active{background:#1e40af}
.params{display:grid;grid-template-columns:1fr 1fr;gap:4px 10px;margin:8px 0}
.pm{display:flex;align-items:center;justify-content:space-between;gap:4px}
.pm label{font-size:11px;color:#94a3b8;flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.pm input{width:68px;background:#0f172a;color:#e2e8f0;border:1px solid #334155;border-radius:4px;padding:4px;text-align:right;font-size:12px;font-variant-numeric:tabular-nums}
.pm input:focus{border-color:#3b82f6;outline:none}
.pm input:disabled{color:#64748b}
.tbtns{display:grid;grid-template-columns:repeat(auto-fill,minmax(65px,1fr));gap:4px}
.tbtns button{padding:8px 4px;font-size:11px}
#log{background:#020617;color:#a3e635;font-family:monospace;font-size:11px;padding:6px;border-radius:4px;max-height:130px;overflow-y:auto;margin-top:6px;white-space:pre-wrap;word-break:break-all;min-height:24px}
.drv{display:flex;flex-direction:column;gap:10px}
.sr{display:flex;align-items:center;gap:8px}
.sr label{min-width:48px;font-size:12px;color:#94a3b8}
.sr input[type=range]{flex:1;accent-color:#3b82f6}
.sr .sv{min-width:48px;text-align:right;font-size:12px;font-variant-numeric:tabular-nums}
.toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);padding:8px 18px;border-radius:6px;font-size:13px;z-index:99;opacity:0;transition:opacity .2s}
.toast.show{opacity:1}
.toast.ok{background:#15803d}.toast.err{background:#b91c1c}.toast.info{background:#1d4ed8}
.hidden{display:none}
</style>
</head>
<body>

<header>
  <span class="logo">UMBREON</span>
  <div class="hdr-r">
    <span class="dot off" id="wsDot"></span>
    <span id="wsText">Connecting</span>
    <span class="badge stop" id="runBadge">STOP</span>
    <span id="frames">#0</span>
  </div>
</header>

<!-- Telemetry -->
<section>
  <div class="grid4">
    <div class="sbox"><div class="v" id="s0">&mdash;</div><div class="l">Left (cm)</div></div>
    <div class="sbox"><div class="v" id="s1">&mdash;</div><div class="l">Front-Left</div></div>
    <div class="sbox"><div class="v" id="s2">&mdash;</div><div class="l">Front-Right</div></div>
    <div class="sbox"><div class="v" id="s3">&mdash;</div><div class="l">Right</div></div>
  </div>
  <div class="info-row">
    <span class="l">Speed</span><span class="v" id="speed">&mdash;</span>
    <span class="l">Target</span><span class="v" id="target">&mdash;</span>
    <span class="l">Steer</span><span class="v" id="steer">&mdash;</span>
  </div>
  <div class="info-row hidden" id="imuRow">
    <span class="l">Yaw</span><span class="v" id="yaw">&mdash;</span>
    <span class="l">Heading</span><span class="v" id="heading">&mdash;</span>
  </div>
</section>

<!-- Controls -->
<section>
  <div class="btns">
    <button class="b-go" onclick="send('$START')">&#9654; START</button>
    <button class="b-no" onclick="send('$STOP')">&#9632; STOP</button>
    <button class="b-bl" onclick="send('$PING')">PING</button>
    <button onclick="send('$STATUS')">STATUS</button>
  </div>
</section>

<!-- Settings -->
<section>
  <h2 id="settH" onclick="tog('sett')">Settings</h2>
  <div id="sett" class="hidden">
    <div class="btns">
      <button onclick="send('$GET')">Read</button>
      <button onclick="writeP()">Write</button>
      <button onclick="send('$SAVE')">Save EE</button>
      <button onclick="send('$LOAD');setTimeout(function(){send('$GET')},300)">Load EE</button>
      <button onclick="send('$RST');setTimeout(function(){send('$GET')},300)">Reset</button>
    </div>
    <div class="params" id="params"><div style="color:#64748b;font-size:12px;grid-column:1/-1;padding:8px 0">Click Read to load settings</div></div>
  </div>
</section>

<!-- Tests -->
<section>
  <h2 id="testH" onclick="tog('test')">Hardware Tests</h2>
  <div id="test" class="hidden">
    <div class="tbtns">
      <button onclick="send('$TEST:lidar')">LiDAR</button>
      <button onclick="send('$TEST:servo')">Servo</button>
      <button onclick="send('$TEST:taho')">Tacho</button>
      <button onclick="mtest('esc')">ESC</button>
      <button onclick="mtest('speed')">Speed</button>
      <button onclick="mtest('autotune')">Tune</button>
      <button onclick="send('$TEST:reactive')">React</button>
    </div>
    <div class="btns" style="margin-top:4px">
      <button class="b-no" onclick="send('$STOP')">Abort</button>
      <button onclick="$('log').textContent=''">Clear</button>
    </div>
    <div id="log"></div>
  </div>
</section>

<!-- Manual Drive -->
<section>
  <h2 id="drvH" onclick="tog('drv')">Manual Drive</h2>
  <div id="drv" class="hidden">
    <div class="drv">
      <div class="sr">
        <label>Steer</label>
        <input type="range" id="dSteer" min="-1000" max="1000" step="50" value="0">
        <span class="sv" id="dSteerV">0</span>
      </div>
      <div class="sr">
        <label>Speed</label>
        <input type="range" id="dSpeed" min="0" max="3.0" step="0.1" value="0">
        <span class="sv" id="dSpeedV">0.0</span>
      </div>
      <div class="btns">
        <button class="b-go" onclick="drvStart()">Drive</button>
        <button class="b-no" onclick="drvStop()">Release</button>
        <button onclick="drvCenter()">Center</button>
      </div>
      <div style="font-size:11px;color:#64748b">Car must be started first. Release stops sending commands.</div>
    </div>
  </div>
</section>

<div class="toast" id="toast"></div>

<script>
var ws,fc=0,dt=null;
function $(id){return document.getElementById(id)}

// --- WebSocket ---
function conn(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){$('wsDot').className='dot on';$('wsText').textContent='Connected'};
  ws.onclose=function(){$('wsDot').className='dot off';$('wsText').textContent='Reconnecting';setTimeout(conn,2000)};
  ws.onmessage=function(e){proc(e.data)};
}
function send(c){if(ws&&ws.readyState===1)ws.send(c)}

// --- Protocol ---
function proc(line){
  line=line.trim();
  if(!line||line.charAt(0)==='#')return;
  if(line.charAt(0)==='$'){
    if(line==='$PONG')toast('PONG','ok');
    else if(line==='$ACK')toast('ACK','ok');
    else if(line.indexOf('$NAK:')===0)toast('NAK: '+line.slice(5),'err');
    else if(line.indexOf('$CFG:')===0)parseCfg(line.slice(5));
    else if(line==='$STS:RUN')setRun(true);
    else if(line==='$STS:STOP')setRun(false);
    else if(line.indexOf('$T:')===0)addLog(line);
    else if(line.indexOf('$TR:')===0)addLog(line);
    else if(line.indexOf('$TDONE:')===0){addLog(line);toast('Test done','info')}
  }else{
    var p=line.split(',');
    if(p.length>=8){
      fc++;
      $('frames').textContent='#'+fc;
      $('s0').textContent=(parseInt(p[1])/10).toFixed(1);
      $('s1').textContent=(parseInt(p[2])/10).toFixed(1);
      $('s2').textContent=(parseInt(p[3])/10).toFixed(1);
      $('s3').textContent=(parseInt(p[4])/10).toFixed(1);
      $('steer').textContent=p[5];
      $('speed').textContent=parseFloat(p[6]).toFixed(2)+' m/s';
      $('target').textContent=parseFloat(p[7]).toFixed(1)+' m/s';
      if(p.length>=10){
        $('imuRow').classList.remove('hidden');
        $('yaw').textContent=parseFloat(p[8]).toFixed(1)+' \u00b0/s';
        $('heading').textContent=parseFloat(p[9]).toFixed(1)+' \u00b0';
      }
    }
  }
}

// --- Settings ---
var LABELS={FOD:'Front Obstacle',SOD:'Side Open',ACD:'All Close',CFD:'Close Front',
KP:'PID Kp',KI:'PID Ki',KD:'PID Kd',MSP:'Min Speed',XSP:'Max Speed',BSP:'Min Reverse',
MNP:'Min Steer',XNP:'Max Steer',NTP:'Neutral',ENH:'Encoder Holes',WDM:'Wheel Diam',
LMS:'Loop ms',SPD1:'Spd Clear',SPD2:'Spd Blocked',COE1:'Coef Clear',COE2:'Coef Blocked',
WDD:'Wrong Dir',RCW:'Race CW',STK:'Stuck Thresh',IMU:'IMU',DBG:'Debug'};
var FLOATS={KP:1,KI:1,KD:1,WDM:1,SPD1:1,SPD2:1,COE1:1,COE2:1,WDD:1};
var RO={IMU:1,DBG:1};

function parseCfg(cfg){
  var el=$('params');el.innerHTML='';
  var pairs=cfg.split(',');
  for(var i=0;i<pairs.length;i++){
    var kv=pairs[i].split('=');if(kv.length<2)continue;
    var k=kv[0],v=kv[1];
    var d=document.createElement('div');d.className='pm';
    var lb=document.createElement('label');lb.textContent=LABELS[k]||k;lb.title=k;
    var inp=document.createElement('input');inp.type='number';inp.id='p_'+k;inp.value=v;
    inp.step=FLOATS[k]?'0.01':'1';
    if(RO[k])inp.disabled=true;
    d.appendChild(lb);d.appendChild(inp);el.appendChild(d);
  }
  toast('Config loaded','info');
}

function writeP(){
  var inputs=$('params').querySelectorAll('input:not([disabled])');
  var a=[];
  for(var i=0;i<inputs.length;i++){
    var k=inputs[i].id.slice(2);
    a.push(k+'='+inputs[i].value);
  }
  if(a.length)send('$SET:'+a.join(','));
}

// --- Tests ---
function mtest(n){if(confirm('Motor will spin! Continue?'))send('$TEST:'+n)}
function addLog(line){var el=$('log');el.textContent+=line+'\n';el.scrollTop=el.scrollHeight}

// --- Run state ---
function setRun(r){
  var b=$('runBadge');
  b.textContent=r?'RUN':'STOP';
  b.className='badge '+(r?'run':'stop');
}

// --- Collapsible ---
function tog(id){$(id).classList.toggle('hidden');$(id+'H').classList.toggle('open')}

// --- Manual drive ---
$('dSteer').oninput=function(){$('dSteerV').textContent=this.value};
$('dSpeed').oninput=function(){$('dSpeedV').textContent=parseFloat(this.value).toFixed(1)};

function drvStart(){
  if(dt)return;
  dt=setInterval(function(){send('$DRV:'+$('dSteer').value+','+$('dSpeed').value)},200);
}
function drvStop(){
  if(dt){clearInterval(dt);dt=null}
  send('$DRV:0,0');
  drvCenter();
}
function drvCenter(){
  $('dSteer').value=0;$('dSteerV').textContent='0';
  $('dSpeed').value=0;$('dSpeedV').textContent='0.0';
}

// --- Toast ---
var tt=null;
function toast(msg,type){
  var el=$('toast');
  el.textContent=msg;
  el.className='toast '+(type||'info')+' show';
  if(tt)clearTimeout(tt);
  tt=setTimeout(function(){el.classList.remove('show')},2000);
}

conn();
</script>
</body>
</html>)rawliteral";

#endif
