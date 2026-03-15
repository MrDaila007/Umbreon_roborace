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
body{font-family:system-ui,sans-serif;background:#0f172a;color:#e2e8f0;font-size:14px}
header{background:#1e293b;padding:10px 14px;display:flex;align-items:center;justify-content:space-between;flex-wrap:wrap;gap:6px;border-bottom:1px solid #334155}
.logo{font-size:18px;font-weight:700;letter-spacing:1px;background:linear-gradient(135deg,#f59e0b,#ef4444);-webkit-background-clip:text;-webkit-text-fill-color:transparent}
.hr{display:flex;align-items:center;gap:10px;font-size:12px}
.dot{width:10px;height:10px;border-radius:50%;display:inline-block}
.on{background:#22c55e}.off{background:#ef4444}
.bg{font-size:11px;font-weight:700;padding:2px 8px;border-radius:4px}
.run{background:#15803d}.stp{background:#b91c1c}
section{margin:8px;padding:12px;background:#1e293b;border-radius:8px;border:1px solid #334155}
h2{font-size:13px;color:#94a3b8;margin-bottom:8px;cursor:pointer;user-select:none}
h2::before{content:'\25b8 '}h2.open::before{content:'\25be '}
.g4{display:grid;grid-template-columns:1fr 1fr;gap:6px}
.sb{background:#0f172a;padding:8px;border-radius:6px;text-align:center}
.sb .v{font-size:22px;font-weight:700;font-variant-numeric:tabular-nums}
.sb .l{font-size:10px;color:#64748b;margin-top:2px}
.ir{display:flex;gap:8px;align-items:center;margin-top:8px;flex-wrap:wrap;font-variant-numeric:tabular-nums}
.ir .l{color:#64748b;font-size:12px}.ir .v{font-weight:600;font-size:13px;min-width:50px}
.bt{display:flex;gap:6px;flex-wrap:wrap}
button{background:#334155;color:#e2e8f0;border:none;padding:10px 16px;border-radius:6px;font-size:13px;cursor:pointer;touch-action:manipulation}
button:active{background:#475569}
.bg0{background:#15803d;font-weight:700}.bg0:active{background:#166534}
.bn{background:#b91c1c;font-weight:700}.bn:active{background:#991b1b}
.bb{background:#1d4ed8}.bb:active{background:#1e40af}
.pm{display:grid;grid-template-columns:1fr 1fr;gap:4px 10px;margin:8px 0}
.pi{display:flex;align-items:center;justify-content:space-between;gap:4px}
.pi label{font-size:11px;color:#94a3b8;flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.pi input{width:68px;background:#0f172a;color:#e2e8f0;border:1px solid #334155;border-radius:4px;padding:4px;text-align:right;font-size:12px;font-variant-numeric:tabular-nums}
.pi input:focus{border-color:#3b82f6;outline:none}
.pi input:disabled{color:#64748b}
.tb{display:grid;grid-template-columns:repeat(auto-fill,minmax(65px,1fr));gap:4px}
.tb button{padding:8px 4px;font-size:11px}
#log{background:#020617;color:#a3e635;font-family:monospace;font-size:11px;padding:6px;border-radius:4px;max-height:130px;overflow-y:auto;margin-top:6px;white-space:pre-wrap;word-break:break-all;min-height:24px}
.sr{display:flex;align-items:center;gap:8px}
.sr label{min-width:48px;font-size:12px;color:#94a3b8}
.sr input[type=range]{flex:1;accent-color:#3b82f6}
.sr .sv{min-width:48px;text-align:right;font-size:12px;font-variant-numeric:tabular-nums}
.toast{position:fixed;bottom:16px;left:50%;transform:translateX(-50%);padding:8px 18px;border-radius:6px;font-size:13px;z-index:99;opacity:0;transition:opacity .2s}
.toast.show{opacity:1}
.toast.ok{background:#15803d}.toast.err{background:#b91c1c}.toast.inf{background:#1d4ed8}
.hid{display:none}
.cb{font-size:18px;min-width:44px}
.fs{font-size:11px;color:#64748b}
</style>
</head>
<body>

<header>
<span class="logo">UMBREON</span>
<div class="hr">
<span class="dot off" id="D"></span>
<span id="W">Connecting</span>
<span class="bg stp" id="R">STOP</span>
<span style="color:#64748b" id="F">#0</span>
</div>
</header>

<section>
<div class="g4">
<div class="sb"><div class="v" id="s0">&mdash;</div><div class="l">Left (cm)</div></div>
<div class="sb"><div class="v" id="s1">&mdash;</div><div class="l">Front-Left</div></div>
<div class="sb"><div class="v" id="s2">&mdash;</div><div class="l">Front-Right</div></div>
<div class="sb"><div class="v" id="s3">&mdash;</div><div class="l">Right</div></div>
</div>
<div class="ir">
<span class="l">Speed</span><span class="v" id="sp">&mdash;</span>
<span class="l">Target</span><span class="v" id="tg">&mdash;</span>
<span class="l">Steer</span><span class="v" id="st">&mdash;</span>
</div>
<div class="ir hid" id="iR">
<span class="l">Yaw</span><span class="v" id="yw">&mdash;</span>
<span class="l">Heading</span><span class="v" id="hd">&mdash;</span>
</div>
</section>

<section>
<h2 id="mapH" class="open" onclick="tog('map')">Track Map</h2>
<div id="map">
<canvas id="mC" style="width:100%;height:220px;background:#020617;border-radius:4px;touch-action:none"></canvas>
<div style="display:flex;gap:6px;margin-top:4px;align-items:center;flex-wrap:wrap">
<button onclick="mR()" style="padding:6px 10px;font-size:11px">Reset</button>
<button onclick="mZ(1.4)" style="padding:6px 10px;font-size:11px">+</button>
<button onclick="mZ(.7)" style="padding:6px 10px;font-size:11px">&minus;</button>
<label class="fs" style="display:flex;align-items:center;gap:4px;margin-left:auto">
<input type="checkbox" id="mF" checked> Follow
</label>
<span class="fs" id="mP">0, 0</span>
</div>
</div>
</section>

<section>
<div class="bt">
<button class="bg0" onclick="S('$START')">&#9654; START</button>
<button class="bn" onclick="S('$STOP')">&#9632; STOP</button>
<button class="bb" onclick="S('$PING')">PING</button>
<button onclick="S('$STATUS')">STATUS</button>
</div>
</section>

<section>
<h2 id="settH" onclick="tog('sett')">Settings</h2>
<div id="sett" class="hid">
<div class="bt">
<button onclick="S('$GET')">Read</button>
<button onclick="wP()">Write</button>
<button onclick="S('$SAVE')">Save EE</button>
<button onclick="S('$LOAD');setTimeout(function(){S('$GET')},300)">Load EE</button>
<button onclick="S('$RST');setTimeout(function(){S('$GET')},300)">Reset</button>
</div>
<div class="pm" id="P"><div class="fs" style="grid-column:1/-1;padding:8px 0">Click Read to load settings</div></div>
</div>
</section>

<section>
<h2 id="testH" onclick="tog('test')">Hardware Tests</h2>
<div id="test" class="hid">
<div class="tb">
<button onclick="S('$TEST:lidar')">LiDAR</button>
<button onclick="S('$TEST:servo')">Servo</button>
<button onclick="S('$TEST:taho')">Tacho</button>
<button onclick="mt('esc')">ESC</button>
<button onclick="mt('speed')">Speed</button>
<button onclick="mt('autotune')">Tune</button>
<button onclick="S('$TEST:reactive')">React</button>
<button onclick="mt('cal')">Calibrate</button>
</div>
<div class="bt" style="margin-top:4px">
<button class="bn" onclick="S('$STOP')">Abort</button>
<button onclick="Q('log').textContent=''">Clear</button>
</div>
<div id="log"></div>
</div>
</section>

<section>
<h2 id="drvH" onclick="tog('drv')">Manual Drive</h2>
<div id="drv" class="hid">
<div style="display:flex;flex-direction:column;gap:10px">
<div style="display:flex;align-items:center;gap:8px;margin-bottom:6px">
<label style="display:flex;align-items:center;gap:6px;font-size:13px;cursor:pointer">
<input type="checkbox" id="dE" onchange="dT(this.checked)">
<span id="dL" style="color:#ef4444;font-weight:600">LOCKED</span>
</label>
</div>
<div class="sr"><label>Steer</label><input type="range" id="dS" min="-1000" max="1000" step="50" value="0" disabled><span class="sv" id="dSV">0</span></div>
<div class="sr"><label>Speed</label><input type="range" id="dV" min="0" max="3.0" step="0.1" value="0" disabled><span class="sv" id="dVV">0.0</span></div>
<div class="bt">
<button class="bg0" id="dG" onclick="dO()" disabled>Drive</button>
<button class="bn" onclick="dX()">Release</button>
<button onclick="dC()">Center</button>
</div>
<div class="fs">Enable checkbox to unlock. Release stops motors.</div>
</div>
</div>
</section>

<section>
<h2 id="calH" onclick="tog('cal')">Servo Calibration</h2>
<div id="cal" class="hid">
<div id="cI">
<div class="ir" style="margin-bottom:8px">
<span class="l">Min</span><span class="v" id="cMn">40</span>
<span class="l">Neutral</span><span class="v" id="cNt">90</span>
<span class="l">Max</span><span class="v" id="cMx">140</span>
</div>
<div class="bt"><button class="bb" onclick="cS()">Calibrate Servo</button></div>
</div>
<div id="cW" class="hid">
<div style="font-size:13px;font-weight:600;margin-bottom:6px" id="cT">Step 1: Set MIN position</div>
<div style="font-size:24px;font-weight:700;text-align:center;margin:8px 0;font-variant-numeric:tabular-nums" id="cA">40&deg;</div>
<div class="bt" style="justify-content:center;margin-bottom:8px">
<button onclick="cJ(-10)" class="cb">&laquo;</button>
<button onclick="cJ(-1)" class="cb">&minus;</button>
<button onclick="cJ(1)" class="cb">+</button>
<button onclick="cJ(10)" class="cb">&raquo;</button>
</div>
<div class="bt">
<button class="bg0" onclick="cN()" id="cO">OK &rarr;</button>
<button class="bn" onclick="cX()">Cancel</button>
</div>
</div>
<div id="cV" class="hid">
<div style="font-size:13px;font-weight:600;margin-bottom:6px">Verify positions:</div>
<div class="ir" style="margin-bottom:8px">
<span class="l">Min</span><span class="v" id="vMn">40</span>
<span class="l">Neutral</span><span class="v" id="vNt">90</span>
<span class="l">Max</span><span class="v" id="vMx">140</span>
</div>
<div class="bt" style="margin-bottom:6px">
<button onclick="cP('min')">Test Min</button>
<button onclick="cP('ntp')">Test Neutral</button>
<button onclick="cP('max')">Test Max</button>
</div>
<div class="bt">
<button class="bg0" onclick="cSv()">Save</button>
<button class="bb" onclick="cS()">Redo</button>
<button class="bn" onclick="cX()">Cancel</button>
</div>
</div>
</div>
</section>

<section>
<h2 id="escH" onclick="tog('esc')">ESC Min Speed</h2>
<div id="esc" class="hid">
<div class="sr"><label>ESC</label><input type="range" id="eS" min="1500" max="1700" step="1" value="1540"><span class="sv" id="eV">1540</span></div>
<div class="ir" style="margin:8px 0"><span class="l">Current MSP</span><span class="v" id="eM">1540</span></div>
<div class="bt">
<button class="bg0" onclick="eA()">Apply &amp; Save</button>
<button class="bn" onclick="eX()">Stop Motor</button>
</div>
<div class="fs" style="margin-top:6px">Slide until wheels just start spinning. Apply to set as MSP.</div>
</div>
</section>

<div class="toast" id="T"></div>

<script>
var ws,fc=0,di=null,lc='';
function Q(id){return document.getElementById(id)}

function cn(){
ws=new WebSocket('ws://'+location.hostname+':81/');
ws.onopen=function(){Q('D').className='dot on';Q('W').textContent='Connected'};
ws.onclose=function(){Q('D').className='dot off';Q('W').textContent='Reconnecting';setTimeout(cn,2000)};
ws.onmessage=function(e){pr(e.data)};
}
function S(c){if(ws&&ws.readyState===1){ws.send(c);lc=c}}

// --- Protocol ---
function pr(l){
l=l.trim();if(!l||l[0]==='#')return;
if(l[0]==='$'){
if(l==='$PONG')tt('PONG','ok');
else if(l==='$ACK'){
if(lc.indexOf('$SET:')===0)tt('Write OK','ok');
else if(lc==='$SAVE')tt('Save EE OK','ok');
else tt('ACK','ok');
}
else if(l.indexOf('$NAK:')===0){
var r=l.slice(5);
if(lc.indexOf('$SET:')===0)tt('Write fail: '+r,'err');
else tt('NAK: '+r,'err');
}
else if(l.indexOf('$CFG:')===0)pC(l.slice(5));
else if(l==='$STS:RUN')sR(1);
else if(l==='$STS:STOP')sR(0);
else if(l.indexOf('$T:')===0)aL(l);
else if(l.indexOf('$TR:')===0)aL(l);
else if(l.indexOf('$TDONE:')===0){aL(l);tt('Test done','inf')}
}else{
var p=l.split(',');
if(p.length>=8){
fc++;
var s0=parseInt(p[1]),s1=parseInt(p[2]),s2=parseInt(p[3]),s3=parseInt(p[4]);
Q('F').textContent='#'+fc;
Q('s0').textContent=(s0/10).toFixed(1);
Q('s1').textContent=(s1/10).toFixed(1);
Q('s2').textContent=(s2/10).toFixed(1);
Q('s3').textContent=(s3/10).toFixed(1);
Q('st').textContent=p[5];
Q('sp').textContent=parseFloat(p[6]).toFixed(2)+' m/s';
Q('tg').textContent=parseFloat(p[7]).toFixed(1)+' m/s';
var hi=p.length>=10,yw=0;
if(hi){yw=parseFloat(p[8]);Q('iR').classList.remove('hid');Q('yw').textContent=yw.toFixed(1)+'\u00b0/s';Q('hd').textContent=parseFloat(p[9]).toFixed(1)+'\u00b0'}
mU(parseInt(p[0]),[s0,s1,s2,s3],parseInt(p[5]),parseFloat(p[6]),yw,hi);
}}}

// --- Settings ---
var LB={FOD:'Front Obstacle',SOD:'Side Open',ACD:'All Close',CFD:'Close Front',
KP:'PID Kp',KI:'PID Ki',KD:'PID Kd',MSP:'Min Spd \u00b5s',XSP:'Max Spd \u00b5s',BSP:'Min Rev \u00b5s',
MNP:'Min Steer',XNP:'Max Steer',NTP:'Neutral',ENH:'Enc Holes',WDM:'Wheel Diam',
LMS:'Loop ms',SPD1:'Spd Clear',SPD2:'Spd Block',COE1:'Coef Clear',COE2:'Coef Block',
WDD:'Wrong Dir',RCW:'Race CW',STK:'Stuck Thr',IMR:'IMU Rot',SVR:'Srv Rev',CAL:'Calibrated',IMU:'IMU',DBG:'Debug'};
var FL={KP:1,KI:1,KD:1,WDM:1,SPD1:1,SPD2:1,COE1:1,COE2:1,WDD:1};
var RO={IMU:1,DBG:1},OR={};

function pC(cfg){
var el=Q('P');el.innerHTML='';OR={};
var ps=cfg.split(','),i,kv,k,v,d,lb,inp;
for(i=0;i<ps.length;i++){
kv=ps[i].split('=');if(kv.length<2)continue;
k=kv[0];v=kv[1];OR[k]=v;
d=document.createElement('div');d.className='pi';
lb=document.createElement('label');lb.textContent=LB[k]||k;lb.title=k;
inp=document.createElement('input');inp.type='number';inp.id='p_'+k;inp.value=v;
inp.step=FL[k]?'0.01':'1';
if(RO[k])inp.disabled=true;
d.appendChild(lb);d.appendChild(inp);el.appendChild(d);
}
var mnp=Q('p_MNP'),xnp=Q('p_XNP'),ntp=Q('p_NTP');
if(mnp){cm=parseInt(mnp.value);Q('cMn').textContent=cm}
if(xnp){cx=parseInt(xnp.value);Q('cMx').textContent=cx}
if(ntp){cn2=parseInt(ntp.value);Q('cNt').textContent=cn2}
var msp=Q('p_MSP');
if(msp){Q('eM').textContent=msp.value;Q('eS').value=msp.value;Q('eV').textContent=msp.value}
tt('Config loaded','inf');
}

function wP(){
var inp=Q('P').querySelectorAll('input:not([disabled])'),a=[],i,k,nv;
for(i=0;i<inp.length;i++){
k=inp[i].id.slice(2);
if(!k||k.length<2||!LB[k])continue;
nv=inp[i].value;
if(OR.hasOwnProperty(k)){if(FL[k]?parseFloat(OR[k])===parseFloat(nv):OR[k]===nv)continue;}
a.push(k+'='+nv);
}
if(a.length){S('$SET:'+a.join(','));tt('Writing '+a.length+' params','inf')}
else tt('No changes','err');
}

function mt(n){if(confirm('Motor will spin! OK?'))S('$TEST:'+n)}
function aL(l){var e=Q('log');e.textContent+=l+'\n';e.scrollTop=e.scrollHeight}
function sR(r){var b=Q('R');b.textContent=r?'RUN':'STOP';b.className='bg '+(r?'run':'stp')}
function tog(id){Q(id).classList.toggle('hid');Q(id+'H').classList.toggle('open')}

// --- Manual drive ---
Q('dS').oninput=function(){Q('dSV').textContent=this.value};
Q('dV').oninput=function(){Q('dVV').textContent=parseFloat(this.value).toFixed(1)};
function dT(en){
S(en?'$DRVEN':'$DRVOFF');
Q('dS').disabled=!en;Q('dV').disabled=!en;Q('dG').disabled=!en;
Q('dL').textContent=en?'ENABLED':'LOCKED';
Q('dL').style.color=en?'#22c55e':'#ef4444';
if(!en)dX();
}
function dO(){if(di||!Q('dE').checked)return;di=setInterval(function(){S('$DRV:'+Q('dS').value+','+Q('dV').value)},200)}
function dX(){if(di){clearInterval(di);di=null}S('$DRV:0,0');dC()}
function dC(){Q('dS').value=0;Q('dSV').textContent='0';Q('dV').value=0;Q('dVV').textContent='0.0'}

// --- Servo Cal ---
var cm=40,cn2=90,cx=140,ca=90,cs=0;
var CT=['Step 1: Set MIN position','Step 2: Set MAX position','Step 3: Set NEUTRAL position'];
function cSP(p){Q('cI').className=p==='idle'?'':'hid';Q('cW').className=p==='wiz'?'':'hid';Q('cV').className=p==='verify'?'':'hid'}
function cM(a){ca=Math.max(0,Math.min(180,a));Q('cA').innerHTML=ca+'&deg;';S('$SRV:'+ca)}
function cJ(d){cM(ca+d)}
function cS(){cs=0;ca=cm;cSP('wiz');Q('cT').textContent=CT[0];Q('cO').innerHTML='OK &rarr;';cM(ca)}
function cN(){
if(cs===0){cm=ca;cs=1;ca=cx;Q('cT').textContent=CT[1];cM(ca)}
else if(cs===1){cx=ca;cs=2;ca=cn2;Q('cT').textContent=CT[2];Q('cO').innerHTML='OK &#10003;';cM(ca)}
else{cn2=ca;Q('vMn').textContent=cm;Q('vNt').textContent=cn2;Q('vMx').textContent=cx;cSP('verify')}
}
function cP(w){cM(w==='min'?cm:w==='max'?cx:cn2)}
function cSv(){
S('$SET:MNP='+cm+',NTP='+cn2+',XNP='+cx);S('$SAVE');cM(cn2);
Q('cMn').textContent=cm;Q('cNt').textContent=cn2;Q('cMx').textContent=cx;
cSP('idle');tt('Servo cal saved','ok');
}
function cX(){cM(cn2);cSP('idle')}

// --- ESC Min Speed ---
Q('eS').oninput=function(){Q('eV').textContent=this.value;S('$ESC:'+this.value)};
function eA(){var v=Q('eS').value;S('$ESC:1500');S('$SET:MSP='+v);S('$SAVE');Q('eM').textContent=v;tt('MSP='+v+' saved','ok')}
function eX(){S('$ESC:1500');Q('eS').value=1500;Q('eV').textContent='1500'}

// --- Track Map ---
var mx=0,my=0,mh=0,mp=0,ms=150,mo=0,mn=0;
var tr=[],wl=[[],[],[],[]],md=false;
var SD=[45,0,0,-45],SL=[.09,.04,-.04,-.09],SF=.253,WB=.173,MX=28*Math.PI/180;
var SC=['#2ca02c','#1f77b4','#ff7f0e','#d62728'];

function mU(t,s,st,sp,yw,hi){
if(mp===0){mp=t;return}
var dt=(t-mp)/1000;mp=t;
if(dt<=0||dt>1)return;
if(hi){mh+=yw*Math.PI/180*dt}
else{var sa=st/1000*MX;if(Math.abs(sa)>.001)mh+=sp*dt/(WB/Math.tan(sa))}
mx+=sp*dt*Math.cos(mh);my+=sp*dt*Math.sin(mh);
tr.push([mx,my]);if(tr.length>600)tr.shift();
var ch=Math.cos(mh),sh=Math.sin(mh),i,dm,sx,sy,ra;
for(i=0;i<4;i++){
dm=s[i]/10000;if(dm<=0||dm>=8)continue;
sx=mx+SF*ch-SL[i]*sh;sy=my+SF*sh+SL[i]*ch;
ra=mh+SD[i]*Math.PI/180;
wl[i].push([sx+dm*Math.cos(ra),sy+dm*Math.sin(ra)]);
if(wl[i].length>400)wl[i].shift();
}
md=true;
}

function mD(){
var c=Q('mC');if(!c)return;
var w=c.clientWidth,h=c.clientHeight;if(w<2)return;
c.width=w;c.height=h;
var ctx=c.getContext('2d'),ox=mo,oy=mn,i,j,p,p0,pl;
ctx.fillStyle='#020617';ctx.fillRect(0,0,w,h);
if(Q('mF').checked&&tr.length>0){ox=-(mx*ms);oy=my*ms}
function tx(a,b){return[w/2+a*ms+ox,h/2-b*ms+oy]}
ctx.strokeStyle='#1e293b';ctx.lineWidth=1;
var gs=ms<50?1:ms<150?.5:.2;
var x0=(-w/2-ox)/ms,x1=(w/2-ox)/ms,y0=(-h/2+oy)/ms,y1=(h/2+oy)/ms;
for(var gx=Math.floor(x0/gs)*gs;gx<=x1;gx+=gs){p=tx(gx,0);ctx.beginPath();ctx.moveTo(p[0],0);ctx.lineTo(p[0],h);ctx.stroke()}
for(var gy=Math.floor(y0/gs)*gs;gy<=y1;gy+=gs){p=tx(0,gy);ctx.beginPath();ctx.moveTo(0,p[1]);ctx.lineTo(w,p[1]);ctx.stroke()}
for(i=0;i<4;i++){ctx.fillStyle=SC[i];for(j=0;j<wl[i].length;j++){p=tx(wl[i][j][0],wl[i][j][1]);ctx.fillRect(p[0]-1.5,p[1]-1.5,3,3)}}
if(tr.length>1){
ctx.strokeStyle='#eab308';ctx.lineWidth=1.5;ctx.beginPath();
p0=tx(tr[0][0],tr[0][1]);ctx.moveTo(p0[0],p0[1]);
var step=Math.max(1,Math.floor(tr.length/300));
for(j=step;j<tr.length;j+=step){p=tx(tr[j][0],tr[j][1]);ctx.lineTo(p[0],p[1])}
pl=tx(tr[tr.length-1][0],tr[tr.length-1][1]);ctx.lineTo(pl[0],pl[1]);ctx.stroke();
}
if(tr.length>0){
var cp=tx(mx,my);
ctx.fillStyle='#fff';ctx.beginPath();ctx.arc(cp[0],cp[1],5,0,6.283);ctx.fill();
ctx.strokeStyle='#eab308';ctx.lineWidth=2;ctx.beginPath();
ctx.moveTo(cp[0],cp[1]);ctx.lineTo(cp[0]+14*Math.cos(-mh),cp[1]+14*Math.sin(-mh));ctx.stroke();
var ch2=Math.cos(mh),sh2=Math.sin(mh);
for(i=0;i<4;i++){
var sx2=mx+SF*ch2-SL[i]*sh2,sy2=my+SF*sh2+SL[i]*ch2;
var ra2=mh+SD[i]*Math.PI/180;
var sp2=tx(sx2,sy2),ep=tx(sx2+.5*Math.cos(ra2),sy2+.5*Math.sin(ra2));
ctx.strokeStyle=SC[i];ctx.lineWidth=1;ctx.globalAlpha=.4;
ctx.beginPath();ctx.moveTo(sp2[0],sp2[1]);ctx.lineTo(ep[0],ep[1]);ctx.stroke();
ctx.globalAlpha=1;
}}
Q('mP').textContent=mx.toFixed(2)+', '+my.toFixed(2);
}

function mR(){mx=0;my=0;mh=0;mp=0;mo=0;mn=0;tr=[];wl=[[],[],[],[]];mD()}
function mZ(f){ms*=f;ms=Math.max(10,Math.min(3000,ms));mD()}

(function(){
var c=Q('mC'),dg=null,pn=0;
c.addEventListener('pointerdown',function(e){if(Q('mF').checked)Q('mF').checked=false;dg={x:e.clientX,y:e.clientY};c.setPointerCapture(e.pointerId)});
c.addEventListener('pointermove',function(e){if(!dg)return;mo+=e.clientX-dg.x;mn+=e.clientY-dg.y;dg={x:e.clientX,y:e.clientY};mD()});
c.addEventListener('pointerup',function(){dg=null});
c.addEventListener('pointercancel',function(){dg=null});
c.addEventListener('wheel',function(e){e.preventDefault();mZ(e.deltaY<0?1.15:.87)},{passive:false});
c.addEventListener('touchstart',function(e){if(e.touches.length===2){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;pn=Math.sqrt(dx*dx+dy*dy)}},{passive:true});
c.addEventListener('touchmove',function(e){if(e.touches.length===2&&pn>0){var dx=e.touches[0].clientX-e.touches[1].clientX,dy=e.touches[0].clientY-e.touches[1].clientY;var d=Math.sqrt(dx*dx+dy*dy);mZ(d/pn);pn=d}},{passive:true});
})();
setInterval(function(){if(md){md=false;mD()}},200);

// --- Toast ---
var ti=null;
function tt(m,t){var e=Q('T');e.textContent=m;e.className='toast '+(t||'inf')+' show';if(ti)clearTimeout(ti);ti=setTimeout(function(){e.classList.remove('show')},2000)}

cn();
</script>
</body>
</html>)rawliteral";

#endif
