#include "app.h"

String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '&') out += F("&amp;");
    else if (c == '<') out += F("&lt;");
    else if (c == '>') out += F("&gt;");
    else if (c == '"') out += F("&quot;");
    else if (c == '\'') out += F("&#39;");
    else out += c;
  }
  return out;
}

String jsonEscape(const String& in) {
  String out;
  out.reserve(in.length() + 16);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '\\') out += "\\\\";
    else if (c == '"') out += "\\\"";
    else if (c == '\n') out += "\\n";
    else if (c == '\r') out += "\\r";
    else if (c == '\t') out += "\\t";
    else out += c;
  }
  return out;
}

String htmlPage() {
  String h;
  h.reserve(7600);
  h += F("<!doctype html><html><head><meta name='viewport' content='width=device-width,initial-scale=1'>");
  h += F("<title>Divider Indexer Controller</title><style>");
  h += F(":root{--bg1:#f4f7fb;--bg2:#e8eef8;--card:#ffffff;--ink:#1e2b3a;--muted:#5c6b7a;--line:#d9e3ef;--accent:#007f73;--accent2:#0a4f8a;}");
  h += F("*{box-sizing:border-box}body{margin:0;font-family:'Trebuchet MS','Segoe UI',sans-serif;background:linear-gradient(180deg,var(--bg1),var(--bg2));color:var(--ink)}");
  h += F(".shell{max-width:980px;margin:0 auto;padding:14px}.title{margin:6px 0 14px;font-size:clamp(1.2rem,4.8vw,2rem);letter-spacing:.02em}");
  h += F(".topbar{display:flex;justify-content:flex-end;align-items:center;gap:8px;margin:0 0 10px}.settings{display:none;margin-bottom:12px}.diag{display:none;margin-bottom:12px}");
  h += F("body.operator button,body.operator input,body.operator select{font-size:18px;padding:12px 14px}body.operator .advanced{display:none!important}");
  h += F(".grid{display:grid;grid-template-columns:1.1fr 1fr;gap:12px}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:12px;box-shadow:0 4px 14px rgba(20,40,60,.08)}");
  h += F(".k{font-size:.82rem;color:var(--muted)}.v{font-size:1.05rem;font-weight:700}.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin:8px 0}");
  h += F("button,input{font-size:15px;border-radius:10px;border:1px solid #c8d6e8;padding:10px 12px}button{background:#fff;cursor:pointer}button.primary{background:var(--accent);color:#fff;border-color:var(--accent)}");
  h += F("button.secondary{background:var(--accent2);color:#fff;border-color:var(--accent2)}button.state-on{background:#1f9d55;color:#fff;border-color:#1f9d55}button.state-off{background:#b45309;color:#fff;border-color:#b45309}");
  h += F("input{min-width:120px}.status{white-space:pre-line;font-family:ui-monospace,Consolas,monospace;font-size:.84rem}");
  h += F(".dialWrap{display:flex;flex-direction:column;align-items:center;gap:6px}.dialDeg{font-size:1.3rem;font-weight:700}.tiny{font-size:.8rem;color:var(--muted)}");
  h += F(".net{margin-top:8px;padding:10px;border:1px dashed #9ab2cc;border-radius:12px;background:#f8fbff}");
  h += F("@media (max-width:800px){.grid{grid-template-columns:1fr}.shell{padding:10px}button,input{flex:1}}");
  h += F("</style></head><body><div class='shell'><h1 class='title'>Divider Indexer Controller</h1>");
  h += F("<div class='topbar'><button class='secondary' onclick='toggleSettings()'>Settings</button><button class='secondary' onclick='toggleDiagnostics()'>Diagnostics</button><button id='operatorModeBtn' class='secondary' onclick='toggleOperatorMode()'>Lock Operator Screen</button></div>");
  h += F("<div id='settingsPanel' class='card settings advanced'><div class='row'><input id='speed' type='number' value='4000' min='5' max='10000' oninput='markDirty(\"speed\")'><button class='secondary' onclick='setSpeed()'>Set Speed</button></div>");
  h += F("<div class='row'><input id='accel' type='number' value='3000' min='5' max='10000' oninput='markDirty(\"accel\")'><button class='secondary' onclick='setAccel()'>Set Accel</button></div>");
  h += F("<div class='row'><span class='tiny'>Module (mm)</span><input id='gearModule' type='number' value='1.0' min='0.001' step='0.001' oninput='markDirty(\"gearModule\")'><button class='secondary' onclick='setModule()'>Set Module</button></div>");
  h += F("<div class='row'><span class='tiny'>Pressure Angle (deg)</span><input id='gearPressureAngle' type='number' value='20.0' min='1' max='45' step='0.1' oninput='markDirty(\"gearPressureAngle\")'><button class='secondary' onclick='setPressureAngle()'>Set Pressure Angle</button></div>");
  h += F("<div class='row'><input id='backlash' type='number' value='0' min='0' step='1' oninput='markDirty(\"backlash\")'><button class='secondary' onclick='setBacklash()'>Set Backlash (steps)</button></div>");
  h += F("<div class='row'><input id='slop' type='number' value='0' min='-200000' step='1' oninput='markDirty(\"slop\")'><button class='secondary' onclick='setSlop()'>Set Slop (steps)</button></div>");
  h += F("<div class='row'><select id='stepperPort' oninput='markDirty(\"stepperPort\")'><option value='1'>Stepper1 (M1/M2)</option><option value='2'>Stepper2 (M3/M4)</option></select><button class='secondary' onclick='setStepperPort()'>Set Stepper Port</button></div>");
  h += F("<div class='row'><input id='setPosDeg' type='number' value='0' min='0' max='360' step='0.001'><button class='secondary' onclick='setPositionDeg()'>Set Absolute Deg</button></div>");
  h += F("<div class='row'><input id='setPosGear' type='number' value='1' min='1' step='1'><button class='secondary' onclick='setPositionGear()'>Set Absolute Gear</button></div>");
  h += F("<div class='row'><button class='secondary' onclick='zeroPosition()'>Zero Position</button></div>");
  h += F("<div class='row'><input id='p1name' placeholder='Preset 1 name'><button class='secondary' onclick='presetSave(1)'>Save P1</button><button class='secondary' onclick='presetLoad(1)'>Load P1</button></div>");
  h += F("<div class='row'><input id='p2name' placeholder='Preset 2 name'><button class='secondary' onclick='presetSave(2)'>Save P2</button><button class='secondary' onclick='presetLoad(2)'>Load P2</button></div>");
  h += F("<div class='row'><input id='p3name' placeholder='Preset 3 name'><button class='secondary' onclick='presetSave(3)'>Save P3</button><button class='secondary' onclick='presetLoad(3)'>Load P3</button></div></div>");
  h += F("<div id='diagPanel' class='card diag advanced'><div class='row'><button class='secondary' onclick='diagResetIsd()'>Reset ISD (Active)</button><button class='secondary' onclick='diagResetIsdPort(1)'>Reset ISD S1</button><button class='secondary' onclick='diagResetIsdPort(2)'>Reset ISD S2</button></div><div class='row'><button class='secondary' onclick='diagBridgeMode(\"m1\")'>M1 ON</button><button class='secondary' onclick='diagBridgeMode(\"m2\")'>M2 ON</button><button class='secondary' onclick='diagBridgeMode(\"m3\")'>M3 ON</button><button class='secondary' onclick='diagBridgeMode(\"m4\")'>M4 ON</button><button class='secondary' onclick='diagBridgeMode(\"off\")'>Bridge OFF</button></div><div class='row'><span class='tiny'>Diag Steps/Press</span><input id='diagStepCount' type='number' value='1' min='1' step='1'></div><div class='row'><button class='secondary' onclick='diagSingleStep(-1)'>Diag Step -</button><button class='secondary' onclick='diagSingleStep(1)'>Diag Step +</button></div><div class='row'><button class='secondary' onclick='diagTestBacklash()'>Test Backlash</button><button class='secondary' onclick='diagResetStepTotal()'>Reset Step Total</button></div><div class='status' id='diagText'>Diagnostics...</div></div>");
  h += F("<div class='grid'>");
  h += F("<div class='card'><div class='dialWrap'>");
  h += F("<svg id='dialSvg' width='250' height='250' viewBox='0 0 250 250' aria-label='Indexer dial'>");
  h += F("<defs><linearGradient id='g' x1='0' y1='0' x2='1' y2='1'><stop offset='0%' stop-color='#f8fcff'/><stop offset='100%' stop-color='#d9e8f7'/></linearGradient></defs>");
  h += F("<circle cx='125' cy='125' r='110' fill='url(#g)' stroke='#8ba7c4' stroke-width='2'/>");
  h += F("<circle cx='125' cy='125' r='85' fill='none' stroke='#b8cadf' stroke-width='1.5'/>");
  h += F("<line x1='125' y1='18' x2='125' y2='36' stroke='#445d78' stroke-width='3'/>");
  h += F("<text x='125' y='52' text-anchor='middle' font-size='14' fill='#2a425c'>0</text>");
  h += F("<text x='197' y='130' text-anchor='middle' font-size='14' fill='#2a425c'>90</text>");
  h += F("<text x='125' y='213' text-anchor='middle' font-size='14' fill='#2a425c'>180</text>");
  h += F("<text x='53' y='130' text-anchor='middle' font-size='14' fill='#2a425c'>270</text>");
  h += F("<line id='prevNeedle' x1='125' y1='125' x2='125' y2='56' stroke='#2f9e44' stroke-width='3' stroke-linecap='round'/>");
  h += F("<line id='nextNeedle' x1='125' y1='125' x2='125' y2='56' stroke='#2f9e44' stroke-width='3' stroke-linecap='round'/>");
  h += F("<line id='needle' x1='125' y1='125' x2='125' y2='34' stroke='#c62828' stroke-width='4' stroke-linecap='round'/>");
  h += F("<circle cx='125' cy='125' r='7' fill='#17324f'/></svg>");
  h += F("<div class='dialDeg'><span id='deg'>0.000</span>&deg;</div><div class='tiny'>Indexer Angle</div>");
  h += F("<div class='tiny' id='dialCtx'>Prev: -<br>Cur: -<br>Next: -</div></div>");
  h += F("</div>");
  h += F("<div class='card'><div class='row'><div><div class='k'>Mode</div><div class='v' id='mode'>-</div></div></div>");
  h += F("<div class='status' id='status'>Loading...</div>");
  h += F("<div class='row'><button onclick='cmd(\"/stepper/stop\")'>Stop</button></div>");
  h += F("<div class='row'><select id='moveUnit' onchange='onMoveUnitChanged()'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("<option value='gears'>Gears</option><option value='degrees' selected>Degrees</option>");
  } else {
    h += F("<option value='gears' selected>Gears</option><option value='degrees'>Degrees</option>");
  }
  h += F("</select><span id='moveLabel' class='tiny'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("Step Degrees");
  } else {
    h += F("Total Gears");
  }
  h += F("</span><input id='moveAmount' type='number' value='");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += String(uiMoveAmount, 3);
  } else {
    h += String(numberOfGears);
  }
  h += F("' min='0.001' step='0.001' oninput='markDirty(\"moveAmount\")'><button class='secondary' onclick='setMoveConfig()'>Apply Mode/Value</button></div>");
  h += F("<div class='row'><button id='indexMinusBtn' class='primary' onclick='indexStep(-1)'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("-Degree");
  } else {
    h += F("-1 Gear");
  }
  h += F("</button><button id='indexPlusBtn' class='primary' onclick='indexStep(1)'>");
  if (uiMoveUnit == MoveUnit::Degrees) {
    h += F("+Degree");
  } else {
    h += F("+1 Gear");
  }
  h += F("</button></div>");
  h += F("</div>");
  if (wifiMode == "AP") {
    h += F("<div class='card net' style='grid-column:1 / -1'><h3>STA Network Setup (Saved Encrypted)</h3>");
    h += F("<div class='row'><input id='ssid' placeholder='SSID' value='");
    h += htmlEscape(savedConfig.ssid);
    h += F("'><input id='password' type='password' placeholder='Password' value='");
    h += htmlEscape(savedConfig.password);
    h += F("'></div>");
    h += F("<div class='row'><input id='ip' placeholder='Static IP' value='");
    h += htmlEscape(savedConfig.staticIp);
    h += F("'><input id='gateway' placeholder='Gateway' value='");
    h += htmlEscape(savedConfig.gateway);
    h += F("'><input id='netmask' placeholder='Netmask' value='");
    h += htmlEscape(savedConfig.netmask);
    h += F("'></div>");
    h += F("<div class='row'><button class='secondary' onclick='saveNetwork()'>Save + Reboot</button></div></div>");
  }
  h += F("</div><script>");
  h += F("async function cmd(u){await fetch(u,{method:'POST'});refresh();}");
  h += F("function toggleSettings(){const p=document.getElementById('settingsPanel');if(!p)return;p.style.display=(p.style.display==='block')?'none':'block';}");
  h += F("function toggleDiagnostics(){const p=document.getElementById('diagPanel');if(!p)return;p.style.display=(p.style.display==='block')?'none':'block';}");
  h += F("function updateOperatorModeBtn(){const b=document.getElementById('operatorModeBtn');if(!b)return;b.innerText=document.body.classList.contains('operator')?'Unlock Operator Screen':'Lock Operator Screen';}");
  h += F("function toggleOperatorMode(){document.body.classList.toggle('operator');updateOperatorModeBtn();}");
  h += F("const dirtyFields=new Set();function markDirty(id){dirtyFields.add(id);}function clearDirty(id){dirtyFields.delete(id);}");
  h += F("async function setSpeed(){const s=document.getElementById('speed').value||4000;const r=await fetch('/stepper/speed?value='+s,{method:'POST'});if(r.ok)clearDirty('speed');refresh();}");
  h += F("async function setAccel(){const a=document.getElementById('accel').value||3000;const r=await fetch('/stepper/accel?value='+a,{method:'POST'});if(r.ok)clearDirty('accel');refresh();}");
  h += F("async function postGearGeometry(){const m=document.getElementById('gearModule').value||'1';const pa=document.getElementById('gearPressureAngle').value||'20';const p=new URLSearchParams({module:m,pressureAngle:pa});const r=await fetch('/settings/gear_geometry',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});if(!r.ok){alert(await r.text());return false;}return true;}");
  h += F("async function setModule(){if(await postGearGeometry())clearDirty('gearModule');refresh();}");
  h += F("async function setPressureAngle(){if(await postGearGeometry())clearDirty('gearPressureAngle');refresh();}");
  h += F("async function setBacklash(){const v=document.getElementById('backlash').value||0;const r=await fetch('/settings/backlash?value='+encodeURIComponent(v),{method:'POST'});if(r.ok)clearDirty('backlash');refresh();}");
  h += F("async function setSlop(){const v=document.getElementById('slop').value||0;const r=await fetch('/settings/slop?value='+encodeURIComponent(v),{method:'POST'});if(r.ok)clearDirty('slop');refresh();}");
  h += F("async function setStepperPort(){const v=document.getElementById('stepperPort').value||'2';const r=await fetch('/settings/stepper_port?value='+encodeURIComponent(v),{method:'POST'});if(r.ok)clearDirty('stepperPort');refresh();}");
  h += F("async function setPositionDeg(){const d=document.getElementById('setPosDeg').value||0;if(!confirm('Set current absolute position to '+d+' degrees?'))return;const r=await fetch('/indexer/set_position_deg?value='+encodeURIComponent(d),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function setPositionGear(){const g=document.getElementById('setPosGear').value||1;if(!confirm('Set current absolute position to gear '+g+'?'))return;const r=await fetch('/indexer/set_position_gear?value='+encodeURIComponent(g),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function zeroPosition(){const r=await fetch('/indexer/zero',{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function presetSave(slot){const n=(document.getElementById('p'+slot+'name')||{}).value||('Preset '+slot);const p=new URLSearchParams({slot:String(slot),name:n});const r=await fetch('/preset/save',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function presetLoad(slot){const r=await fetch('/preset/load?slot='+slot,{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagResetIsd(){const r=await fetch('/diag/reset_isd',{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagResetIsdPort(port){const r=await fetch('/diag/reset_isd_port?port='+encodeURIComponent(port),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagBridgeMode(mode){const r=await fetch('/diag/bridge_mode?mode='+encodeURIComponent(mode),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagSingleStep(dir){const c=(document.getElementById('diagStepCount')||{}).value||'1';const r=await fetch('/diag/single_step?dir='+encodeURIComponent(dir)+'&count='+encodeURIComponent(c),{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagTestBacklash(){const r=await fetch('/diag/test_backlash',{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function diagResetStepTotal(){const r=await fetch('/diag/reset_step_total',{method:'POST'});if(!r.ok){alert(await r.text());}refresh();}");
  h += F("async function indexStep(dir){await fetch('/indexer/step?dir='+dir,{method:'POST'});refresh();}");
  h += F("function onMoveUnitChanged(){const u=document.getElementById('moveUnit').value;const lbl=document.getElementById('moveLabel');const m=document.getElementById('moveAmount');if(lbl){lbl.innerText=(u==='degrees')?'Step Degrees':'Total Gears';}if(m){m.min=(u==='degrees')?'0.001':'1';m.step=(u==='degrees')?'0.001':'1';}}");
  h += F("async function setMoveConfig(){const p=new URLSearchParams({unit:document.getElementById('moveUnit').value,amount:document.getElementById('moveAmount').value||'1'});const r=await fetch('/move/config',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});if(!r.ok){alert(await r.text());}else{clearDirty('moveAmount');}refresh();}");
  h += F("async function saveNetwork(){const p=new URLSearchParams({ssid:document.getElementById('ssid').value,password:document.getElementById('password').value,ip:document.getElementById('ip').value,gateway:document.getElementById('gateway').value,netmask:document.getElementById('netmask').value});const r=await fetch('/config/network',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded'},body:p});alert(await r.text());}");
  h += F("function renderDial(deg){const needle=document.getElementById('needle');needle.setAttribute('transform',`rotate(${deg} 125 125)`);document.getElementById('deg').innerText=Number(deg).toFixed(3);}");
  h += F("function wrapDeg(v){let d=v%360;if(d<0)d+=360;return d;}");
  h += F("function setDialMarker(id,deg){const el=document.getElementById(id);if(el){el.setAttribute('transform',`rotate(${deg} 125 125)`);}}");
  h += F("function updateDialContext(j){let prev='-',cur='-',next='-';let prevDeg=0,curDeg=0,nextDeg=0;if(j.moveUnit==='degrees'){const step=Number(j.moveAmount)||1;curDeg=Number(j.indexerDeg)||0;prevDeg=wrapDeg(curDeg-step);nextDeg=wrapDeg(curDeg+step);prev=`${prevDeg.toFixed(3)} deg`;cur=`${curDeg.toFixed(3)} deg`;next=`${nextDeg.toFixed(3)} deg`;}else{const gears=Math.max(1,parseInt(j.gears)||1);const stepDeg=360/gears;const eps=1e-6;curDeg=wrapDeg(Number(j.indexerDeg)||0);const idx=Math.floor(curDeg/stepDeg);const lineDeg=idx*stepDeg;const onLine=Math.abs(curDeg-lineDeg)<eps||Math.abs(curDeg)<eps;let prevGear=1,nextGear=1;if(onLine){const curGear=(idx===0)?gears:idx;prevGear=curGear-1;if(prevGear<1)prevGear=gears;nextGear=curGear+1;if(nextGear>gears)nextGear=1;}else{prevGear=(idx===0)?gears:idx;nextGear=prevGear+1;if(nextGear>gears)nextGear=1;}prevDeg=(prevGear===gears)?360:(prevGear*stepDeg);nextDeg=nextGear*stepDeg;prev=`G${prevGear}/${gears} (~${prevDeg.toFixed(1)} deg)`;cur=`${curDeg.toFixed(3)} deg`;next=`G${nextGear}/${gears} (~${nextDeg.toFixed(1)} deg)`;}setDialMarker('prevNeedle',prevDeg);setDialMarker('nextNeedle',nextDeg);const el=document.getElementById('dialCtx');if(el){el.innerHTML=`Prev: ${prev}<br>Cur: ${cur}<br>Next: ${next}`;}}");
  h += F("function setIfIdle(id,val){const el=document.getElementById(id);if(!el)return;if(document.activeElement===el||dirtyFields.has(id))return;el.value=val;}");
  h += F("function updateMoveUi(j){const lbl=document.getElementById('moveLabel');if(lbl){lbl.innerText=(j.moveUnit==='degrees')?'Step Degrees':'Total Gears';}const m=document.getElementById('moveAmount');if(!m)return;m.min=(j.moveUnit==='degrees')?'0.001':'1';m.step=(j.moveUnit==='degrees')?'0.001':'1';if(document.activeElement!==m&&!dirtyFields.has('moveAmount')){m.value=(j.moveUnit==='degrees')?j.degreeStep:j.gears;}}");
  h += F("async function refresh(){const r=await fetch('/status');const j=await r.json();document.getElementById('mode').innerText=j.wifiMode;setIfIdle('speed',j.speed);setIfIdle('accel',j.accel);setIfIdle('gearModule',j.gearModule);setIfIdle('gearPressureAngle',j.gearPressureAngle);setIfIdle('backlash',j.backlash);setIfIdle('slop',j.slop);setIfIdle('stepperPort',j.stepperPort);setIfIdle('setPosGear',j.currentGear);setIfIdle('p1name',j.p1);setIfIdle('p2name',j.p2);setIfIdle('p3name',j.p3);updateMoveUi(j);const unitSel=document.getElementById('moveUnit');if(document.activeElement!==unitSel){unitSel.value=j.moveUnit;}document.getElementById('indexPlusBtn').innerText=(j.moveUnit==='degrees')?'+Degree':'+1 Gear';document.getElementById('indexMinusBtn').innerText=(j.moveUnit==='degrees')?'-Degree':'-1 Gear';document.getElementById('status').innerText=`Actual ${j.position} (${Number(j.indexerDeg).toFixed(3)} deg)\\nTarget ${j.target} (${Number(j.cmdDeg).toFixed(3)} deg)\\nModule ${Number(j.gearModule).toFixed(3)} mm  PA ${Number(j.gearPressureAngle).toFixed(1)} deg\\nO.D. ${Number(j.gearOutsideDiameter).toFixed(3)} mm  Tooth Depth ${Number(j.gearToothDepth).toFixed(3)} mm\\nAngle/Tooth ${Number(j.angleBetweenGears).toFixed(3)} deg`;const d=document.getElementById('diagText');if(d){d.innerText=`WiFi: ${j.wifiMode} RSSI=${j.rssi}dBm\\nUptime: ${Math.floor(j.uptimeMs/1000)}s\\nISR: ${j.isrHz} Hz  StepRate: ${j.stepHz} Hz\\nTotal ISR Steps: ${j.totalIsrSteps}\\nBacklash: ${j.backlash} steps\\nSlop: ${j.slop} steps\\nBridgeTest: ${j.diagBridgeMode}\\nFault: ${j.lastFault}\\nMissed(est): ${j.missedEst}`;}renderDial(j.indexerDeg);updateDialContext(j);}");
  h += F("setInterval(refresh,1000);refresh();updateOperatorModeBtn();");
  h += F("</script></body></html>");
  return h;
}

void sendJsonStatus() {
  long posAbs = getStepperPositionAtomic();
  long physicalTgtAbs = getTargetPositionAtomic();
  long logicalTgtAbs = indexedLogicalPosition;
  long pos = modPositive(posAbs, STEPS_PER_INDEXER_REV);
  long tgt = modPositive(logicalTgtAbs, STEPS_PER_INDEXER_REV);
  long physicalTgt = modPositive(physicalTgtAbs, STEPS_PER_INDEXER_REV);
  long cmdPos = lround(commandedStepsFromZero);
  long absErr = labs(physicalTgtAbs - posAbs);
  int currentGear = (uiMoveUnit == MoveUnit::Gears) ? (logicalGearIndex + 1) : getCurrentGearFromPosition(indexedLogicalPosition);
  float cmdDeg = (static_cast<float>(tgt) * 360.0f) / static_cast<float>(STEPS_PER_INDEXER_REV);
  float actualDeg = getIndexerDegrees();
  float gearOutsideDiameter = gearModule * static_cast<float>(numberOfGears + 2);
  float gearToothDepth = gearModule * 2.25f;
  float angleBetweenGears = (numberOfGears > 0) ? (360.0f / static_cast<float>(numberOfGears)) : 0.0f;
  String json = "{";
  json += "\"ip\":\"" + ipAddr.toString() + "\",";
  json += "\"bootIp\":\"" + (bootIpCaptured ? bootIpAddr.toString() : String("")) + "\",";
  json += "\"wifiMode\":\"" + wifiMode + "\",";
  json += "\"enabled\":" + String(stepperEnabled ? "true" : "false") + ",";
  json += "\"position\":" + String(pos) + ",";
  json += "\"target\":" + String(tgt) + ",";
  json += "\"physicalTarget\":" + String(physicalTgt) + ",";
  json += "\"cmdPosition\":" + String(cmdPos) + ",";
  json += "\"positionError\":" + String(absErr) + ",";
  json += "\"indexerDeg\":" + String(actualDeg, 3) + ",";
  json += "\"cmdDeg\":" + String(cmdDeg, 3) + ",";
  json += "\"gears\":" + String(numberOfGears) + ",";
  json += "\"stepperPort\":" + String(stepperPort) + ",";
  json += "\"currentGear\":" + String(currentGear) + ",";
  json += "\"ticksPerGear\":" + String(ticksPerGear) + ",";
  json += "\"moveUnit\":\"" + String(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears") + "\",";
  json += "\"moveAmount\":" + String(uiMoveAmount, 3) + ",";
  json += "\"degreeStep\":" + String(degreeStepSetting, 3) + ",";
  json += "\"gearModule\":" + String(gearModule, 3) + ",";
  json += "\"gearPressureAngle\":" + String(gearPressureAngleDeg, 1) + ",";
  json += "\"gearOutsideDiameter\":" + String(gearOutsideDiameter, 3) + ",";
  json += "\"gearToothDepth\":" + String(gearToothDepth, 3) + ",";
  json += "\"angleBetweenGears\":" + String(angleBetweenGears, 3) + ",";
  json += "\"speed\":" + String(speedStepsPerSec, 1) + ",";
  json += "\"accel\":" + String(accelStepsPerSec2, 1) + ",";
  json += "\"backlash\":" + String(backlashSteps) + ",";
  json += "\"slop\":" + String(slopSteps) + ",";
  json += "\"b1\":" + String(button1Pressed ? "true" : "false") + ",";
  json += "\"b2\":" + String(button2Pressed ? "true" : "false") + ",";
  json += "\"hasCfg\":" + String(hasStoredNetworkConfig ? "true" : "false") + ",";
  json += "\"uptimeMs\":" + String(millis()) + ",";
  json += "\"rssi\":" + String((wifiMode == "STA") ? WiFi.RSSI() : 0) + ",";
  json += "\"isrHz\":" + String(diagIsrTicksPerSec) + ",";
  json += "\"stepHz\":" + String(diagStepRatePerSec) + ",";
  json += "\"totalIsrSteps\":\"" + formatUint64(getTotalInterruptStepsAtomic()) + "\",";
  json += "\"missedEst\":" + String(missedStepEstimate) + ",";
  String diagMode = "off";
  if (diagBridgeMode == DiagBridgeMode::M1On) diagMode = "m1";
  else if (diagBridgeMode == DiagBridgeMode::M2On) diagMode = "m2";
  else if (diagBridgeMode == DiagBridgeMode::M3On) diagMode = "m3";
  else if (diagBridgeMode == DiagBridgeMode::M4On) diagMode = "m4";
  json += "\"diagBridgeMode\":\"" + diagMode + "\",";
  json += "\"lastFault\":\"" + jsonEscape(lastFault) + "\",";
  json += "\"p1\":\"" + jsonEscape(presets[0].name) + "\",";
  json += "\"p2\":\"" + jsonEscape(presets[1].name) + "\",";
  json += "\"p3\":\"" + jsonEscape(presets[2].name) + "\"";
  json += "}";
  server.send(200, "application/json", json);
}

void handleRoot() { server.send(200, "text/html", htmlPage()); }
void handleStatus() { sendJsonStatus(); }
void handleStepperStop() {
  noInterrupts();
  long pos = stepperPosition;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  lastCommandDir = 0;
  timerMotionActive = false;
  halfStepInProgress = false;
  interrupts();
  degreeIdealSynced = false;
  syncIndexedLogicalPosition(pos);
  server.send(200, "text/plain", "OK");
}

void handleStepperMove() {
  Serial.println("[HTTP] /stepper/move");
  if (!stepperEnabled) {
    Serial.println("[STEP] move rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  if (!server.hasArg("steps")) {
    server.send(400, "text/plain", "Missing steps");
    return;
  }
  long steps = server.arg("steps").toInt();
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  if (steps < 0) steps = -steps;
  if (dir < 0) steps = -steps;
  long currentPos = getStepperPositionAtomic();
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  long nextTarget = applyBacklashCompensation(currentPos, currentPos + steps);
  if (nextTarget != currentPos) showMovingScreen();
  setTargetAndCommandedAtomic(nextTarget);
  degreeIdealSynced = false;
  syncIndexedLogicalPosition(nextTarget);
  Serial.print("[STEP] move target=");
  Serial.println(modPositive(nextTarget, STEPS_PER_INDEXER_REV));
  server.send(200, "text/plain", "OK");
}

void handleStepperSingleStep() {
  Serial.println("[HTTP] /stepper/single");
  if (!stepperEnabled) {
    Serial.println("[STEP] single step rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  long currentPos = getStepperPositionAtomic();
  long nextTarget = applyBacklashCompensation(currentPos, currentPos + ((dir < 0) ? -1L : 1L));
  if (nextTarget != currentPos) showMovingScreen();
  setTargetAndCommandedAtomic(nextTarget);
  degreeIdealSynced = false;
  syncIndexedLogicalPosition(nextTarget);
  Serial.print("[STEP] single queued dir=");
  Serial.print(dir);
  Serial.print(" target=");
  Serial.println(modPositive(nextTarget, STEPS_PER_INDEXER_REV));
  server.send(200, "text/plain", "OK");
}

void handleStepperSpeed() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float v = server.arg("value").toFloat();
  if (v < 5.0f) v = 5.0f;
  if (v > 10000.0f) v = 10000.0f;
  speedStepsPerSec = v;
  applyStepperSpeed();
  noInterrupts();
  timerStepIntervalRequestUs = stepIntervalUs;
  timerStepIntervalDirty = true;
  interrupts();
  saveControlSettings();
  Serial.print("[STEP] speed=");
  Serial.println(speedStepsPerSec);
  server.send(200, "text/plain", "OK");
}

void handleStepperAccel() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float a = server.arg("value").toFloat();
  if (a < 5.0f) a = 5.0f;
  if (a > 10000.0f) a = 10000.0f;
  accelStepsPerSec2 = a;
  saveControlSettings();
  Serial.print("[STEP] accel=");
  Serial.println(accelStepsPerSec2);
  server.send(200, "text/plain", "OK");
}

void handleIndexerStep() {
  Serial.println("[HTTP] /indexer/step");
  if (!stepperEnabled) {
    Serial.println("[STEP] index step rejected: disabled");
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  long currentPos = getStepperPositionAtomic();
  long logicalCurrentPos = indexedLogicalPosition;
  long logicalTarget = computeIndexedAbsoluteTargetFromCurrent(logicalCurrentPos, dir, uiMoveUnit, uiMoveAmount);
  long nextTarget = computeIndexedPhysicalTarget(currentPos, logicalCurrentPos, logicalTarget);
  if (uiMoveUnit == MoveUnit::Gears) {
    long gearMoves = lround(uiMoveAmount);
    if (gearMoves < 1) gearMoves = 1;
    logicalGearIndex = normalizeGearIndex(logicalGearIndex + ((dir > 0) ? gearMoves : -gearMoves));
  }
  nextTarget = applyBacklashCompensation(currentPos, nextTarget);
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  if (nextTarget != currentPos) showMovingScreen();
  setTargetAndCommandedAtomic(nextTarget);
  indexedLogicalPosition = logicalTarget;
  commandedStepsFromZero = static_cast<double>(logicalTarget);
  Serial.print("[STEP] index dir=");
  Serial.print(dir);
  Serial.print(" unit=");
  Serial.print(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears");
  Serial.print(" amount=");
  Serial.print(uiMoveAmount, 3);
  Serial.print(" target=");
  Serial.println(modPositive(nextTarget, STEPS_PER_INDEXER_REV));
  server.send(200, "text/plain", "OK");
}

void handleIndexerSetGears() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  int nextGears = server.arg("value").toInt();
  if (nextGears < 1) {
    server.send(400, "text/plain", "Gears must be >= 1");
    return;
  }
  numberOfGears = nextGears;
  recalcIndexerTicks();
  uiMoveUnit = MoveUnit::Gears;
  uiMoveAmount = 1.0f;
  degreeIdealSynced = false;
  saveControlSettings();
  server.send(200, "text/plain", "OK");
}

void handleMoveConfig() {
  if (!server.hasArg("unit") || !server.hasArg("amount")) {
    server.send(400, "text/plain", "Missing unit or amount");
    return;
  }
  String unit = server.arg("unit");
  float amount = server.arg("amount").toFloat();
  if (amount <= 0.0f) {
    server.send(400, "text/plain", "Amount must be > 0");
    return;
  }
  if (unit == "degrees") {
    uiMoveUnit = MoveUnit::Degrees;
    uiMoveAmount = amount;
    degreeStepSetting = amount;
    syncDegreeIdealToPosition(getStepperPositionAtomic());
    saveControlSettings();
  } else {
    uiMoveUnit = MoveUnit::Gears;
    int nextGears = static_cast<int>(lround(amount));
    if (nextGears < 1) {
      server.send(400, "text/plain", "Gears must be >= 1");
      return;
    }
    numberOfGears = nextGears;
    recalcIndexerTicks();
    syncIndexedLogicalPosition(indexedLogicalPosition);
    uiMoveAmount = 1.0f;
    degreeIdealSynced = false;
    saveControlSettings();
  }
  Serial.print("[MOVE] unit=");
  Serial.print(uiMoveUnit == MoveUnit::Degrees ? "degrees" : "gears");
  Serial.print(" amount=");
  Serial.println(uiMoveAmount, 3);
  server.send(200, "text/plain", "OK");
}

void handleIndexerZero() {
  noInterrupts();
  stepperPosition = 0;
  targetPosition = 0;
  commandedStepsFromZero = 0.0;
  timerMotionActive = false;
  lastCommandDir = 0;
  halfStepInProgress = false;
  interrupts();
  syncDegreeIdealToPosition(0);
  syncIndexedLogicalPosition(0);
  saveControlSettings();
  Serial.println("[INDEX] zeroed current position reference");
  server.send(200, "text/plain", "OK");
}

void handleIndexerSetPositionDeg() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  float deg = server.arg("value").toFloat();
  if (std::isnan(deg) || std::isinf(deg)) {
    server.send(400, "text/plain", "Invalid degree value");
    return;
  }
  while (deg < 0.0f) deg += 360.0f;
  while (deg >= 360.0f) deg -= 360.0f;
  long pos = lround((static_cast<double>(deg) * static_cast<double>(STEPS_PER_INDEXER_REV)) / 360.0);
  pos -= modPositive(pos, COMMUTATION_STATES_PER_FULL_STEP);
  noInterrupts();
  stepperPosition = pos;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  timerMotionActive = false;
  lastCommandDir = 0;
  halfStepInProgress = false;
  interrupts();
  syncDegreeIdealToPosition(pos);
  syncIndexedLogicalPosition(pos);
  saveControlSettings();
  Serial.print("[INDEX] set position deg=");
  Serial.print(deg, 3);
  Serial.print(" steps=");
  Serial.println(pos);
  server.send(200, "text/plain", "OK");
}

void handleIndexerSetPositionGear() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  int gear = server.arg("value").toInt();
  if (gear < 1 || gear > numberOfGears) {
    server.send(400, "text/plain", "Gear out of range");
    return;
  }
  long pos = lround((static_cast<double>(gear - 1) * static_cast<double>(STEPS_PER_INDEXER_REV)) /
                    static_cast<double>(numberOfGears));
  pos -= modPositive(pos, COMMUTATION_STATES_PER_FULL_STEP);
  noInterrupts();
  stepperPosition = pos;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  timerMotionActive = false;
  lastCommandDir = 0;
  halfStepInProgress = false;
  interrupts();
  syncDegreeIdealToPosition(pos);
  syncIndexedLogicalPosition(pos);
  saveControlSettings();
  Serial.print("[INDEX] set position gear=");
  Serial.print(gear);
  Serial.print(" steps=");
  Serial.println(pos);
  server.send(200, "text/plain", "OK");
}

void handleSetBacklash() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  long v = server.arg("value").toInt();
  if (v < 0) v = 0;
  if (v > 200000) v = 200000;
  backlashSteps = v;
  saveControlSettings();
  server.send(200, "text/plain", "OK");
}

void handleSetSlop() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  long v = server.arg("value").toInt();
  if (v < -200000) v = -200000;
  if (v > 200000) v = 200000;
  slopSteps = v;
  saveControlSettings();
  server.send(200, "text/plain", "OK");
}

void handleSetGearGeometry() {
  if (!server.hasArg("module") || !server.hasArg("pressureAngle")) {
    server.send(400, "text/plain", "Missing module or pressureAngle");
    return;
  }
  float nextModule = server.arg("module").toFloat();
  float nextPressureAngle = server.arg("pressureAngle").toFloat();
  if (nextModule <= 0.0f) {
    server.send(400, "text/plain", "Module must be > 0");
    return;
  }
  if (nextPressureAngle <= 0.0f || nextPressureAngle > 45.0f) {
    server.send(400, "text/plain", "Pressure angle must be > 0 and <= 45");
    return;
  }
  gearModule = nextModule;
  gearPressureAngleDeg = nextPressureAngle;
  saveControlSettings();
  server.send(200, "text/plain", "OK");
}

void handleSetStepperPort() {
  if (!server.hasArg("value")) {
    server.send(400, "text/plain", "Missing value");
    return;
  }
  int v = server.arg("value").toInt();
  if (v != 1 && v != 2) {
    server.send(400, "text/plain", "Stepper port must be 1 or 2");
    return;
  }
  noInterrupts();
  long pos = stepperPosition;
  targetPosition = pos;
  commandedStepsFromZero = static_cast<double>(pos);
  timerMotionActive = false;
  outputCommand = OUTPUT_CMD_NONE;
  interrupts();
  diagBridgeMode = DiagBridgeMode::Off;
  hardDisableStepperPins();
  applyStepperPortSelection(static_cast<uint8_t>(v));
  pinMode(STEP1_IN1, OUTPUT);
  pinMode(STEP1_IN2, OUTPUT);
  pinMode(STEP1_IN3, OUTPUT);
  pinMode(STEP1_IN4, OUTPUT);
  pinMode(STEP2_IN1, OUTPUT);
  pinMode(STEP2_IN2, OUTPUT);
  pinMode(STEP2_IN3, OUTPUT);
  pinMode(STEP2_IN4, OUTPUT);
  digitalWrite(STEP1_IN1, LOW);
  digitalWrite(STEP1_IN2, LOW);
  digitalWrite(STEP1_IN3, LOW);
  digitalWrite(STEP1_IN4, LOW);
  digitalWrite(STEP2_IN1, LOW);
  digitalWrite(STEP2_IN2, LOW);
  digitalWrite(STEP2_IN3, LOW);
  digitalWrite(STEP2_IN4, LOW);
  saveControlSettings();
  Serial.print("[CFG] stepper_port=");
  Serial.println(stepperPort);
  server.send(200, "text/plain", "OK");
}

void handlePresetSave() {
  if (!server.hasArg("slot")) {
    server.send(400, "text/plain", "Missing slot");
    return;
  }
  int slot = server.arg("slot").toInt();
  if (slot < 1 || slot > 3) {
    server.send(400, "text/plain", "Slot must be 1..3");
    return;
  }
  int i = slot - 1;
  presets[i].name = server.hasArg("name") ? server.arg("name") : ("Preset " + String(slot));
  if (presets[i].name.length() == 0) presets[i].name = "Preset " + String(slot);
  presets[i].gears = numberOfGears;
  presets[i].degreeStep = degreeStepSetting;
  presets[i].speed = speedStepsPerSec;
  presets[i].accel = accelStepsPerSec2;
  presets[i].gearModule = gearModule;
  presets[i].gearPressureAngleDeg = gearPressureAngleDeg;
  savePresetSlot(i);
  server.send(200, "text/plain", "OK");
}

void handlePresetLoad() {
  if (!server.hasArg("slot")) {
    server.send(400, "text/plain", "Missing slot");
    return;
  }
  int slot = server.arg("slot").toInt();
  if (slot < 1 || slot > 3) {
    server.send(400, "text/plain", "Slot must be 1..3");
    return;
  }
  int i = slot - 1;
  numberOfGears = presets[i].gears;
  if (numberOfGears < 1) numberOfGears = 1;
  recalcIndexerTicks();
  syncIndexedLogicalPosition(indexedLogicalPosition);
  degreeStepSetting = presets[i].degreeStep;
  if (degreeStepSetting <= 0.0f) degreeStepSetting = 10.0f;
  if (uiMoveUnit == MoveUnit::Degrees) uiMoveAmount = degreeStepSetting;
  else uiMoveAmount = 1.0f;
  speedStepsPerSec = presets[i].speed;
  if (speedStepsPerSec < 5.0f) speedStepsPerSec = 5.0f;
  accelStepsPerSec2 = presets[i].accel;
  if (accelStepsPerSec2 < 5.0f) accelStepsPerSec2 = 5.0f;
  gearModule = presets[i].gearModule;
  if (gearModule <= 0.0f) gearModule = 1.0f;
  gearPressureAngleDeg = presets[i].gearPressureAngleDeg;
  if (gearPressureAngleDeg <= 0.0f) gearPressureAngleDeg = 20.0f;
  if (gearPressureAngleDeg > 45.0f) gearPressureAngleDeg = 45.0f;
  applyStepperSpeed();
  noInterrupts();
  timerStepIntervalRequestUs = stepIntervalUs;
  timerStepIntervalDirty = true;
  interrupts();
  saveControlSettings();
  server.send(200, "text/plain", "OK");
}

void handleSaveNetworkConfig() {
  Serial.println("[HTTP] /config/network POST");
  if (wifiMode != "AP") {
    Serial.println("[HTTP] rejected: not AP mode");
    server.send(403, "text/plain", "Only available in AP mode");
    return;
  }
  if (!server.hasArg("ssid") || !server.hasArg("password") || !server.hasArg("ip") ||
      !server.hasArg("gateway") || !server.hasArg("netmask")) {
    Serial.println("[HTTP] rejected: missing fields");
    server.send(400, "text/plain", "Missing fields");
    return;
  }
  NetworkConfig nextCfg;
  nextCfg.ssid = server.arg("ssid");
  nextCfg.password = server.arg("password");
  nextCfg.staticIp = server.arg("ip");
  nextCfg.gateway = server.arg("gateway");
  nextCfg.netmask = server.arg("netmask");
  IPAddress ip, gw, mask;
  if (nextCfg.ssid.length() == 0 || !parseIpArg(nextCfg.staticIp, ip) ||
      !parseIpArg(nextCfg.gateway, gw) || !parseIpArg(nextCfg.netmask, mask)) {
    server.send(400, "text/plain", "Invalid SSID or IP settings");
    return;
  }
  if (!saveNetworkConfig(nextCfg)) {
    server.send(500, "text/plain", "Failed to save encrypted config");
    return;
  }
  savedConfig = nextCfg;
  hasStoredNetworkConfig = true;
  server.send(200, "text/plain", "Saved encrypted config. Rebooting...");
  delay(350);
  ESP.restart();
}

void handleDiagResetIsd() {
  noInterrupts();
  timerMotionActive = false;
  outputCommand = OUTPUT_CMD_NONE;
  interrupts();
  writeStepperOutputs(false, false, false, false);
  delay(2);
  if (diagBridgeMode == DiagBridgeMode::Off) {
    hardEnableStepperPins();
    noInterrupts();
    outputCommand = OUTPUT_CMD_HOLD_PHASE;
    interrupts();
  } else {
    applyDiagBridgeModeOutput();
  }
  lastFault = "NONE";
  Serial.println("[DIAG] ISD reset sequence applied");
  server.send(200, "text/plain", "OK");
}

void handleDiagResetIsdPort() {
  if (!server.hasArg("port")) {
    server.send(400, "text/plain", "Missing port");
    return;
  }
  int port = server.arg("port").toInt();
  if (port != 1 && port != 2) {
    server.send(400, "text/plain", "port must be 1 or 2");
    return;
  }
  uint8_t in1 = (port == 1) ? STEP1_IN1 : STEP2_IN1;
  uint8_t in2 = (port == 1) ? STEP1_IN2 : STEP2_IN2;
  uint8_t in3 = (port == 1) ? STEP1_IN3 : STEP2_IN3;
  uint8_t in4 = (port == 1) ? STEP1_IN4 : STEP2_IN4;
  noInterrupts();
  timerMotionActive = false;
  outputCommand = OUTPUT_CMD_NONE;
  interrupts();
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(2);
  digitalWrite(in1, HIGH);
  delayMicroseconds(50);
  digitalWrite(in1, LOW);
  if (diagBridgeMode != DiagBridgeMode::Off) applyDiagBridgeModeOutput();
  else hardDisableStepperPins();
  lastFault = "NONE";
  Serial.print("[DIAG] ISD reset sequence applied for stepper");
  Serial.println(port);
  server.send(200, "text/plain", "OK");
}

void handleDiagBridgeMode() {
  if (!server.hasArg("mode")) {
    server.send(400, "text/plain", "Missing mode");
    return;
  }
  String mode = server.arg("mode");
  if (mode == "m1") diagBridgeMode = DiagBridgeMode::M1On;
  else if (mode == "m2") diagBridgeMode = DiagBridgeMode::M2On;
  else if (mode == "m3") diagBridgeMode = DiagBridgeMode::M3On;
  else if (mode == "m4") diagBridgeMode = DiagBridgeMode::M4On;
  else if (mode == "off") diagBridgeMode = DiagBridgeMode::Off;
  else {
    server.send(400, "text/plain", "mode must be off|m1|m2|m3|m4");
    return;
  }
  noInterrupts();
  timerMotionActive = false;
  outputCommand = OUTPUT_CMD_NONE;
  interrupts();
  if (diagBridgeMode == DiagBridgeMode::Off) {
    hardDisableStepperPins();
    Serial.println("[DIAG] bridge test OFF");
  } else {
    applyDiagBridgeModeOutput();
    Serial.print("[DIAG] bridge test mode=");
    if (diagBridgeMode == DiagBridgeMode::M1On) Serial.println("M1_ON");
    else if (diagBridgeMode == DiagBridgeMode::M2On) Serial.println("M2_ON");
    else if (diagBridgeMode == DiagBridgeMode::M3On) Serial.println("M3_ON");
    else Serial.println("M4_ON");
  }
  server.send(200, "text/plain", "OK");
}

void handleDiagTestBacklash() {
  if (!stepperEnabled) {
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  if (backlashSteps <= 0) {
    server.send(400, "text/plain", "Set backlash above 0 first");
    return;
  }
  diagBacklashTestActive = true;
  diagBacklashTestDir = (lastCommandDir > 0) ? -1 : 1;
  diagBacklashTestRemainingSegments = 6;
  diagBacklashPauseUntilMs = 0;
  server.send(200, "text/plain", "OK");
}

void processDiagBacklashTest() {
  if (!diagBacklashTestActive) {
    return;
  }
  if (!stepperEnabled) {
    diagBacklashTestActive = false;
    diagBacklashTestRemainingSegments = 0;
    diagBacklashPauseUntilMs = 0;
    return;
  }
  if (getStepperPositionAtomic() != getTargetPositionAtomic()) {
    return;
  }
  if (halfStepInProgress) {
    return;
  }
  if (diagBacklashTestRemainingSegments <= 0) {
    diagBacklashTestActive = false;
    diagBacklashPauseUntilMs = 0;
    syncIndexedLogicalPosition(getStepperPositionAtomic());
    return;
  }
  if (diagBacklashPauseUntilMs == 1) {
    diagBacklashPauseUntilMs = millis() + 1000UL;
    return;
  }
  if (diagBacklashPauseUntilMs != 0 && millis() < diagBacklashPauseUntilMs) {
    return;
  }
  diagBacklashPauseUntilMs = 0;

  applyStepperSpeed();
  currentSpeedStepsPerSec = START_SPEED_STEPS_PER_SEC;
  lastRampUs = micros();
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
  }

  long currentPos = getStepperPositionAtomic();
  long nextTarget = applyBacklashCompensation(
      currentPos, currentPos + (static_cast<long>(diagBacklashTestDir) * backlashSteps));
  if (nextTarget == currentPos) {
    diagBacklashTestRemainingSegments = 0;
    diagBacklashTestActive = false;
    return;
  }
  showMovingScreen();
  setTargetAndCommandedAtomic(nextTarget);
  diagBacklashTestDir = -diagBacklashTestDir;
  diagBacklashTestRemainingSegments--;
  diagBacklashPauseUntilMs = 1;
}

void handleDiagSingleStep() {
  if (!stepperEnabled) {
    server.send(409, "text/plain", "Stepper is disabled");
    return;
  }
  int dir = server.hasArg("dir") ? server.arg("dir").toInt() : 1;
  long count = server.hasArg("count") ? server.arg("count").toInt() : 1;
  if (count < 1) count = 1;
  long currentPos = getStepperPositionAtomic();
  if (stepperOutputsReleased) {
    hardEnableStepperPins();
    applyStepperSpeed();
  }
  long nextTarget = currentPos + ((dir < 0) ? -count : count);
  if (nextTarget != currentPos) showMovingScreen();
  setTargetAndCommandedAtomic(nextTarget);
  degreeIdealSynced = false;
  syncIndexedLogicalPosition(nextTarget);
  Serial.print("[DIAG] single step dir=");
  Serial.print(dir < 0 ? -1 : 1);
  Serial.print(" count=");
  Serial.print(count);
  Serial.print(" start=");
  Serial.print(modPositive(currentPos, STEPS_PER_INDEXER_REV));
  Serial.print(" target=");
  Serial.println(modPositive(nextTarget, STEPS_PER_INDEXER_REV));
  server.send(200, "text/plain", "OK");
}

void handleDiagResetStepTotal() {
  noInterrupts();
  totalInterruptStepsTaken = 0;
  interrupts();
  Serial.println("[DIAG] total ISR step counter reset");
  server.send(200, "text/plain", "OK");
}

void setupWeb() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/stepper/stop", HTTP_POST, handleStepperStop);
  server.on("/stepper/move", HTTP_POST, handleStepperMove);
  server.on("/stepper/single", HTTP_POST, handleStepperSingleStep);
  server.on("/stepper/speed", HTTP_POST, handleStepperSpeed);
  server.on("/stepper/accel", HTTP_POST, handleStepperAccel);
  server.on("/indexer/step", HTTP_POST, handleIndexerStep);
  server.on("/indexer/set_gears", HTTP_POST, handleIndexerSetGears);
  server.on("/indexer/zero", HTTP_POST, handleIndexerZero);
  server.on("/indexer/set_position_deg", HTTP_POST, handleIndexerSetPositionDeg);
  server.on("/indexer/set_position_gear", HTTP_POST, handleIndexerSetPositionGear);
  server.on("/settings/backlash", HTTP_POST, handleSetBacklash);
  server.on("/settings/slop", HTTP_POST, handleSetSlop);
  server.on("/settings/gear_geometry", HTTP_POST, handleSetGearGeometry);
  server.on("/settings/stepper_port", HTTP_POST, handleSetStepperPort);
  server.on("/preset/save", HTTP_POST, handlePresetSave);
  server.on("/preset/load", HTTP_POST, handlePresetLoad);
  server.on("/move/config", HTTP_POST, handleMoveConfig);
  server.on("/config/network", HTTP_POST, handleSaveNetworkConfig);
  server.on("/diag/reset_isd", HTTP_POST, handleDiagResetIsd);
  server.on("/diag/reset_isd_port", HTTP_POST, handleDiagResetIsdPort);
  server.on("/diag/bridge_mode", HTTP_POST, handleDiagBridgeMode);
  server.on("/diag/single_step", HTTP_POST, handleDiagSingleStep);
  server.on("/diag/test_backlash", HTTP_POST, handleDiagTestBacklash);
  server.on("/diag/reset_step_total", HTTP_POST, handleDiagResetStepTotal);
  server.begin();
}
