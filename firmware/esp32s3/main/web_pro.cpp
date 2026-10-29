// Copyright (C) 2026 Matthew Armstrong (https://github.com/The-Dorkknight)
// SPDX-License-Identifier: GPL-3.0-only. No warranty. Work in progress (0.1.0-wip).
// GET /tune: the CAPTAIN / SKIPPER tuning page (2026-10-24): input shaping with the accelerometer
// (find it, pick the axis, run, save), pressure advance, bed mesh (a coloured text heightmap), screw
// tilt, excluding objects during a print, and the motion benchmark. Everything goes through the
// console commands (/gcode) and /status.json, /accel.json, /bench; nothing here moves anything itself.
#include "esp_http_server.h"
#include "sdkconfig.h"
#include "sf.h"

#if CONFIG_SKIPPER_PRO
static const char TUNE_HTML[] =
    R"(<!doctype html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Skipper Flipper tuning</title><style>
body{background:#e9e6dc;color:#141414;font:16px/1.45 Georgia,serif;margin:0}
.b{background:#141414;color:#e9e6dc;text-align:center;padding:10px;font-weight:700;letter-spacing:.06em}
main{max-width:760px;margin:0 auto;padding:12px}h1{font:700 2rem Impact,Arial Black,sans-serif;border-bottom:4px solid #141414;margin:.3em 0}
h2{font:700 1.3rem Impact,Arial Black,sans-serif;margin:1.2em 0 .3em;letter-spacing:.03em}
.c{background:#f7f5ee;border:2px solid #141414;padding:10px;margin:8px 0}
button{font:700 15px Georgia,serif;background:#141414;color:#e9e6dc;border:2px solid #141414;padding:7px 12px;margin:3px 4px 3px 0;cursor:pointer}
button.y{background:#ffd400;color:#141414}button:disabled{opacity:.45}
input{font:inherit;width:5em;padding:3px;border:2px solid #141414;background:#fff}
small{color:#555}pre{background:#141414;color:#e9e6dc;padding:8px;overflow-x:auto;font:14px/1.25 ui-monospace,Menlo,Consolas,monospace;margin:6px 0}
table{border-collapse:collapse;width:100%}td,th{border:1px solid #141414;padding:3px 6px;text-align:left;font-size:14px}
.st{font-weight:700}.l0{color:#4d6bff}.l1{color:#5fc8ff}.l2{color:#45d06a}.l3{color:#ffad33}.l4{color:#ff4d4d}
.leg span{display:inline-block;margin-right:10px;font:700 13px ui-monospace,Menlo,monospace;background:#141414;padding:1px 5px}
.ok{color:#1d7a2e}.bad{color:#b3141c}
</style></head><body>)" SF_NAV R"(<div class="b">)" SF_BANNER R"(</div><main><h1>Tuning</h1>
<p><small>Everything here is new (2026-10-24) and has only run in simulation and on the PC. Stay with the printer;
STOP is in the left bar. Home first (Printer page) for anything that moves.</small></p>

<h2>Input shaping</h2><div class="c">
<div>Accelerometer: <span id="chip" class="st">...</span></div>
<p><button class="y" id="find">1. Find accelerometer</button> <label><input type="checkbox" id="usb" style="width:auto"> also the USB-C port
(a Klipper USB accelerometer; the USB log then stops until a restart)</label> <button id="read">Read it</button> <span id="rd"></span></p>
<details><summary><small>Wiring (any of these)</small></summary><small>
<b>Easiest:</b> an ADXL345 or MPU-6050 board on the OLED's two wires, SDA GPIO8 and SCL GPIO9, plus 3V3 and GND
(ADXL345: tie CS to 3V3 for I2C). <b>ADXL345 on SPI</b> (Klipper's way): CS, SCL/SCK, SDA/SDI, SDO to any four spare pins,
plus 3V3 and GND. <b>Klipper USB accelerometer</b> (BTT ADXL345 V2, KUSBA): into the ESP32's USB-C port with an OTG adapter,
the ESP32 powered from the printer board's 5 V. The search drives the spare pins briefly: connect nothing else to them.
</small></details>
<p><b>2. Which axis?</b> Fix the sensor firmly to the part that moves along it: <b>X</b> = the print head;
<b>Y</b> = the bed on a bed-slinger (Tina 2, Ender 3), else the head.</p>
<p><button class="y" id="tx">Test X</button><button class="y" id="ty">Test Y</button>
<small>accel per Hz <input id="aph" value="60"> (60 is Klipper's; small motors that skip: try 30)</small></p>
<div id="prog" class="st"></div><pre id="spec" style="display:none"></pre>
<div id="res"></div>
<p><small>In use now: X <span id="isx">-</span>, Y <span id="isy">-</span></small>
<button id="offx">X off</button><button id="offy">Y off</button></p></div>

<h2>Pressure advance</h2><div class="c">
<p>Now: <span id="pa" class="st">-</span> (smooth time <span id="pas">-</span> s)</p>
<p><input id="pav" value="0.05"> <button id="pas1">Set</button> <button class="y" id="pas2">Save in printer.cfg</button></p>
<p><small>Finding the value: in a print, <code>TUNING_TOWER COMMAND=SET_PRESSURE_ADVANCE PARAMETER=ADVANCE START=0 FACTOR=.005</code>
in the start G-code makes it rise with the height (Klipper's square tower test); measure where the corners look best:
ADVANCE = 0.005 x that height in mm.</small></p></div>

<h2>Bed mesh</h2><div class="c">
<p><button class="y" id="mcal">Probe mesh</button> <button id="msave">Save</button> <button id="mload">Load saved</button>
<button id="mclr">Clear</button> <span id="mst"></span></p>
<pre id="mesh">no mesh</pre>
<div class="leg"><span class="l4">high</span><span class="l3">medium-high</span><span class="l2">base</span><span class="l1">medium-low</span><span class="l0">low</span></div>
<p><small>Values in mm, the back of the bed on top. Needs [bed_mesh] and [probe] in printer.cfg; the mesh is used for moves until cleared.</small></p></div>

<h2>Screw tilt</h2><div class="c">
<p><button class="y" id="scr">Measure screws</button> <span id="sst"></span></p><div id="screws"></div>
<p><small>Needs [screws_tilt_adjust] (screw1, screw2, ... and screw_thread). 01:20 = one full turn and 20 minutes (a third of a turn).</small></p></div>

<h2>Objects in this print</h2><div class="c" id="objs"><small>None: the slicer must label objects (PrusaSlicer / OrcaSlicer:
"Label objects: firmware-specific", Klipper flavour).</small></div>

<h2>Motion benchmark</h2><div class="c"><p><button id="bench">Run</button> <small>How much of the ESP32's processor moves
take, with and without input shaping. Not while printing; about 20 s.</small></p><pre id="bo" style="display:none"></pre></div>

<h2>Print benchmark</h2><div class="c"><p><button class="y" id="pb">Run the print benchmark</button> <small>A whole print
(sf-bench.gcode: 12 layers, about 90 s) through the real motion pipeline, timed stage by stage. <b>Nothing moves or
heats:</b> with the printer board connected, the stepper drivers stay off (its step queues are really used);
without one, the ESP32 runs it alone. The first run writes the file to the SD card. Results also go to the
important-events log (Log page).</small></p><pre id="pbo" style="display:none"></pre></div>
</main><script>
var $=function(i){return document.getElementById(i)};
function g(c,cb){return fetch('/gcode?cmd='+encodeURIComponent(c),{cache:'no-store'}).then(function(r){return r.text()}).then(function(t){if(cb)cb(t);else if(t.indexOf('error')==0)alert(t);return t})}
var LV=['l0','l1','l2','l3','l4'];
function meshText(m){if(!m)return 'no mesh';var s='range '+m.range.toFixed(3)+' mm\n',w=Math.max(6,1);
for(var r=m.rows-1;r>=0;r--){var y=m.min[1]+(m.max[1]-m.min[1])*r/(m.rows-1);s+=('y'+y.toFixed(0)).padStart(5)+' ';
for(var c=0;c<m.cols;c++){var i=r*m.cols+c,v=m.z[i];s+='<span class="'+LV[m.lvl[i]]+'">'+((v>=0?'+':'')+v.toFixed(3)).padStart(7)+'</span>'}s+='\n'}
s+='      ';for(var c=0;c<m.cols;c++){var x=m.min[0]+(m.max[0]-m.min[0])*c/(m.cols-1);s+=('x'+x.toFixed(0)).padStart(7)}return s}
function bars(sp){if(!sp)return '';var mx=0;sp.forEach(function(p){if(p[1]>mx)mx=p[1]});var s='';
sp.forEach(function(p){var n=mx>0?Math.round(p[1]/mx*25):0;s+=(p[0].toFixed(1)+' Hz').padStart(9)+' '+'#'.repeat(n)+'\n'});return s}
var lastAxis='x';
function shaperTable(r){if(!r)return '';var s='<table><tr><th>shaper</th><th>Hz</th><th>vibrations left</th><th>smoothing</th><th>max accel</th></tr>';
r.shapers.forEach(function(x){var b=x.name==r.best;s+='<tr'+(b?' style="background:#ffd400"':'')+'><td>'+x.name+(b?' (best)':'')+'</td><td>'+x.freq.toFixed(1)+'</td><td>'+(x.vibrs*100).toFixed(1)+' %</td><td>'+x.smoothing.toFixed(3)+'</td><td>'+x.max_accel+'</td></tr>'});
s+='</table><p><button class="y" id="isave">3. Save '+r.axis.toUpperCase()+': '+r.best+' '+r.freq.toFixed(1)+' Hz</button> <small>then keep max_accel at or below '+r.max_accel+' mm/s&sup2;</small></p>';return s}
function acc(){fetch('/accel.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
$('chip').textContent=j.found?j.chip:'none found yet';$('find').disabled=j.busy;
var p=j.busy&&j.state=='idle'?j.scan:(j.state!='idle'&&j.state!='done'?j.state:'');if(j.error)p='<span class="bad">'+j.error+'</span>';
if(!j.busy&&j.scan&&j.scan.indexOf('Found')==0&&!j.result)p=j.scan;$('prog').innerHTML=p;
if(j.spectrum){$('spec').style.display='';$('spec').textContent=bars(j.spectrum)}
if(j.result){$('res').innerHTML=shaperTable(j.result);var b=$('isave');if(b)b.onclick=function(){var r=j.result,A=r.axis.toUpperCase();
g('SET_INPUT_SHAPER SHAPER_TYPE_'+A+'='+r.best+' SHAPER_FREQ_'+A+'='+r.freq.toFixed(1)+' SAVE=1',function(t){alert(t)})}}
setTimeout(acc,j.busy||j.state!='done'&&j.state!='idle'?700:3000)}).catch(function(){setTimeout(acc,3000)})}
function st(){fetch('/status.json',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){var m=j.machine;if(!m)return;
var th=m.toolhead||{};$('isx').textContent=th.shaper_x||'off';$('isy').textContent=th.shaper_y||'off';
$('pa').textContent=th.pa!==undefined?th.pa.toFixed(4):'-';$('pas').textContent=th.pa_smooth!==undefined?th.pa_smooth.toFixed(3):'-';
(m.tests||[]).forEach(function(t){if((t.id=='res_x'||t.id=='res_y')&&t.state=='running')$('prog').textContent='Resonance test '+t.id.slice(4).toUpperCase()+': '+t.detail;
if((t.id=='res_x'||t.id=='res_y')&&t.state=='failed'&&t.detail)$('prog').innerHTML='<span class="bad">'+t.detail+'</span>'});
$('mesh').innerHTML=meshText(m.mesh);var pr=m.probing||{};
$('mst').textContent=pr.active&&pr.kind=='mesh'?'probing '+(pr.point+1)+' of '+pr.points:(pr.kind=='mesh'&&pr.error?pr.error:'');
$('sst').textContent=pr.active&&pr.kind=='screws'?'probing '+(pr.point+1)+' of '+pr.points:(pr.kind=='screws'&&pr.error?pr.error:'');
if(m.screws){var s='<table><tr><th>screw</th><th>z</th><th>turn</th></tr>';m.screws.forEach(function(x){
s+='<tr><td>'+x.name+'</td><td>'+x.z.toFixed(3)+'</td><td class="st">'+(x.base?'base':(x.adjust=='00:00'?'level':x.sign+' '+x.adjust))+'</td></tr>'});$('screws').innerHTML=s+'</table>'}
var o=m.objects;if(o&&o.list.length){var s='';o.list.forEach(function(x){s+='<div>'+(x.name==o.current?'&#9654; ':'')+x.name+' '+
(x.excluded?'<span class="bad">excluded</span>':'<button data-n="'+x.name+'">Exclude</button>')+'</div>'});$('objs').innerHTML=s;
Array.prototype.forEach.call($('objs').querySelectorAll('button'),function(b){b.onclick=function(){if(confirm('Skip the rest of '+b.dataset.n+'?'))g('EXCLUDE_OBJECT NAME='+b.dataset.n)}})}
}).catch(function(){}).then(function(){setTimeout(st,1000)})}
$('find').onclick=function(){fetch('/accel?scan=1'+($('usb').checked?'&usb=1':''));$('prog').textContent='searching...';setTimeout(acc,300)};
$('read').onclick=function(){$('rd').textContent='...';fetch('/accel?read=1',{cache:'no-store'}).then(function(r){return r.json()}).then(function(j){
$('rd').textContent=j.error?j.error:'x '+j.x+'  y '+j.y+'  z '+j.z+' mm/s² (total '+j.g+'; still = about 9807)'})};
function test(a){var v=parseFloat($('aph').value)||60;$('res').innerHTML='';g('SHAPER_CALIBRATE AXIS='+a+' ACCEL_PER_HZ='+v,function(t){$('prog').textContent=t;setTimeout(acc,500)})}
$('tx').onclick=function(){test('X')};$('ty').onclick=function(){test('Y')};
$('offx').onclick=function(){g('SET_INPUT_SHAPER SHAPER_FREQ_X=0')};$('offy').onclick=function(){g('SET_INPUT_SHAPER SHAPER_FREQ_Y=0')};
$('pas1').onclick=function(){g('SET_PRESSURE_ADVANCE ADVANCE='+$('pav').value)};
$('pas2').onclick=function(){g('SET_PRESSURE_ADVANCE ADVANCE='+$('pav').value+' SAVE=1',function(t){alert(t)})};
$('mcal').onclick=function(){g('BED_MESH_CALIBRATE')};$('msave').onclick=function(){g('BED_MESH_PROFILE SAVE=default',function(t){alert(t)})};
$('mload').onclick=function(){g('BED_MESH_PROFILE LOAD=default')};$('mclr').onclick=function(){g('BED_MESH_CLEAR')};
$('scr').onclick=function(){g('SCREWS_TILT_CALCULATE')};
$('bench').onclick=function(){$('bo').style.display='';$('bo').textContent='running...';fetch('/bench?run=1');
(function w(){fetch('/bench',{cache:'no-store'}).then(function(r){return r.text()}).then(function(t){$('bo').textContent=t;if(t.indexOf('running')==0)setTimeout(w,2000)})})()};
$('pb').onclick=function(){$('pbo').style.display='';$('pbo').textContent='starting...';g('SF_BENCH',function(t){$('pbo').textContent=t;
if(t.indexOf('ok benchmark running')==0)(function w(){setTimeout(function(){g('SF_BENCH_RESULT',function(r){
if(r.indexOf('still running')>0||r.indexOf('no benchmark')>0){$('pbo').textContent=t+'\n... running';w()}else $('pbo').textContent=r.replace(/\. /g,'.\n')})},3000)})()})};
acc();st();
</script></body></html>)";
#else
static const char TUNE_HTML[] =
    "<!doctype html><html><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\">"
    "<title>Skipper Flipper tuning</title></head><body style=\"background:#e9e6dc;font:16px Georgia,serif\">" SF_NAV
    "<main style=\"padding:16px\"><h1>Tuning</h1><p>Input shaping, pressure advance, bed mesh and screw tilt need the "
    "CAPTAIN or SKIPPER build (4 MB flash or more): this DECKHAND build has no room for them. "
    "<a href=\"/about\">About this build</a></p></main></body></html>";
#endif

static esp_err_t tune_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "max-age=30");
    return httpd_resp_send(req, TUNE_HTML, HTTPD_RESP_USE_STRLEN);
}

extern "C" void sf_tune_register(void *httpd)
{
    static const httpd_uri_t u = {.uri = "/tune", .method = HTTP_GET, .handler = tune_get, .user_ctx = NULL};
    httpd_register_uri_handler((httpd_handle_t)httpd, &u);
}
