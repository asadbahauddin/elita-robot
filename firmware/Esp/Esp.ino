/*
 * ============================================================
 *  ELITA ROBOT — ESP32 DevKit v1 WiFi Bridge  — v4.3 (koneksi tahan putus + stopwatch)
 *
 *  Board    : ESP32 Dev Module
 *  Libraries: WebSockets by Markus Sattler (Links2004) — Library Manager
 *             WiFi, WebServer — bawaan core ESP32
 *
 *  WiFi AP  : SSID=ElitaRobot, pass=elita1234, IP=192.168.4.1 (default AP)
 *  Web      : http://192.168.4.1        (dashboard)
 *  WS       : ws://192.168.4.1:81       (Python & browser)
 *  API      : POST /cmd  (body = JSON command), GET /status
 *
 *  WIRING ESP32 <-> Uno (tidak berubah):
 *    GPIO16 (RX2) <- Uno A0 TX  [divider: A0 - 1k - node - 2k - GND]
 *    GPIO17 (TX2) -> Uno A1 RX  [langsung]
 *    GND          -- Uno GND    [WAJIB]
 * ============================================================
 */

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ============================================================
// KONFIGURASI
// ============================================================
const char* AP_SSID = "ElitaRobot";
const char* AP_PASS = "elita1234";
#define AP_CHANNEL        6
#define AP_MAX_CLIENTS    4

#define UNO_RX_PIN       16
#define UNO_TX_PIN       17
#define UNO_BAUD      19200
#define UNO_TIMEOUT_MS 1500    // Uno kirim telemetri min. tiap 500 ms
#define UNO_LINE_MAX    240    // diag v4 bisa > 200 byte
#define UNO_LINE_STALE   30    // ms, buang baris setengah jadi

#define FWD_MAX_LEN      72    // buffer RX Uno 80 byte
#define UART_IDLE_GAP_MS  4    // jalur RX dari Uno harus diam sebelum kirim
#define TXQ_LEN           6
#define STATS_MS       5000

// ============================================================
// OBJEK & STATE
// ============================================================
HardwareSerial   unoSerial(2);
WebServer        server(80);
WebSocketsServer ws(81);

String        unoBuf;
unsigned long lastUnoLineMs = 0;
unsigned long lastUnoByteMs = 0;
bool          unoOnline     = false;

uint32_t statUnoLines = 0, statUnoBad = 0;
uint32_t statFwdOk = 0, statFwdRejected = 0, statFwdDropped = 0;
unsigned long lastStatsMs = 0;
unsigned long lastKeepMs  = 0;

bool checkClients = false;

String  txq[TXQ_LEN];
uint8_t qHead = 0, qCount = 0;

// ============================================================
// HTML DASHBOARD
// ============================================================
static const char DASHBOARD[] PROGMEM = R"rawhtml(
<!DOCTYPE html>
<html lang="id">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>ELITA Robot</title>
<style>
  :root{
    --bg:#0f1117;--card:#1a1d27;--border:#2a2d3e;
    --accent:#4f8ef7;--accent2:#38d9a9;--warn:#ffd43b;
    --text:#e2e8f0;--muted:#64748b;--red:#f87171;--green:#4ade80;
    --purple:#c084fc;--orange:#fb923c;
  }
  *{box-sizing:border-box;margin:0;padding:0}
  body{background:var(--bg);color:var(--text);font-family:'Segoe UI',monospace;min-height:100vh}
  header{padding:16px 24px;border-bottom:1px solid var(--border);display:flex;align-items:center;gap:12px;background:var(--card);flex-wrap:wrap}
  .dot{width:10px;height:10px;border-radius:50%;background:var(--red)}
  .dot.on{background:var(--green);animation:pulse 2s infinite}
  .dot.busy{background:var(--orange);animation:pulse 1s infinite}
  @keyframes pulse{0%,100%{opacity:1}50%{opacity:.4}}
  h1{font-size:18px;font-weight:600}
  .badge{font-size:11px;padding:3px 9px;border-radius:20px;border:1px solid var(--border);color:var(--muted)}
  .badge.on{border-color:var(--green);color:var(--green)}
  .badge.off{border-color:var(--red);color:var(--red)}
  .badge.warn{background:var(--red);border-color:var(--red);color:#fff;font-weight:700;display:none}
  .badge.warn.show{display:inline-block}
  .badge.cut{background:var(--orange);border-color:var(--orange);color:#000;font-weight:700;display:none}
  .badge.cut.show{display:inline-block}
  .tag{font-size:11px;color:var(--muted);margin-left:auto}
  .grid{display:grid;grid-template-columns:1fr 1fr;gap:16px;padding:20px;max-width:960px;margin:0 auto}
  @media(max-width:600px){.grid{grid-template-columns:1fr;padding:12px}}
  .card{background:var(--card);border:1px solid var(--border);border-radius:12px;padding:18px}
  .card.full{grid-column:span 2}
  @media(max-width:600px){.card.full{grid-column:span 1}}
  .card-title{font-size:11px;color:var(--muted);text-transform:uppercase;letter-spacing:1px;margin-bottom:12px}
  .kv{display:grid;grid-template-columns:1fr 1fr;gap:10px 16px}
  .k{font-size:12px;color:var(--muted)}
  .v{font-size:20px;font-weight:700;font-family:monospace;color:var(--accent)}
  .v.ok{color:var(--green)} .v.bad{color:var(--red)} .v.cyan{color:var(--accent2)}
  .btn-grid{display:grid;grid-template-columns:1fr 1fr 1fr;gap:8px;margin-bottom:12px}
  .btn{padding:12px 10px;border:1px solid var(--border);border-radius:8px;background:var(--bg);color:var(--text);cursor:pointer;font-size:13px;transition:background .15s;touch-action:manipulation}
  .btn:hover{background:var(--accent);border-color:var(--accent);color:#fff}
  .btn:disabled{opacity:.4;cursor:not-allowed}
  .btn.stop{border-color:var(--red);color:var(--red)}
  .btn.stop:hover{background:var(--red);color:#fff}
  .btn.on{background:var(--red);border-color:var(--red);color:#fff}
  .pwm-row{display:flex;align-items:center;gap:10px;margin-top:8px}
  .pwm-row label{font-size:12px;color:var(--muted);min-width:36px}
  input[type=range]{flex:1;accent-color:var(--accent)}
  .pwm-val{font-size:13px;font-weight:600;min-width:32px;text-align:right}
  .servo-wrap{text-align:center}
  .servo-angle-label{font-size:28px;font-weight:700;color:var(--accent2);margin-bottom:12px;display:block}
  input[type=range].servo-slider{width:100%;accent-color:var(--accent2)}
  .scale{display:flex;justify-content:space-between;font-size:11px;color:var(--muted);margin-top:4px}
  .row-btns{display:flex;gap:8px;margin-top:10px}
  .row-btns .btn{flex:1;font-size:12px}
  .cutter-btn{width:100%;padding:16px;border-radius:12px;border:none;color:#fff;font-size:16px;font-weight:700;
    cursor:pointer;letter-spacing:1px;background:linear-gradient(135deg,#475569,#334155);transition:opacity .2s;touch-action:manipulation}
  .cutter-btn.on{background:linear-gradient(135deg,#ea580c,#c2410c)}
  .cutter-btn:disabled{opacity:.4;cursor:not-allowed}
  .estop-btn{width:100%;padding:16px;border-radius:12px;border:2px solid var(--red);background:transparent;color:var(--red);
    font-size:16px;font-weight:800;letter-spacing:2px;cursor:pointer;margin-top:10px;touch-action:manipulation}
  .estop-btn:hover{background:var(--red);color:#fff}
  canvas{display:block;width:100%;height:120px;background:var(--bg);border:1px solid var(--border);border-radius:8px}
  .legend{display:flex;gap:14px;font-size:11px;color:var(--muted);margin-top:6px}
  .legend i{display:inline-block;width:10px;height:3px;margin-right:5px;vertical-align:middle}
  .note{font-size:11px;color:var(--muted);text-align:center;margin-top:8px}
  .status{text-align:center;font-size:11px;color:var(--muted);padding:10px}
  #last-update{color:var(--accent2)}
  .diag-grid{display:grid;grid-template-columns:repeat(4,1fr);gap:8px}
  @media(max-width:600px){.diag-grid{grid-template-columns:repeat(2,1fr)}}
  .diag-grid .btn{font-size:12px;padding:10px 6px}
  .btn.sv{border-color:var(--accent2);color:var(--accent2)}
  .btn.sv:hover{background:var(--accent2);color:#000}
  .diag-out{margin-top:12px;background:var(--bg);border:1px solid var(--border);border-radius:8px;
    padding:10px;font-family:monospace;font-size:12px;line-height:1.5;color:var(--accent2);
    white-space:pre-wrap;word-break:break-word;min-height:70px;max-height:260px;overflow:auto}
  .diag-warn{color:var(--warn)}
  .sw-top{display:flex;align-items:baseline;gap:16px;flex-wrap:wrap}
  .sw-time{font-family:monospace;font-size:44px;font-weight:700;color:var(--muted)}
  .sw-time.run{color:var(--green)}
  .sw-mode{font-size:14px;color:var(--muted)}
  .sw-btns{display:flex;gap:8px;margin-left:auto}
  .sw-btns .btn{font-size:12px}
  table.log{width:100%;border-collapse:collapse;margin-top:12px;font-family:monospace;font-size:12px}
  table.log th,table.log td{padding:6px 8px;border-bottom:1px solid var(--border);text-align:left}
  table.log th{color:var(--muted);font-weight:600}
  table.log td.num{text-align:right}
  .sw-avg{margin-top:10px;font-family:monospace;font-size:12px;color:var(--accent2);white-space:pre-wrap}
</style>
</head>
<body>
<header>
  <div class="dot" id="status-dot"></div>
  <h1>ELITA Robot</h1>
  <span class="badge off" id="b-ws">WS</span>
  <span class="badge off" id="b-uno">UNO</span>
  <span class="badge cut" id="b-cut">CUTTER ON</span>
  <span class="badge warn" id="b-wd">WATCHDOG &mdash; tekan STOP</span>
  <span class="tag">192.168.4.1</span>
</header>

<div class="grid">

  <!-- STATUS -->
  <div class="card">
    <div class="card-title">Status Robot</div>
    <div class="kv">
      <div><div class="k">Motor Kiri</div><div class="v" id="v-mk">&mdash;</div></div>
      <div><div class="k">Motor Kanan</div><div class="v" id="v-mr">&mdash;</div></div>
      <div><div class="k">Motor (ON/OFF)</div><div class="v cyan" id="v-ms">&mdash;</div></div>
      <div><div class="k">Uptime Uno</div><div class="v cyan" id="v-up">&mdash;</div></div>
      <div><div class="k">Watchdog</div><div class="v" id="v-wd">&mdash;</div></div>
      <div><div class="k">Frame Rusak</div><div class="v" id="v-bf">&mdash;</div></div>
      <div><div class="k">Encoder Kiri</div><div class="v cyan" id="v-el">&mdash;</div><div class="k" id="v-elr">&nbsp;</div></div>
      <div><div class="k">Encoder Kanan</div><div class="v cyan" id="v-er">&mdash;</div><div class="k" id="v-err">&nbsp;</div></div>
    </div>
  </div>

  <!-- MOTOR -->
  <div class="card">
    <div class="card-title">Motor Drive (Ackerman)</div>
    <div class="btn-grid">
      <div></div>
      <button class="btn" id="btn-fwd" onclick="toggleDir('fwd')">&#9650; Maju</button>
      <div></div>
      <button class="btn" id="btn-lft" onclick="toggleDir('lft')">&#9664; Kiri</button>
      <button class="btn stop" onclick="stopMotor()">STOP</button>
      <button class="btn" id="btn-rgt" onclick="toggleDir('rgt')">Kanan &#9654;</button>
      <div></div>
      <button class="btn" id="btn-rev" onclick="toggleDir('rev')">&#9660; Mundur</button>
      <div></div>
    </div>
    <div class="pwm-row">
      <label>PWM</label>
      <input type="range" id="pwm-slider" min="0" max="255" value="255">
      <span class="pwm-val" id="pwm-disp">255</span>
    </div>
    <div class="note">Tekan arah yang sama sekali lagi untuk berhenti<br>
      Motor ON/OFF (ENB di D9 tidak bisa PWM karena Timer1 dipakai Servo) &mdash; PWM 0 = berhenti</div>
  </div>

  <!-- STOPWATCH -->
  <div class="card full">
    <div class="card-title">Stopwatch Kalibrasi &mdash; mulai otomatis saat tombol arah, berhenti saat STOP</div>
    <div class="sw-top">
      <span class="sw-time" id="sw-time">0.00 s</span>
      <span class="sw-mode" id="sw-mode">siap</span>
      <div class="sw-btns">
        <button class="btn" onclick="swReset()">Reset Timer</button>
        <button class="btn" onclick="swClearLog()">Hapus Log</button>
        <button class="btn" onclick="swCsv()">Unduh CSV</button>
      </div>
    </div>
    <table class="log">
      <thead><tr><th>#</th><th>Gerak</th><th>Servo</th><th class="num">Waktu (s)</th><th>Jam</th></tr></thead>
      <tbody id="sw-log"><tr><td colspan="5" style="color:var(--muted)">Belum ada percobaan</td></tr></tbody>
    </table>
    <div class="sw-avg" id="sw-avg"></div>
    <div class="note">Ganti arah tanpa STOP = percobaan lama dicatat &amp; timer mulai lagi.
      E-STOP / watchdog / koneksi putus juga menghentikan timer.<br>
      Waktu diukur di browser: jeda WiFi di awal &amp; akhir hampir saling menghapus (&plusmn;0,1 s).</div>
  </div>

  <!-- SERVO STEERING -->
  <div class="card">
    <div class="card-title">Servo Steering MG996R &mdash; D12</div>
    <div class="servo-wrap">
      <svg width="140" height="70" viewBox="0 0 140 70" style="margin-bottom:4px">
        <path d="M10,70 A60,60 0 0,1 130,70" fill="none" stroke="#2a2d3e" stroke-width="8" stroke-linecap="round"/>
        <path d="M10,70 A60,60 0 0,1 130,70" fill="none" stroke="#4f8ef7" stroke-width="4" stroke-linecap="round" opacity=".5"/>
        <line id="servo-needle" x1="70" y1="70" x2="70" y2="15" stroke="#38d9a9" stroke-width="3" stroke-linecap="round"/>
        <circle cx="70" cy="70" r="5" fill="#38d9a9"/>
      </svg>
      <span class="servo-angle-label" id="servo-disp">75&deg;</span>
      <input type="range" class="servo-slider" id="servo-slider" min="45" max="110" value="75">
      <div class="scale"><span>45&deg; Kiri</span><span>75&deg; Center</span><span>110&deg; Kanan</span></div>
      <div class="row-btns">
        <button class="btn" onclick="setServo(SERVO_L,true)">&#9664; Kiri Max</button>
        <button class="btn" onclick="setServo(SERVO_C,true)">Center</button>
        <button class="btn" onclick="setServo(SERVO_R,true)">Kanan Max &#9654;</button>
      </div>
      <div class="note">Servo aktual (telemetri): <span id="servo-actual" style="color:var(--accent2);font-weight:700">&mdash;</span></div>
    </div>
  </div>

  <!-- CUTTER -->
  <div class="card">
    <div class="card-title">Cutter (Relay) &mdash; A5</div>
    <button class="cutter-btn" id="btn-cutter" onclick="toggleCutter()">CUTTER OFF</button>
    <button class="estop-btn" onclick="eStop()">&#9632; EMERGENCY STOP</button>
    <div class="note">E-STOP mematikan motor + cutter dan mereset latch watchdog.<br>
      Robot otomatis STOP bila tab ditutup / HP dikunci.</div>
  </div>

  <!-- DIAGNOSTIK -->
  <div class="card full">
    <div class="card-title">Diagnostik Hardware &mdash; angkat roda dulu!</div>
    <div class="diag-grid">
      <button class="btn" onclick="hwTest('q')">Kiri MAJU</button>
      <button class="btn" onclick="hwTest('a')">Kiri MUNDUR</button>
      <button class="btn" onclick="hwTest('e')">Kanan MAJU</button>
      <button class="btn" onclick="hwTest('d')">Kanan MUNDUR</button>
      <button class="btn" onclick="hwTest('w')">Dua MAJU</button>
      <button class="btn" onclick="hwTest('s')">Dua MUNDUR</button>
      <button class="btn stop" onclick="hwTest('x')">STOP</button>
      <button class="btn" onclick="hwTest('p')">Baca Pin</button>
      <button class="btn sv" onclick="hwTest('j')">Servo 45&deg;</button>
      <button class="btn sv" onclick="hwTest('k')">Servo 75&deg;</button>
      <button class="btn sv" onclick="hwTest('l')">Servo 110&deg;</button>
      <button class="btn sv" onclick="hwTest('v')">Servo Sweep</button>
      <button class="btn" onclick="hwTest('r')">Reset Encoder</button>
    </div>
    <div class="diag-out" id="diag-out">Belum ada data. Tekan "Baca Pin" untuk cek Uno sudah v3.2.</div>
    <div class="note">Motor tes jalan 2 detik lalu berhenti sendiri &middot; cutter dipaksa OFF &middot; butuh Uno v4<br>
      Kalibrasi encoder: Reset Encoder &rarr; putar roda TEPAT 1 putaran dengan tangan &rarr; Baca Pin &rarr; catat angkanya</div>
  </div>

  <!-- CHART -->
  <div class="card full">
    <div class="card-title">Live Telemetri</div>
    <canvas id="chart"></canvas>
    <div class="legend"><span><i style="background:#4f8ef7"></i>PWM (0&ndash;255)</span>
      <span><i style="background:#38d9a9"></i>Servo (45&ndash;110&deg;)</span>
      <span><i style="background:#fb923c"></i>Cutter</span></div>
  </div>

</div>
<div class="status">Last update: <span id="last-update">-</span> &nbsp;|&nbsp; <span id="log">menunggu koneksi...</span></div>

<script>
const SERVO_C = 75, SERVO_L = 45, SERVO_R = 110;
const HB_MS = 250, TEL_STALE_MS = 1500, MAX_PT = 80;

let ws = null, wsOpen = false, lastTel = 0, lastSend = 0, unoOn = false;
let st = { mk: 0, mr: 0, ms: 255, sv: SERVO_C, ct: 0 };
let encPrev = null;
let activeDir = null;
const hist = { ms: [], sv: [], ct: [] };

const $ = id => document.getElementById(id);
function log(m) { $('log').textContent = m; }
function isActive() { return !!(st.mk || st.mr || st.ct); }

// ---------- koneksi ----------
// Koneksi dibuat tahan "koneksi setengah mati" (HP tidur / pindah WiFi):
// - ESP32 kirim keepalive {"t":"st"} tiap 1 s -> kalau 4 s tanpa pesan apa pun,
//   socket dianggap mati, ditutup paksa, lalu reconnect.
// - Saat tab kembali terlihat, koneksi dicek & disambung ulang bila perlu.
let lastMsg = 0, reconnTimer = null;
function scheduleReconnect(ms) {
  clearTimeout(reconnTimer);
  reconnTimer = setTimeout(connect, ms);
}
function connect() {
  clearTimeout(reconnTimer); reconnTimer = null;
  if (ws && (ws.readyState === WebSocket.OPEN || ws.readyState === WebSocket.CONNECTING)) return;
  let sock;
  try { sock = new WebSocket('ws://' + location.hostname + ':81'); }
  catch (x) { scheduleReconnect(2000); return; }
  ws = sock;
  sock.onopen = () => {
    if (sock !== ws) return;
    wsOpen = true; lastMsg = Date.now(); paintHeader(); log('WebSocket terhubung');
  };
  sock.onclose = () => {
    if (sock !== ws) return;               // abaikan socket lama
    wsOpen = false; unoOn = false; localHalt(); paintHeader();
    log('Koneksi terputus - menyambung ulang...');
    scheduleReconnect(1500);
  };
  sock.onerror = () => log('WebSocket error');
  sock.onmessage = e => {
    if (sock !== ws) return;
    lastMsg = Date.now();
    let d; try { d = JSON.parse(e.data); } catch (x) { return; }
    if (d.t === 'tel') onTel(d);
    else if (d.t === 'st') { unoOn = !!d.uno; paintHeader(); }
    else if (d.t === 'err') log('ESP32: ' + d.m);
  };
}
function forceReconnect(why) {
  log(why + ' - menyambung ulang...');
  const old = ws; ws = null; wsOpen = false; unoOn = false;
  localHalt(); paintHeader();
  try { if (old) old.close(); } catch (x) {}
  scheduleReconnect(300);
}
// Pengawas koneksi
setInterval(() => {
  if (wsOpen && Date.now() - lastMsg > 4000) forceReconnect('Tidak ada data dari ESP32 > 4 s');
  else if (!wsOpen && !reconnTimer && (!ws || ws.readyState === WebSocket.CLOSED)) scheduleReconnect(0);
}, 1000);
document.addEventListener('visibilitychange', () => {
  if (!document.hidden && (!ws || ws.readyState !== WebSocket.OPEN)) scheduleReconnect(0);
});
function send(obj) {
  if (!wsOpen) { log('Tidak terhubung - perintah tidak terkirim'); return false; }
  ws.send(JSON.stringify(obj)); return true;
}
function sendDrive() {
  lastSend = Date.now();
  return send({ cmd: 'drive', mk: st.mk, mr: st.mr, ms: st.ms, sv: st.sv, ct: st.ct });
}

// ---------- header ----------
function paintHeader(wd) {
  const dot = $('status-dot');
  dot.className = 'dot' + (wsOpen && unoOn ? (isActive() ? ' busy' : ' on') : '');
  $('b-ws').className = 'badge ' + (wsOpen ? 'on' : 'off');
  $('b-uno').className = 'badge ' + (unoOn ? 'on' : 'off');
  $('b-cut').classList.toggle('show', !!st.ct);
  if (wd !== undefined) $('b-wd').classList.toggle('show', !!wd);
}

// ---------- motor (toggle ala NISA) ----------
const dirs = {
  fwd: { id: 'btn-fwd', label: '▲ Maju',   stop: '■ STOP MAJU',   mk: 1,  sv: SERVO_C },
  rev: { id: 'btn-rev', label: '▼ Mundur', stop: '■ STOP MUNDUR', mk: -1, sv: SERVO_C },
  lft: { id: 'btn-lft', label: '◀ Kiri',   stop: '■ STOP KIRI',   mk: 1,  sv: SERVO_L },
  rgt: { id: 'btn-rgt', label: 'Kanan ▶',  stop: '■ STOP KANAN',  mk: 1,  sv: SERVO_R }
};
function paintDir(dir, on) {
  const b = $(dirs[dir].id);
  b.textContent = on ? dirs[dir].stop : dirs[dir].label;
  b.classList.toggle('on', on);
}
function toggleDir(dir) {
  if (activeDir === dir) { stopMotor(); return; }
  if (activeDir) paintDir(activeDir, false);
  activeDir = dir; paintDir(dir, true);
  st.mk = st.mr = dirs[dir].mk;
  st.ms = parseInt($('pwm-slider').value);
  setServo(dirs[dir].sv, false);
  if (sendDrive()) { log('drive ' + dir + ' pwm=' + st.ms + ' servo=' + st.sv); swStart(dir); }
  paintHeader();
}
function stopMotor() {           // motor berhenti, cutter dipertahankan
  if (activeDir) paintDir(activeDir, false);
  activeDir = null; st.mk = st.mr = 0;
  sendDrive(); swStop(); paintHeader(); log('motor stop');
}

// ---------- stopwatch kalibrasi ----------
const SW_LBL = { fwd: 'Maju', rev: 'Mundur', lft: 'Kiri (belok)', rgt: 'Kanan (belok)' };
let sw = { running: false, t0: 0, last: 0, mode: null, sv: SERVO_C };
let swLog = [];
function swFmt(s) { return s.toFixed(2) + ' s'; }
function swStart(mode) {
  if (sw.running) swStop();
  sw = { running: true, t0: performance.now(), last: 0, mode: mode, sv: st.sv };
  $('sw-mode').textContent = 'berjalan: ' + SW_LBL[mode] + ' (servo ' + st.sv + '\u00B0)';
}
function swStop() {
  if (!sw.running) return;
  sw.last = (performance.now() - sw.t0) / 1000;
  sw.running = false;
  swLog.push({ n: swLog.length + 1, mode: sw.mode, sv: sw.sv, t: sw.last,
               clock: new Date().toLocaleTimeString() });
  $('sw-mode').textContent = 'berhenti: ' + SW_LBL[sw.mode];
  swRender();
}
function swReset() {
  if (sw.running) sw.t0 = performance.now(); else sw.last = 0;
  if (!sw.running) $('sw-mode').textContent = 'siap';
}
function swClearLog() { swLog = []; swRender(); }
function swRender() {
  const tb = $('sw-log');
  if (!swLog.length) {
    tb.innerHTML = '<tr><td colspan="5" style="color:var(--muted)">Belum ada percobaan</td></tr>';
    $('sw-avg').textContent = ''; return;
  }
  tb.innerHTML = swLog.slice(-30).reverse().map(r =>
    '<tr><td>' + r.n + '</td><td>' + SW_LBL[r.mode] + '</td><td>' + r.sv + '\u00B0</td>' +
    '<td class="num">' + r.t.toFixed(2) + '</td><td>' + r.clock + '</td></tr>').join('');
  const g = {};
  swLog.forEach(r => { const k = SW_LBL[r.mode]; (g[k] = g[k] || []).push(r.t); });
  $('sw-avg').textContent = 'Rata-rata:  ' + Object.keys(g).map(k => {
    const a = g[k], m = a.reduce((x, y) => x + y, 0) / a.length;
    return k + ' = ' + m.toFixed(2) + ' s (' + a.length + 'x)';
  }).join('   |   ');
}
function swCsv() {
  if (!swLog.length) { log('Log kosong'); return; }
  const rows = ['no,gerak,servo_deg,waktu_s,jam'].concat(
    swLog.map(r => [r.n, SW_LBL[r.mode], r.sv, r.t.toFixed(3), r.clock].join(',')));
  const a = document.createElement('a');
  a.href = URL.createObjectURL(new Blob([rows.join('\n')], { type: 'text/csv' }));
  a.download = 'elita_kalibrasi.csv'; a.click();
  setTimeout(() => URL.revokeObjectURL(a.href), 1000);
}
setInterval(() => {
  const t = sw.running ? (performance.now() - sw.t0) / 1000 : sw.last;
  const el = $('sw-time');
  el.textContent = swFmt(t); el.classList.toggle('run', sw.running);
}, 50);

// ---------- servo ----------
function needle(val) {
  const deg = val < SERVO_C ? (val - SERVO_C) / (SERVO_C - SERVO_L) * 90
                            : (val - SERVO_C) / (SERVO_R - SERVO_C) * 90;
  const r = deg * Math.PI / 180;
  $('servo-needle').setAttribute('x2', (70 + 55 * Math.sin(r)).toFixed(1));
  $('servo-needle').setAttribute('y2', (70 - 55 * Math.cos(r)).toFixed(1));
}
function setServo(val, sendNow) {
  val = Math.min(Math.max(parseInt(val), SERVO_L), SERVO_R);
  st.sv = val;
  $('servo-slider').value = val; $('servo-disp').textContent = val + '°'; needle(val);
  if (sendNow) sendDrive();
}

// ---------- cutter & e-stop ----------
function paintCutter() {
  $('btn-cutter').textContent = st.ct ? 'CUTTER ON' : 'CUTTER OFF';
  $('btn-cutter').classList.toggle('on', !!st.ct);
}
function toggleCutter() { st.ct = st.ct ? 0 : 1; paintCutter(); sendDrive(); paintHeader(); log('cutter ' + (st.ct ? 'ON' : 'OFF')); }
function localHalt() {
  swStop();
  if (activeDir) paintDir(activeDir, false);
  activeDir = null; st.mk = st.mr = 0; st.ct = 0; paintCutter();
}
function eStop() {
  localHalt();
  send({ cmd: 'stop' });
  setTimeout(() => send({ cmd: 'stop' }), 60);
  setTimeout(() => send({ cmd: 'stop' }), 120);
  paintHeader(); log('EMERGENCY STOP (3x)');
}

// ---------- diagnostik ----------
const DIAG_LBL = { q:'Kiri MAJU', a:'Kiri MUNDUR', e:'Kanan MAJU', d:'Kanan MUNDUR', w:'Dua MAJU',
  s:'Dua MUNDUR', x:'STOP', j:'Servo 45', k:'Servo 75', l:'Servo 110', v:'Sweep mulai',
  V:'Sweep selesai', p:'Baca pin', t:'Auto-stop 2 s', r:'Reset encoder' };
let diagLines = [], diagTimer = null;
function diagPush(txt, warn) {
  const t = new Date().toLocaleTimeString();
  diagLines.unshift((warn ? '\u26A0 ' : '') + t + '  ' + txt);
  if (diagLines.length > 8) diagLines.pop();
  $('diag-out').textContent = diagLines.join('\n\n');
}
function hwTest(a) {
  if (activeDir || st.ct) localHalt();
  if (!send({ cmd: 'test', a: a })) return;
  log('tes: ' + (DIAG_LBL[a] || a));
  clearTimeout(diagTimer);
  diagTimer = setTimeout(() => diagPush('Tidak ada balasan dari Uno untuk "' + (DIAG_LBL[a] || a) +
    '". Cek: Uno sudah di-flash v3.2? Badge UNO hijau?', true), 1500);
}
function onDiag(d) {
  clearTimeout(diagTimer);
  diagPush('[' + (DIAG_LBL[d.a] || d.a) + ']\n' + d.p +
    '\nEncoder  kiri: ' + d.el + ' (error ' + d.xl + ')   kanan: ' + d.er + ' (error ' + d.xr + ')' +
    '\nServo perintah: ' + d.sv + '\u00B0  |  Command diterima Uno: ' + d.rx +
    '  |  Frame rusak: ' + d.bf +
    (d.wd ? '\nWATCHDOG TER-LATCH (tekan EMERGENCY STOP untuk reset)' : ''));
}

// ---------- telemetri ----------
function onTel(d) {
  lastTel = Date.now(); unoOn = true;
  if (d.dg) { paintHeader(); onDiag(d); return; }
  const dir = v => v > 0 ? 'MAJU' : v < 0 ? 'MUNDUR' : 'STOP';
  $('v-mk').textContent = dir(d.mk); $('v-mr').textContent = dir(d.mr);
  $('v-ms').textContent = d.ms;
  $('v-up').textContent = (d.up / 10).toFixed(1) + 's';
  $('v-wd').textContent = d.wd ? 'AKTIF' : 'OK'; $('v-wd').className = 'v ' + (d.wd ? 'bad' : 'ok');
  $('v-bf').textContent = d.bf; $('v-bf').className = 'v ' + (d.bf > 0 ? 'bad' : 'ok');
  $('servo-actual').textContent = d.sv + '°';
  $('v-el').textContent = d.el; $('v-er').textContent = d.er;
  if (encPrev && d.up > encPrev.up) {
    const dt = (d.up - encPrev.up) / 10;
    $('v-elr').textContent = Math.round((d.el - encPrev.el) / dt) + ' count/s';
    $('v-err').textContent = Math.round((d.er - encPrev.er) / dt) + ' count/s';
  }
  encPrev = { el: d.el, er: d.er, up: d.up };
  if (d.wd && isActive()) { localHalt(); log('Watchdog Uno aktif - tekan EMERGENCY STOP untuk reset'); }
  paintHeader(d.wd);
  push(hist.ms, d.ms); push(hist.sv, d.sv); push(hist.ct, d.ct);
  drawChart();
  $('last-update').textContent = new Date().toLocaleTimeString();
}
function push(a, v) { a.push(v); if (a.length > MAX_PT) a.shift(); }

function drawChart() {
  const cv = $('chart'), ctx = cv.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  cv.width = cv.offsetWidth * dpr; cv.height = cv.offsetHeight * dpr;
  ctx.scale(dpr, dpr);
  const W = cv.offsetWidth, H = cv.offsetHeight;
  ctx.clearRect(0, 0, W, H);
  ctx.strokeStyle = '#2a2d3e'; ctx.lineWidth = 1;
  for (let i = 1; i < 4; i++) { const y = H * i / 4; ctx.beginPath(); ctx.moveTo(0, y); ctx.lineTo(W, y); ctx.stroke(); }
  const line = (arr, lo, hi, color) => {
    if (arr.length < 2) return;
    ctx.strokeStyle = color; ctx.lineWidth = 1.8; ctx.beginPath();
    arr.forEach((v, i) => {
      const x = W * i / (MAX_PT - 1), y = H - 4 - (H - 8) * (v - lo) / (hi - lo);
      i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
    });
    ctx.stroke();
  };
  line(hist.ct, 0, 1, '#fb923c');
  line(hist.ms, 0, 255, '#4f8ef7');
  line(hist.sv, 40, 115, '#38d9a9');
}

// ---------- input ----------
$('pwm-slider').oninput = function () {
  st.ms = parseInt(this.value); $('pwm-disp').textContent = this.value;
  if (isActive() && Date.now() - lastSend > 80) sendDrive();
};
$('pwm-slider').onchange = () => { if (isActive()) sendDrive(); };
$('servo-slider').oninput = function () { setServo(this.value, Date.now() - lastSend > 80); };
$('servo-slider').onchange = function () { setServo(this.value, true); };

// Heartbeat: jaga watchdog Uno selama motor/cutter aktif
// Heartbeat pendek (ping 14 byte) supaya byte masuk ke Uno sesedikit mungkin
// (SoftwareSerial RX bisa bikin servo tersentak); drive penuh tiap 1 s untuk koreksi.
setInterval(() => {
  if (!isActive() || !wsOpen) return;
  if (Date.now() - lastSend >= 1000) sendDrive(); else send({ cmd: 'ping' });
}, HB_MS);
// Telemetri basi -> Uno offline
setInterval(() => {
  if (lastTel && Date.now() - lastTel > TEL_STALE_MS && unoOn) { unoOn = false; paintHeader(); }
}, 500);
// Tab disembunyikan / HP dikunci -> berhenti
document.addEventListener('visibilitychange', () => { if (document.hidden && isActive()) eStop(); });
window.addEventListener('resize', drawChart);

needle(SERVO_C); drawChart(); connect();
</script>
</body>
</html>
)rawhtml";

// ============================================================
// HELPER
// ============================================================
static bool isValidFrame(const uint8_t* p, size_t n) {
  if (n < 2 || n > FWD_MAX_LEN) return false;
  if (p[0] != '{' || p[n - 1] != '}') return false;
  for (size_t i = 0; i < n; i++) {
    if (p[i] < 0x20 || p[i] > 0x7E) return false;
  }
  return true;
}

static void enqueueForUno(const String& s) {
  if (qCount == TXQ_LEN) {               // penuh: buang yang paling lama
    qHead = (qHead + 1) % TXQ_LEN;
    qCount--;
    statFwdDropped++;
  }
  txq[(qHead + qCount) % TXQ_LEN] = s;
  qCount++;
}

// Kirim 1 command per iterasi, hanya saat jalur dari Uno diam
static void pumpQueue() {
  if (qCount == 0) return;
  if (unoBuf.length() > 0) return;                         // Uno sedang kirim baris
  if (millis() - lastUnoByteMs < UART_IDLE_GAP_MS) return;
  unoSerial.print(txq[qHead]);
  unoSerial.print('\n');
  txq[qHead] = "";
  qHead = (qHead + 1) % TXQ_LEN;
  qCount--;
  statFwdOk++;
}

static String statusJson() {
  char b[48];
  snprintf(b, sizeof(b), "{\"t\":\"st\",\"uno\":%d,\"cl\":%d}",
           unoOnline ? 1 : 0, (int)ws.connectedClients());
  return String(b);
}

static void broadcastStatus() {
  String s = statusJson();
  ws.broadcastTXT(s);
}

// ============================================================
// WEBSOCKET EVENT
// ============================================================
void onWsEvent(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED: {
      IPAddress ip = ws.remoteIP(num);
      Serial.printf("[WS] client #%u konek dari %s\n", num, ip.toString().c_str());
      String s = statusJson();
      ws.sendTXT(num, s);
      break;
    }
    case WStype_DISCONNECTED:
      Serial.printf("[WS] client #%u putus\n", num);
      checkClients = true;               // dicek di loop (aman dari urutan callback)
      break;

    case WStype_TEXT: {
      if (!isValidFrame(payload, length)) {
        statFwdRejected++;
        ws.sendTXT(num, "{\"t\":\"err\",\"m\":\"frame ditolak (format/panjang)\"}");
        break;
      }
      String msg;
      msg.reserve(length);
      for (size_t i = 0; i < length; i++) msg += (char)payload[i];
      enqueueForUno(msg);
      break;
    }
    default:
      break;
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("\n[ELITA ESP32] v3 boot");

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);                  // respons WiFi lebih stabil
  // Opsional: kurangi daya TX bila curiga brownout (jarak pendek tetap cukup)
  // WiFi.setTxPower(WIFI_POWER_11dBm);
  bool apOk = WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  delay(100);
  if (!apOk) {
    Serial.println("[WiFi] softAP GAGAL -> restart 3 s");
    delay(3000);
    ESP.restart();
  }
  Serial.print("[WiFi] AP \""); Serial.print(AP_SSID);
  Serial.print("\" IP: "); Serial.println(WiFi.softAPIP());

  unoSerial.begin(UNO_BAUD, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  Serial.printf("[UART2] RX=GPIO%d TX=GPIO%d @ %d baud\n", UNO_RX_PIN, UNO_TX_PIN, UNO_BAUD);

  ws.begin();
  ws.onEvent(onWsEvent);
  // Ping tiap 2 s, putuskan client yang 2x tidak membalas pong
  // -> koneksi mati (HP tidur / pindah WiFi) tidak menumpuk & menghabiskan slot.
  ws.enableHeartbeat(2000, 2000, 2);

  server.on("/", []() { server.send_P(200, "text/html", DASHBOARD); });

  server.on("/cmd", HTTP_POST, []() {
    if (!server.hasArg("plain")) { server.send(400, "text/plain", "No body"); return; }
    String body = server.arg("plain");
    body.trim();
    if (!isValidFrame((const uint8_t*)body.c_str(), body.length())) {
      statFwdRejected++;
      server.send(400, "text/plain", "Frame ditolak");
      return;
    }
    enqueueForUno(body);
    server.send(200, "text/plain", "OK");
  });

  server.on("/status", HTTP_GET, []() {
    server.send(200, "application/json", statusJson());
  });

  server.onNotFound([]() { server.send(404, "text/plain", "Not found"); });

  server.begin();
  Serial.println("[HTTP] Server up di http://192.168.4.1");

  lastUnoLineMs = millis();
  lastStatsMs   = millis();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  server.handleClient();
  ws.loop();

  unsigned long now = millis();

  // --- 1. Telemetri dari Uno ---
  while (unoSerial.available()) {
    char c = (char)unoSerial.read();
    lastUnoByteMs = millis();
    if (c == '\n') {
      unoBuf.trim();
      bool ok = unoBuf.length() >= 2 && unoBuf[0] == '{' && unoBuf[unoBuf.length() - 1] == '}';
      for (size_t i = 0; ok && i < unoBuf.length(); i++) {
        if (unoBuf[i] < 0x20 || unoBuf[i] > 0x7E) ok = false;
      }
      if (ok) {
        statUnoLines++;
        lastUnoLineMs = millis();
        if (!unoOnline) {
          unoOnline = true;
          Serial.println("[ELITA] Uno ONLINE");
          broadcastStatus();
        }
        String out = "{\"t\":\"tel\"," + unoBuf.substring(1);
        ws.broadcastTXT(out);
      } else if (unoBuf.length() > 0) {
        statUnoBad++;
      }
      unoBuf = "";
    } else if (unoBuf.length() < UNO_LINE_MAX) {
      unoBuf += c;
    }
  }
  now = millis();
  if (unoBuf.length() > 0 && now - lastUnoByteMs > UNO_LINE_STALE) {
    statUnoBad++;
    unoBuf = "";
  }

  // --- 2. Deteksi Uno offline ---
  if (unoOnline && now - lastUnoLineMs > UNO_TIMEOUT_MS) {
    unoOnline = false;
    Serial.println("[ELITA] Uno OFFLINE (timeout)");
    broadcastStatus();
  }

  // --- 3. Client terakhir putus -> stop ---
  if (checkClients) {
    checkClients = false;
    if (ws.connectedClients() == 0) {
      Serial.println("[ELITA] tidak ada client -> STOP ke Uno");
      enqueueForUno("{\"cmd\":\"stop\"}");
      enqueueForUno("{\"cmd\":\"stop\"}");
    }
  }

  // --- 4. Kirim antrian command ke Uno ---
  pumpQueue();

  // --- 5. Keepalive ke semua client tiap 1 s (dashboard reconnect jika hilang) ---
  if (now - lastKeepMs >= 1000) {
    lastKeepMs = now;
    if (ws.connectedClients() > 0) broadcastStatus();
  }

  // --- 6. Statistik ---
  if (now - lastStatsMs >= STATS_MS) {
    lastStatsMs = now;
    Serial.printf("[STAT] clients=%d uno=%s telOK=%lu telBad=%lu fwdOK=%lu fwdRej=%lu fwdDrop=%lu heap=%lu\n",
                  (int)ws.connectedClients(), unoOnline ? "ON" : "OFF",
                  (unsigned long)statUnoLines, (unsigned long)statUnoBad,
                  (unsigned long)statFwdOk, (unsigned long)statFwdRejected,
                  (unsigned long)statFwdDropped, (unsigned long)ESP.getFreeHeap());
  }
}