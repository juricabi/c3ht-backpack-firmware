'use strict';
// C3HT EU web UI. Session cookie (HttpOnly) + CSRF token (kept in memory only).
// No inline scripts (CSP: default-src 'self').

const $ = (id) => document.getElementById(id);
let csrf = null;
let info = null;
let websock = null;
let Euler = { heading: 0, pitch: 0, roll: 0 };

function msg(id, text, cls) {
  const e = $(id);
  e.textContent = text || '';
  e.className = 'msg' + (cls ? ' ' + cls : '');
}

async function api(path, opts = {}) {
  const o = { method: opts.method || 'GET', headers: {}, credentials: 'same-origin', cache: 'no-store' };
  if (opts.json !== undefined) {
    o.headers['Content-Type'] = 'application/json';
    o.body = JSON.stringify(opts.json);
  }
  if (o.method !== 'GET' && csrf) o.headers['X-C3HT-CSRF'] = csrf;
  const r = await fetch(path, o);
  let j = {};
  try { j = await r.json(); } catch (e) { /* empty body */ }
  return { status: r.status, j };
}

function showLogin(text) {
  $('app').classList.add('hidden');
  $('login').classList.remove('hidden');
  msg('login-msg', text || '');
  $('login-pw').focus();
}

async function enterApp(j) {
  csrf = j.csrf;
  $('login').classList.add('hidden');
  $('app').classList.remove('hidden');
  $('banner-default').classList.toggle('hidden', !j.default_pw);
  await refreshInfo();
}

async function login(pw) {
  const { status, j } = await api('/api/login', { method: 'POST', json: { password: pw } });
  if (status === 200) return enterApp(j);
  if (status === 429) return showLogin('Too many wrong attempts. Wait a few minutes and try again.');
  showLogin('Wrong password.');
}

function row(t, k, v) {
  const tr = t.insertRow();
  tr.insertCell().textContent = k;
  tr.insertCell().textContent = v;
}

async function refreshInfo() {
  const { status, j } = await api('/api/info');
  if (status === 401) return showLogin('Session ended, please log in again.');
  if (status !== 200) return;
  info = j;
  const t = $('info');
  t.textContent = '';
  row(t, 'Device', (j.product || j.target) + (j.dev_unit ? '  (DEV unit)' : ''));
  row(t, 'Firmware', j.version + '  (security version ' + j.security_version + ')');
  row(t, 'Wi-Fi network', j.ssid);
  row(t, 'Card ID', j.card_id);
  row(t, 'Password', j.default_pw ? 'card password' : 'your own password');
  row(t, 'Credential store', j.credential);
  row(t, 'Signed updates', j.ota ? 'enabled (keys ' + j.keys + ')' : 'disabled in this build (USB only)');
  row(t, 'Lowest accepted security version', j.min_security_version);
  row(t, 'IMU', j.imu ? 'detected' : 'not detected');
  row(t, 'Radio power setting', j.tx_power_qdbm >= 0 ? (j.tx_power_qdbm / 4) + ' dBm' : '-');
  row(t, 'Uptime', j.uptime_s + ' s (boot #' + j.boot + ')');
  $('ap-left').textContent = j.ap_remaining_s;
  $('upd-target').textContent = j.target;
  $('banner-default').classList.toggle('hidden', !j.default_pw);
  const ev = $('events');
  ev.textContent = '';
  (j.events || []).forEach((e) => row(ev, 'boot ' + e.boot + ' +' + e.up + 's', e.t + (e.n > 1 ? ' x' + e.n : '') + (e.d ? ' (' + e.d + ')' : '')));
}

// ---------------------------------------------------------------- tabs
document.querySelectorAll('nav button').forEach((b) => b.addEventListener('click', () => {
  document.querySelectorAll('nav button').forEach((x) => x.classList.toggle('active', x === b));
  document.querySelectorAll('.tab').forEach((t) => t.classList.toggle('hidden', t.id !== b.dataset.tab));
  if (b.dataset.tab === 'tab-ht') startHeadTracking();
  if (b.dataset.tab !== 'tab-ht') refreshInfo();
}));

$('login-form').addEventListener('submit', (e) => { e.preventDefault(); login($('login-pw').value); $('login-pw').value = ''; });
$('btn-logout').addEventListener('click', async () => { await api('/api/logout', { method: 'POST', json: {} }); csrf = null; showLogin('Logged out.'); });
$('btn-stop').addEventListener('click', async () => { await api('/api/reboot', { method: 'POST', json: {} }); showLogin('Wi-Fi is switching off.'); });

// ---------------------------------------------------------------- password
$('pw-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  const nw = $('pw-new').value;
  if (nw.length < 16) return msg('pw-msg', 'At least 16 characters, or use Generate.', 'bad');
  const { status, j } = await api('/api/password', { method: 'POST', json: { current: $('pw-cur').value, new: nw } });
  $('pw-cur').value = ''; $('pw-new').value = '';
  if (status === 200) { showLogin(j.msg); } else msg('pw-msg', j.msg || 'Failed', 'bad');
});
$('pw-gen').addEventListener('click', async () => {
  const cur = $('pw-cur').value;
  if (!cur) return msg('pw-msg', 'Enter the current password first.', 'bad');
  const { status, j } = await api('/api/password', { method: 'POST', json: { current: cur, generate: true } });
  $('pw-cur').value = '';
  if (status === 200) {
    showLogin('New password (write it down now, it is shown only once): ' + j.password + '  ' + j.msg);
  } else msg('pw-msg', j.msg || 'Failed', 'bad');
});

$('fr-form').addEventListener('submit', async (e) => {
  e.preventDefault();
  if (!confirm('Erase password, binding, calibration and log?')) return;
  const { status, j } = await api('/api/factory-reset', { method: 'POST', json: { current: $('fr-cur').value } });
  $('fr-cur').value = '';
  if (status === 200) showLogin('Factory reset done. The device restarts.'); else msg('fr-msg', j.msg || 'Failed', 'bad');
});

// ---------------------------------------------------------------- update
$('btn-upload').addEventListener('click', () => {
  const f = $('fw-file').files[0];
  if (!f) return msg('upd-msg', 'Choose a .bin file first.', 'bad');
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/update' + ($('fw-xtarget').checked ? '?xtarget=1' : ''));
  xhr.setRequestHeader('Content-Type', 'application/octet-stream');
  xhr.setRequestHeader('X-C3HT-CSRF', csrf);
  xhr.upload.onprogress = (ev) => { if (ev.lengthComputable) $('fw-progress').value = Math.round(ev.loaded * 100 / ev.total); };
  xhr.onload = () => {
    let j = {};
    try { j = JSON.parse(xhr.responseText); } catch (e) { /* ignore */ }
    if (xhr.status === 202) { msg('upd-msg', 'Uploaded. Checking signature...'); pollUpdate(); } else msg('upd-msg', j.msg || ('Upload failed (' + xhr.status + ')'), 'bad');
  };
  xhr.onerror = () => msg('upd-msg', 'Connection lost during upload.', 'bad');
  msg('upd-msg', 'Uploading...');
  xhr.send(f);
});

async function pollUpdate() {
  const { status, j } = await api('/api/update/status');
  if (status !== 200) return msg('upd-msg', 'Lost connection; the device may be restarting.', 'bad');
  if (j.state === 'verifying' || j.state === 'receiving') return setTimeout(pollUpdate, 500);
  if (j.state === 'done') return msg('upd-msg', j.msg + '. Wait until the LED is steady before removing power.', 'ok');
  msg('upd-msg', 'Rejected: ' + j.msg, 'bad');
}

// ---------------------------------------------------------------- head tracking
function loadScript(src) {
  return new Promise((resolve, reject) => {
    const s = document.createElement('script');
    s.src = src; s.onload = resolve; s.onerror = reject;
    document.body.appendChild(s);
  });
}

let htStarted = false;
function startHeadTracking() {
  if (htStarted) return;
  htStarted = true;
  loadScript('p5.js').then(() => {
    websock = new WebSocket('ws://' + window.location.host + '/ws');
    websock.onopen = () => websock.send('auth:' + csrf);
    websock.onclose = () => { msg('ht-msg', 'Head-tracking connection closed.', 'bad'); htStarted = false; };
    websock.onmessage = (evt) => {
      const d = JSON.parse(evt.data);
      if (d.done) msg('ht-msg', 'Calibration finished.', 'ok');
      if (d.orientation) {
        $('x-angle').value = d.pitch; $('y-angle').value = d.roll; $('z-angle').value = d.heading;
        $('label-x').textContent = Math.round(d.pitch); $('label-y').textContent = Math.round(d.roll); $('label-z').textContent = Math.round(d.heading);
      }
      if (d.heading !== undefined && !d.orientation) {
        Euler = d;
        $('angle-x').textContent = Euler.roll.toFixed(1);
        $('angle-y').textContent = Euler.pitch.toFixed(1);
        $('angle-z').textContent = Euler.heading.toFixed(1);
      }
    };
  }).catch(() => msg('ht-msg', 'Could not load the 3D view.', 'bad'));
}

const wsSend = (s) => { if (websock && websock.readyState === 1) websock.send(s); };
function setOrientation() {
  $('label-x').textContent = $('x-angle').value; $('label-y').textContent = $('y-angle').value; $('label-z').textContent = $('z-angle').value;
  wsSend('o:' + $('x-angle').value + ':' + $('y-angle').value + ':' + $('z-angle').value);
}
['x-angle', 'y-angle', 'z-angle'].forEach((id) => $(id).addEventListener('input', setOrientation));
$('set-center').addEventListener('click', () => wsSend('sc'));
$('cal-gyro').addEventListener('click', () => { msg('ht-msg', 'Keep the board flat and still...'); wsSend('ci'); });
$('reset-board').addEventListener('click', () => { ['x-angle', 'y-angle', 'z-angle'].forEach((id) => { $(id).value = 0; }); setOrientation(); wsSend('ro'); });
$('save-orientation').addEventListener('click', () => { wsSend('sv'); msg('ht-msg', 'Orientation saved.', 'ok'); });

// p5.js global-mode callbacks
let plane, tex;
window.preload = function () { plane = loadModel('airplane.obj', true); tex = loadImage('texture.gif'); };
window.setup = function () { const c = createCanvas(360, 360, WEBGL); c.parent('canvas-holder'); };
window.draw = function () {
  background(192);
  rotateY(radians(Euler.heading + 180));
  rotateZ(radians(Euler.pitch));
  rotateX(radians(-Euler.roll));
  push(); noStroke(); scale(1.5); translate(0, -26, 0); texture(tex); model(plane); pop();
};

// ---------------------------------------------------------------- start
(async function init() {
  // Second QR code on the card: http://10.0.0.1/#k=<password>. Use it once, then
  // remove it from the address bar and history.
  const m = location.hash.match(/^#k=([0-9A-Za-z-]{8,64})$/);
  if (m) {
    history.replaceState(null, '', '/');
    return login(m[1]);
  }
  const { status, j } = await api('/api/session');
  if (status === 200) return enterApp(j);
  showLogin('');
})();
setInterval(() => { if (csrf && !$('tab-status').classList.contains('hidden')) refreshInfo(); }, 15000);
