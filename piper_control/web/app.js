import {RobotViewer} from './viewer.js';

const $ = id => document.getElementById(id);
const RAD = Math.PI / 180;
const fmt = (value, digits = 3) => Number.isFinite(value) ? Number(value).toFixed(digits) : '—';
let state = null;
let info = null;
let viewer = null;
let socket = null;
let connected = false;
let initialized = false;
let lastReceived = 0;
let sampleTimes = [];
let reconnectTimer = null;
let latestFault = '';

function showNotice(message = '') {
  $('notice').textContent = message;
  $('notice').hidden = !message;
}

function feedback(message, kind = '') {
  $('command-message').textContent = message;
  $('command-feedback').className = `command-feedback ${kind}`;
}

async function request(path, body) {
  const response = await fetch(path, {
    method: body === undefined ? 'GET' : 'POST',
    headers: body === undefined ? {} : {'Content-Type': 'application/json'},
    body: body === undefined ? undefined : JSON.stringify(body),
  });
  const data = await response.json().catch(() => ({}));
  if (!response.ok) {
    const detail = data.detail || data.error || response.statusText;
    throw new Error(typeof detail === 'string' ? detail : JSON.stringify(detail));
  }
  return data;
}

async function command(path, body, message) {
  feedback('正在发送指令…');
  try {
    const result = await request(path, body);
    feedback(message + (Number.isFinite(result.duration) ? ` · 规划 ${result.duration.toFixed(2)} s` : '')
      + (result.id ? ` · ${result.id.slice(0, 8)}` : ''), 'success');
  } catch (error) {
    feedback(error.message, 'error');
  }
}

function quaternionFromRPY([roll, pitch, yaw]) {
  const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
  const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
  const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
  return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
    cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy];
}

function rpyFromQuaternion([x, y, z, w]) {
  return [Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
    Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x)))),
    Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))];
}

function fields(container, prefix, names, step) {
  names.forEach((name, index) => {
    const label = document.createElement('label');
    label.textContent = name;
    const input = document.createElement('input');
    input.id = `${prefix}-${index}`;
    input.type = 'number';
    input.step = step;
    input.value = '0';
    input.required = true;
    label.append(input);
    $(container).append(label);
  });
}

function buildJointRows(names, limits) {
  $('joint-inputs').replaceChildren();
  $('telemetry-body').replaceChildren();
  names.forEach((name, i) => {
    const row = document.createElement('div');
    row.className = 'joint-row';
    row.innerHTML = `<label class="joint-name" for="joint-${i}"><i></i>J${i + 1}</label><span class="joint-current" id="current-${i}">—</span><input id="joint-${i}" type="number" step="0.01" value="0" required>`;
    const input = row.querySelector('input');
    input.setAttribute('aria-label', `${name} 目标角度，度`);
    if (Number.isFinite(limits[i]?.lower)) input.min = (Math.ceil(limits[i].lower / RAD * 100) / 100).toString();
    if (Number.isFinite(limits[i]?.upper)) input.max = (Math.floor(limits[i].upper / RAD * 100) / 100).toString();
    $('joint-inputs').append(row);
    const tableRow = document.createElement('tr');
    tableRow.innerHTML = `<td></td><td id="q-${i}">—</td><td id="deg-${i}">—</td><td id="dq-${i}">—</td><td id="tau-${i}">—</td><td><div class="travel"><div class="travel-fill" id="fill-${i}"></div><div class="travel-marker" id="marker-${i}"></div></div></td>`;
    tableRow.firstChild.textContent = name;
    $('telemetry-body').append(tableRow);
  });
}

function setConnected(value, text) {
  connected = value;
  $('connection-dot').className = value ? 'online' : 'error';
  $('connection-text').textContent = text || (value ? '实时连接' : '连接中断');
  document.body.classList.toggle('disconnected', !value);
  $('enable').disabled = !value || Boolean(state?.enabled);
  $('disable').disabled = !value || !state?.enabled;
  ['execute-joints', 'execute-tcp', 'execute-arc'].forEach(id => { $(id).disabled = !value || !state?.enabled; });
  ['copy-joints', 'copy-tcp', 'copy-arc'].forEach(id => { $(id).disabled = !value; });
}

function copyJoints() {
  state?.positions?.forEach((q, i) => {
    const input = $(`joint-${i}`);
    if (input) input.value = fmt(Math.max(input.min ? Number(input.min) : -Infinity,
      Math.min(input.max ? Number(input.max) : Infinity, q / RAD)), 2);
  });
}

function copyTCP() {
  if (!state?.tcp_pose) return;
  state.tcp_pose.position.forEach((value, i) => { $(`tcp-p-${i}`).value = fmt(value, 5); });
  rpyFromQuaternion(state.tcp_pose.orientation).forEach((value, i) => { $(`tcp-r-${i}`).value = fmt(value / RAD, 3); });
}

function copyArc() {
  if (!state?.tcp_pose) return;
  state.tcp_pose.position.forEach((value, i) => {
    $(`arc-via-${i}`).value = fmt(value, 5);
    $(`arc-target-${i}`).value = fmt(value, 5);
  });
  rpyFromQuaternion(state.tcp_pose.orientation).forEach((value, i) => {
    $(`arc-via-r-${i}`).value = fmt(value / RAD, 3);
    $(`arc-target-r-${i}`).value = fmt(value / RAD, 3);
  });
}

function updateState(next) {
  state = next;
  lastReceived = performance.now();
  sampleTimes.push(lastReceived);
  sampleTimes = sampleTimes.filter(t => t > lastReceived - 2000);
  setConnected(next.connected !== false, next.connected === false ? '机器人离线' : undefined);
  const modes = {idle:'等待指令', disabled:'已失能', fault:'故障', movej:'关节运动', trajectory:'轨迹执行', movel:'直线运动', movec:'圆弧运动', servo_joints:'关节实时控制', servo_pose:'TCP 实时控制', servo_twist:'TCP 速度控制'};
  $('robot-mode').textContent = modes[next.mode] || next.mode || (next.enabled ? '已使能' : '待机');
  $('power-state').textContent = next.enabled ? '电机已使能' : '电机未使能';
  $('viewer-state').textContent = `${next.backend === 'hardware' ? 'HARDWARE' : next.backend === 'sim' ? 'SIMULATION' : (next.backend || '').toUpperCase()} · ${next.enabled ? 'ENABLED' : 'STANDBY'}`;
  const motion = next.motion || {};
  $('motion-kind').textContent = motion.kind || '空闲';
  const motionStatus = {running: '执行中', succeeded: '已完成', completed: '已完成', canceled: '已停止', cancelled: '已停止', failed: '执行失败', aborted: '执行中止', idle: '等待指令'};
  const progress = Number.isFinite(motion.progress) && motion.status === 'running' ? ` · ${(motion.progress * 100).toFixed(0)}%` : '';
  $('motion-state').textContent = (motionStatus[motion.status] || motion.status || '等待指令') + progress;
  if (motion.error && motion.error !== latestFault) feedback(motion.error, 'error');
  latestFault = motion.error || '';
  const errors = Array.isArray(next.errors) ? next.errors : next.errors ? [next.errors] : [];
  const notices = errors.map(e => typeof e === 'string' ? e : JSON.stringify(e));
  if (next.backend === 'hardware' && next.command_ready === false && next.extra?.firmware?.teach_status === 1) {
    notices.unshift('示教录制尚未结束：按钮绿灯常亮表示正在录制，松开按钮不会结束录制。请短按一次示教按钮，确认绿灯熄灭；不要双击，双击会回放轨迹。');
  }
  showNotice(notices.join(' · '));

  const limits = info?.joint_limits || [];
  (next.positions || []).forEach((q, i) => {
    if (!$(`q-${i}`)) return;
    $(`q-${i}`).textContent = fmt(q, 4);
    $(`deg-${i}`).textContent = fmt(q / RAD, 2);
    $(`current-${i}`).textContent = fmt(q / RAD, 2);
    $(`dq-${i}`).textContent = fmt(next.velocities?.[i], 4);
    $(`tau-${i}`).textContent = fmt(next.efforts?.[i], 3);
    const lower = limits[i]?.lower ?? -Math.PI;
    const upper = limits[i]?.upper ?? Math.PI;
    const percent = Math.max(0, Math.min(100, (q - lower) / (upper - lower) * 100));
    $(`fill-${i}`).style.width = `${percent}%`;
    $(`marker-${i}`).style.left = `${percent}%`;
  });
  if (next.tcp_pose) {
    ['x', 'y', 'z'].forEach((axis, i) => { $(`tcp-${axis}`).textContent = fmt(next.tcp_pose.position[i], 4); });
    $('tcp-rpy').textContent = rpyFromQuaternion(next.tcp_pose.orientation).map(v => fmt(v / RAD, 1)).join(' / ');
  }
  if (!initialized && next.positions?.length) {
    copyJoints(); copyTCP(); copyArc(); initialized = true;
  }
  viewer?.update(next);
  const time = typeof next.timestamp === 'number' ? new Date(next.timestamp * 1000) : new Date(next.timestamp || Date.now());
  $('telemetry-stamp').textContent = `更新于 ${time.toLocaleTimeString('zh-CN', {hour12: false})}`;
}

function connect() {
  clearTimeout(reconnectTimer);
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  socket = new WebSocket(`${protocol}//${location.host}/ws/state`);
  socket.onmessage = event => {
    try { updateState(JSON.parse(event.data)); }
    catch (error) { showNotice(`状态数据错误：${error.message}`); }
  };
  socket.onclose = () => {
    setConnected(false);
    $('viewer-state').textContent = 'DISCONNECTED';
    $('robot-mode').textContent = '连接中断';
    reconnectTimer = setTimeout(connect, 1500);
  };
  socket.onerror = () => socket.close();
}

function readVector(prefix) {
  return [0, 1, 2].map(i => Number($(prefix + '-' + i).value));
}

fields('tcp-position-inputs', 'tcp-p', ['X', 'Y', 'Z'], 'any');
fields('tcp-orientation-inputs', 'tcp-r', ['Roll', 'Pitch', 'Yaw'], 'any');
fields('arc-via-inputs', 'arc-via', ['X', 'Y', 'Z'], 'any');
fields('arc-target-inputs', 'arc-target', ['X', 'Y', 'Z'], 'any');
fields('arc-via-orientation-inputs', 'arc-via-r', ['Roll', 'Pitch', 'Yaw'], 'any');
fields('arc-target-orientation-inputs', 'arc-target-r', ['Roll', 'Pitch', 'Yaw'], 'any');
$('arc-rotate').onchange = () => { $('arc-orientation-fields').hidden = !$('arc-rotate').checked; };
buildJointRows(['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], []);
setConnected(false, '正在连接');

$('enable').onclick = () => command('/api/enable', {}, '机械臂已使能');
$('disable').onclick = () => command('/api/disable', {}, '机械臂已失能');
$('stop').onclick = () => command('/api/stop', {}, '停止指令已发送');
$('copy-joints').onclick = copyJoints;
$('copy-tcp').onclick = copyTCP;
$('copy-arc').onclick = copyArc;

document.querySelectorAll('[data-tab]').forEach(button => {
  button.onclick = () => {
    document.querySelectorAll('[data-tab]').forEach(tab => {
      tab.classList.toggle('active', tab === button);
      tab.setAttribute('aria-selected', String(tab === button));
    });
    ['joints', 'tcp', 'arc'].forEach(name => { $(name === 'joints' ? 'joint-form' : `${name}-form`).hidden = name !== button.dataset.tab; });
  };
});

$('joint-form').onsubmit = event => {
  event.preventDefault();
  if (!connected || !state?.enabled) return;
  const positions = (info?.joint_names || state.joint_names).map((_, i) => Number($(`joint-${i}`).value) * RAD);
  command('/api/movej', {positions, duration: Number($('joint-duration').value)}, '关节轨迹已提交');
};

$('tcp-form').onsubmit = event => {
  event.preventDefault();
  if (!connected || !state?.enabled) return;
  const pose = {position: readVector('tcp-p'), orientation: quaternionFromRPY(readVector('tcp-r').map(v => v * RAD))};
  command('/api/movel', {pose, speed: Number($('tcp-speed').value),
    angular_speed: Number($('tcp-angular-speed').value) * RAD}, '直线轨迹已提交');
};

$('arc-form').onsubmit = event => {
  event.preventDefault();
  if (!connected || !state?.enabled || !state?.tcp_pose) return;
  const orientation = [...state.tcp_pose.orientation];
  const viaOrientation = $('arc-rotate').checked ? quaternionFromRPY(readVector('arc-via-r').map(v => v * RAD)) : orientation;
  const targetOrientation = $('arc-rotate').checked ? quaternionFromRPY(readVector('arc-target-r').map(v => v * RAD)) : orientation;
  command('/api/movec', {
    via: {position: readVector('arc-via'), orientation: viaOrientation},
    pose: {position: readVector('arc-target'), orientation: targetOrientation},
    speed: Number($('arc-speed').value),
    angular_speed: Number($('arc-angular-speed').value) * RAD,
  }, '圆弧轨迹已提交');
};

document.querySelectorAll('[data-view]').forEach(button => {
  button.onclick = () => {
    viewer?.setView(button.dataset.view);
    document.querySelectorAll('[data-view]').forEach(item => item.classList.toggle('active', item === button));
  };
});
$('reset-view').onclick = () => document.querySelector('[data-view="iso"]').click();
$('toggle-trail').onclick = () => {
  const visible = $('toggle-trail').getAttribute('aria-pressed') !== 'true';
  $('toggle-trail').setAttribute('aria-pressed', String(visible));
  if (viewer) viewer.trail.visible = visible;
};

setInterval(() => {
  const elapsed = lastReceived ? (performance.now() - lastReceived) / 1000 : Infinity;
  const age = elapsed + (state?.feedback_age || 0);
  $('feedback-age').textContent = Number.isFinite(age) ? `反馈延迟 ${Math.round(age * 1000)} ms` : '等待反馈';
  if (elapsed > 2 && connected) {
    setConnected(false, '反馈已过期');
    $('viewer-state').textContent = 'STALE';
    $('robot-mode').textContent = '反馈已过期';
  }
  $('state-rate').textContent = sampleTimes.length > 1 && elapsed < 2 ? Math.round((sampleTimes.length - 1) * 1000 / (sampleTimes.at(-1) - sampleTimes[0])) : '—';
}, 250);

async function init() {
  try {
    info = await request('/api/info');
    $('robot-name').textContent = info.name || 'Robot';
    $('robot-description').textContent = `${info.joint_names.length} 自由度 · ${info.control_rate_hz || '—'} Hz 控制`;
    $('backend').textContent = info.backend === 'hardware' ? '真机 HARDWARE' : ['sim', 'simulation'].includes(info.backend) ? '仿真 SIMULATION' : info.backend || 'ROBOT';
    $('backend').classList.toggle('hardware', info.backend === 'hardware');
    $('base-frame').textContent = info.base_frame || 'base_link';
    $('tcp-frame').textContent = info.tcp_frame || 'tcp';
    buildJointRows(info.joint_names, info.joint_limits || []);
    if (state) { copyJoints(); copyTCP(); copyArc(); }
  } catch (error) {
    showNotice(`无法读取机器人信息：${error.message}`);
  }
  try {
    viewer = new RobotViewer($('robot-canvas'), $('viewport'), showNotice);
    viewer.angleOffsets = info?.angle_offsets || [];
    await viewer.load(info?.model_url || '/robot/model.urdf');
    $('viewer-loading').hidden = true;
  } catch (error) {
    $('viewer-loading').textContent = `三维模型不可用：${error.message}`;
  }
}

// Telemetry starts independently: a model resource failure must not block controls.
connect();
init();
