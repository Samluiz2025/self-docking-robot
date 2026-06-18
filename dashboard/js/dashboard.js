// ── ROS connection ──────────────────────────────────────────────────────────
const ROBOT_IP   = window.location.hostname || 'localhost';
const WS_URL     = `ws://${ROBOT_IP}:9090`;

let ros, connected = false;

function connect() {
  ros = new ROSLIB.Ros({ url: WS_URL });
  ros.on('connection', () => {
    connected = true;
    document.getElementById('conn-badge').className = 'badge connected';
    document.getElementById('conn-badge').textContent = 'Connected';
    subscribeAll();
    log('Connected to robot at ' + ROBOT_IP);
  });
  ros.on('error', () => {
    connected = false;
    document.getElementById('conn-badge').className = 'badge disconnected';
    document.getElementById('conn-badge').textContent = 'Disconnected';
    setTimeout(connect, 5000);
  });
  ros.on('close', () => { connected = false; setTimeout(connect, 5000); });
}

// ── Subscriptions ───────────────────────────────────────────────────────────
function subscribeAll() {
  sub('/robot_state',        'std_msgs/String',               m => updateState(m.data));
  sub('/battery_state',      'sensor_msgs/BatteryState',      m => updateBattery(m));
  sub('/ultrasonic/front',   'sensor_msgs/Range',             m => updateUss(m.range));
  sub('/ir_dock_detected',   'std_msgs/Bool',                 m => updateIR(m.data));
  sub('/cmd_vel',            'geometry_msgs/Twist',           m => updateSpeed(m.linear.x));
  sub('/map',                'nav_msgs/OccupancyGrid',        m => drawMap(m));
  sub('/camera/image_raw/compressed', 'sensor_msgs/CompressedImage', m => updateCamera(m));
}

function sub(topic, type, cb) {
  const t = new ROSLIB.Topic({ ros, name: topic, messageType: type });
  t.subscribe(cb);
}

// ── State display ───────────────────────────────────────────────────────────
const STATE_COLORS = {
  IDLE: '#3d7eff', LOW_BATTERY: '#f59e0b', SEEKING_DOCK: '#a855f7',
  APPROACHING_DOCK: '#f97316', DOCKED: '#22c55e', UNDOCKING: '#64748b'
};

function updateState(state) {
  const el = document.getElementById('robot-state');
  el.textContent = state;
  el.style.background = (STATE_COLORS[state] || '#1e3a5f') + '33';
  el.style.color = STATE_COLORS[state] || '#3d7eff';
  log(`State: ${state}`, 'state');
}

// ── Battery chart ───────────────────────────────────────────────────────────
const battChart = new Chart(document.getElementById('battery-chart'), {
  type: 'line',
  data: { labels: [], datasets: [{
    label: 'Voltage (V)', data: [], borderColor: '#3d7eff',
    backgroundColor: '#3d7eff22', fill: true, tension: 0.3, pointRadius: 0,
  }]},
  options: { responsive: true, maintainAspectRatio: false, animation: false,
             plugins: { legend: { display: false } },
             scales: { y: { min: 9, max: 13, grid: { color: '#2a2d3a' } },
                       x: { display: false } } }
});

function updateBattery(m) {
  const now = new Date().toLocaleTimeString();
  battChart.data.labels.push(now);
  battChart.data.datasets[0].data.push(m.voltage);
  if (battChart.data.labels.length > 60) {
    battChart.data.labels.shift(); battChart.data.datasets[0].data.shift();
  }
  battChart.update('none');
  document.getElementById('batt-v').textContent   = m.voltage.toFixed(2);
  document.getElementById('batt-a').textContent   = m.current.toFixed(2);
  document.getElementById('batt-pct').textContent = (m.percentage * 100).toFixed(0);
}

function updateUss(range) {
  document.getElementById('sensor-uss').textContent = range.toFixed(2) + ' m';
  document.getElementById('sensor-uss').style.color = range < 0.3 ? '#ef4444' : '#e2e8f0';
}

function updateIR(detected) {
  const el = document.getElementById('sensor-ir');
  el.textContent = detected ? 'DETECTED' : 'None';
  el.style.color = detected ? '#22c55e' : '#64748b';
}

function updateSpeed(v) {
  document.getElementById('sensor-spd').textContent = v.toFixed(2) + ' m/s';
}

// ── Map renderer ────────────────────────────────────────────────────────────
let mapMeta = null;
const canvas = document.getElementById('map-canvas');
const ctx    = canvas.getContext('2d');

function drawMap(msg) {
  mapMeta = msg.info;
  const { width, height, resolution } = msg.info;
  canvas.width = width; canvas.height = height;
  const img = ctx.createImageData(width, height);
  for (let i = 0; i < msg.data.length; i++) {
    const v = msg.data[i];
    const c = v === -1 ? 128 : v === 0 ? 240 : 30;
    img.data[i*4]   = c; img.data[i*4+1] = c;
    img.data[i*4+2] = c; img.data[i*4+3] = 255;
  }
  ctx.putImageData(img, 0, 0);
}

canvas.addEventListener('click', e => {
  if (!mapMeta || !connected) return;
  const rect = canvas.getBoundingClientRect();
  const sx = (e.clientX - rect.left) / rect.width  * canvas.width;
  const sy = (e.clientY - rect.top)  / rect.height * canvas.height;
  const wx = mapMeta.origin.position.x + sx * mapMeta.resolution;
  const wy = mapMeta.origin.position.y + (canvas.height - sy) * mapMeta.resolution;
  sendNavGoal(wx, wy);
});

function sendNavGoal(x, y) {
  const goal = new ROSLIB.ActionClient({
    ros, serverName: '/navigate_to_pose',
    actionName: 'nav2_msgs/action/NavigateToPose'
  });
  log(`Nav goal → (${x.toFixed(2)}, ${y.toFixed(2)})`);
}

// ── Camera ──────────────────────────────────────────────────────────────────
function updateCamera(m) {
  document.getElementById('camera-img').src = 'data:image/jpeg;base64,' + m.data;
}

// ── Controls ────────────────────────────────────────────────────────────────
function triggerDock() {
  if (!connected) return;
  const t = new ROSLIB.Topic({ ros, name: '/trigger_dock', messageType: 'std_msgs/Bool' });
  t.publish(new ROSLIB.Message({ data: true }));
  log('Return to dock triggered');
}

function emergencyStop() {
  if (!connected) return;
  const t = new ROSLIB.Topic({ ros, name: '/cmd_vel', messageType: 'geometry_msgs/Twist' });
  t.publish(new ROSLIB.Message({ linear: {x:0,y:0,z:0}, angular: {x:0,y:0,z:0} }));
  log('EMERGENCY STOP', 'state');
}

// ── Mission log ─────────────────────────────────────────────────────────────
const seenStates = new Set();
function log(msg, cls = '') {
  if (cls === 'state' && seenStates.has(msg)) return;
  if (cls === 'state') seenStates.add(msg);
  const div  = document.getElementById('mission-log');
  const time = new Date().toLocaleTimeString();
  div.innerHTML += `<div class="entry ${cls}">[${time}] ${msg}</div>`;
  div.scrollTop = div.scrollHeight;
}

function clearLog() {
  document.getElementById('mission-log').innerHTML = '';
  seenStates.clear();
}

// ── Start ───────────────────────────────────────────────────────────────────
connect();
