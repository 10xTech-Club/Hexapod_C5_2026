// ─── State ───────────────────────────────────────────────────────
var ws;
var servoData = {};
var sweeping  = false;
var walking   = false;
var currentSpeed = 500;

// Hex SVG leg walk animation interval
var walkAnimInterval = null;
var walkAnimStep = 0;

// Groups A (L1,L3,R5) and B (L2,R4,R6) for alternating gait viz
var legGroupA = ['leg1','leg3','leg5'];
var legGroupB = ['leg2','leg4','leg6'];
var footGroupA = [0, 2, 4]; // foot circle indices
var footGroupB = [1, 3, 5];

var legLabels = {
    1: { code: 'FL', name: 'Front-left' },
    2: { code: 'ML', name: 'Mid-left' },
    3: { code: 'RL', name: 'Rear-left' },
    4: { code: 'FR', name: 'Front-right' },
    5: { code: 'MR', name: 'Mid-right' },
    6: { code: 'RR', name: 'Rear-right' }
};

var cmdLabels = {
    startWalk:         { label: 'WALK FWD',   color: '#3b82f6' },
    startWalkBackward: { label: 'WALK BCK',   color: '#3b82f6' },
    startTurnLeft:     { label: 'TURN LEFT',  color: '#06b6d4' },
    startTurnRight:    { label: 'TURN RIGHT', color: '#06b6d4' },
    stopWalk:          { label: 'STOP',        color: '#ef4444' },
    waveHand:          { label: 'WAVE HAND',  color: '#06b6d4' },
    danceMove:         { label: 'DANCE',       color: '#a78bfa' },
    waveAll_Left:      { label: 'WAVE ALL L', color: '#06b6d4' },
    waveAll_Right:     { label: 'WAVE ALL R', color: '#06b6d4' }
};

// ─── WebSocket ───────────────────────────────────────────────────
function connectWebSocket() {
    var protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    ws = new WebSocket(protocol + '//' + window.location.hostname + ':81');

    ws.onopen = function () {
        var el = document.getElementById('status');
        el.className = 'status-bar online';
        el.textContent = 'ONLINE';
        document.getElementById('pingDot').classList.add('live');
        logActivity('CONNECTED', '#10b981');
    };

    ws.onclose = function () {
        var el = document.getElementById('status');
        el.className = 'status-bar offline';
        el.textContent = 'OFFLINE';
        document.getElementById('pingDot').classList.remove('live');
        stopWalkAnim();
        logActivity('DISCONNECTED', '#ef4444');
        setTimeout(connectWebSocket, 3000);
    };

    ws.onerror = function () {};

    ws.onmessage = function (event) {
        try {
            var data = JSON.parse(event.data);
            switch (data.type) {
                case 'servoUpdate':
                    updateServoDisplay(data.servo, data.angle, data.pwm);
                    break;
                case 'walkStatus':
                    updateWalkStatus(data.walking, data.step, data.direction);
                    break;
                case 'speedUpdate':
                    updateSpeedFromServer(data.speed);
                    break;
                case 'strideUpdate':
                    updateStrideFromServer(data.stride);
                    break;
                case 'sensorData':
                    updateSensors(data);
                    break;
                case 'satStatus':
                    updateSatStatus(data);
                    break;
                case 'chatReceived':
                    satAddMessage(data.from, data.msg);
                    break;
                case 'log':
                    logActivity(data.msg, '#a78bfa');
                    break;
            }
        } catch (e) {}
    };
}

// ─── Satellite Communication ──────────────────────────────────────
var satConnected  = false;
var satRecipient  = 'satellite';   // default: send only to satellite

function satConnect() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'connectSatellite' }));
        document.getElementById('satStatusText').textContent = 'Activating ESP-NOW...';
        logActivity('SAT ESP-NOW ACTIVATE', '#f59e0b');
    }
}

function satDisconnect() {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'disconnectSatellite' }));
    }
}

function setSatRecipient(target) {
    satRecipient = target;
    document.querySelectorAll('.sat-rcpt-btn').forEach(function(btn) {
        btn.classList.remove('active');
    });
    var activeBtn = document.getElementById('rcpt-' + target);
    if (activeBtn) activeBtn.classList.add('active');
}

function satSendChat() {
    var input = document.getElementById('satChatInput');
    var msg = input.value.trim();
    if (!msg || !satConnected) return;
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'sendChat', msg: msg, to: satRecipient }));
        var toLabel = satRecipient === 'all' ? 'BOTH VIA SAT' : (satRecipient.toUpperCase() + ' VIA SAT');
        satAddMessage('me', msg, toLabel);
        input.value = '';
    }
}

function satAddMessage(from, msg, toLabel) {
    var log = document.getElementById('satChatLog');
    if (!log) return;
    var div = document.createElement('div');
    var isOut = (from === 'me');
    div.className = 'sat-msg ' + (isOut ? 'sat-msg-out' : 'sat-msg-in');
    var now = new Date();
    var time = now.getHours().toString().padStart(2,'0') + ':' + now.getMinutes().toString().padStart(2,'0');
    var fromLabel = isOut ? ('HEX' + (toLabel ? ' → ' + toLabel : '')) : from.toUpperCase();
    div.innerHTML = '<span class="sat-msg-from">' + fromLabel + '</span>' +
                    '<span class="sat-msg-text">' + msg + '</span>' +
                    '<span class="sat-msg-time">' + time + '</span>';
    log.appendChild(div);
    log.scrollTop = log.scrollHeight;
    // Cap at 60 messages
    while (log.children.length > 60) log.removeChild(log.firstChild);
}

function updateSatStatus(data) {
    var badge    = document.getElementById('satLinkBadge');
    var dot      = document.getElementById('satDot');
    var statusTx = document.getElementById('satStatusText');
    var chatSec  = document.getElementById('satChatSection');
    var infoRow  = document.getElementById('satInfoRow');
    var btnConn  = document.getElementById('btnSatConnect');
    var btnDisc  = document.getElementById('btnSatDisconnect');

    // ESP-NOW is connectionless — no WiFi or WS handshake states.
    // "connected" = user has activated the link (enabled=true).
    satConnected = data.enabled === true;

    if (satConnected) {
        badge.className      = 'sat-badge online';
        badge.textContent    = 'ONLINE';
        dot.className        = 'sat-dot live';
        statusTx.textContent = 'ESP-NOW Active';
        chatSec.style.display = 'block';
        infoRow.style.display = 'flex';
        btnConn.style.display = 'none';
        btnDisc.style.display = 'inline-block';
        logActivity('SAT LINK ONLINE', '#10b981');
    } else {
        badge.className      = 'sat-badge offline';
        badge.textContent    = 'OFFLINE';
        dot.className        = 'sat-dot';
        statusTx.textContent = 'ESP-NOW Inactive';
        chatSec.style.display = 'none';
        infoRow.style.display = 'none';
        btnConn.style.display = 'inline-block';
        btnDisc.style.display = 'none';
        logActivity('SAT LINK OFFLINE', '#ef4444');
    }
}

// ─── Command sender with animation ──────────────────────────────
function sendCmd(type, srcEl, color) {
    if (!color) color = '#3b82f6';
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: type }));
        if (srcEl) triggerCommandEffect(srcEl, color);
        var info = cmdLabels[type];
        logActivity(info ? info.label : type, color);
        triggerHexEffect(type, color);
    }
}

// ─── MQ135 → AQI + CO₂ conversion ───────────────────────────────
// Piecewise linear mapping from raw ADC (0-1023) to AQI and CO₂ ppm.
// Values are estimates only — MQ135 is not calibrated for a single gas.
function rawToGasMetrics(raw) {
    var aqi;
    if (raw < 300)      aqi = raw / 300 * 50;
    else if (raw < 500) aqi = 50  + (raw - 300) / 200 * 50;
    else if (raw < 700) aqi = 100 + (raw - 500) / 200 * 50;
    else if (raw < 900) aqi = 150 + (raw - 700) / 200 * 50;
    else                aqi = Math.min(200 + (raw - 900) / 124 * 100, 500);
    aqi = Math.round(aqi);

    var co2;
    if (raw < 200)      co2 = 400;
    else if (raw < 500) co2 = 400  + (raw - 200) / 300 * 400;
    else if (raw < 800) co2 = 800  + (raw - 500) / 300 * 1200;
    else                co2 = Math.min(2000 + (raw - 800) / 224 * 3000, 9999);
    co2 = Math.round(co2);

    return { aqi: aqi, co2: co2 };
}

function aqiCategory(aqi) {
    if (aqi <= 50)  return { label: 'GOOD',      color: '#10b981' };
    if (aqi <= 100) return { label: 'MODERATE',  color: '#f59e0b' };
    if (aqi <= 150) return { label: 'SENSITIVE',  color: '#f97316' };
    if (aqi <= 200) return { label: 'UNHEALTHY', color: '#ef4444' };
    return              { label: 'HAZARDOUS', color: '#a855f7' };
}

function co2Label(co2) {
    if (co2 < 600)  return 'FRESH';
    if (co2 < 1000) return 'NORMAL';
    if (co2 < 2000) return 'HIGH';
    return 'DANGER';
}

// ─── Sensors ─────────────────────────────────────────────────────
function deskel(id) {
    var el = document.getElementById(id);
    if (el) { el.classList.remove('skel'); el.classList.remove('skel-bar'); }
}

function updateSensors(data) {
    if (data.temp !== undefined) {
        deskel('tempValue'); deskel('tempBar');
        document.getElementById('tempValue').textContent = data.temp.toFixed(1);
        document.getElementById('tempBar').style.width = Math.min(data.temp / 60 * 100, 100) + '%';
    }
    if (data.hum !== undefined) {
        deskel('humValue'); deskel('humBar');
        document.getElementById('humValue').textContent = data.hum.toFixed(0);
        document.getElementById('humBar').style.width = Math.min(data.hum, 100) + '%';
    }
    if (data.gas !== undefined) {
        deskel('aqiValue'); deskel('aqiCat'); deskel('aqiBar');
        deskel('co2Value'); deskel('co2Status'); deskel('co2Bar');
        var m = rawToGasMetrics(data.gas);
        var cat = aqiCategory(m.aqi);
        // AQI box
        document.getElementById('aqiValue').textContent = m.aqi;
        var aqiCatEl = document.getElementById('aqiCat');
        aqiCatEl.textContent  = cat.label;
        aqiCatEl.style.color  = cat.color;
        document.getElementById('aqiBar').style.width = Math.min(m.aqi / 300 * 100, 100) + '%';
        // CO₂ box
        document.getElementById('co2Value').textContent = m.co2 >= 9999 ? '>9999' : m.co2;
        document.getElementById('co2Bar').style.width = Math.min(m.co2 / 5000 * 100, 100) + '%';
        document.getElementById('co2Status').textContent = co2Label(m.co2);
    }
    if (data.rssi !== undefined) {
        deskel('rssiVal'); deskel('wifiQual');
        var rssi = data.rssi;
        document.getElementById('rssiVal').textContent = rssi + ' dBm';
        // Signal bars: -60=4 bars, -70=3, -80=2, -90=1, worse=0
        var bars = rssi >= -60 ? 4 : rssi >= -70 ? 3 : rssi >= -80 ? 2 : rssi >= -90 ? 1 : 0;
        for (var i = 1; i <= 4; i++) {
            var b = document.getElementById('wsb' + i);
            if (b) b.classList.toggle('active', i <= bars);
        }
        var qual = bars === 4 ? 'STRONG' : bars === 3 ? 'GOOD' : bars === 2 ? 'FAIR' : bars === 1 ? 'WEAK' : 'POOR';
        var qualEl = document.getElementById('wifiQual');
        if (qualEl) qualEl.textContent = qual;
    }
}

// ─── Camera servo ────────────────────────────────────────────────
function setCamServo(axis, val) {
    val = parseInt(val);
    if (axis === 'pan')  document.getElementById('panVal').textContent  = val + '\u00B0';
    if (axis === 'tilt') document.getElementById('tiltVal').textContent = val + '\u00B0';
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'setCamServo', axis: axis, angle: val }));
    }
}

function centerCam() {
    setCamServo('pan', 90);
    setCamServo('tilt', 90);
    document.getElementById('panSlider').value  = 90;
    document.getElementById('tiltSlider').value = 90;
    document.getElementById('panVal').textContent  = '90\u00B0';
    document.getElementById('tiltVal').textContent = '90\u00B0';
}

// ─── Speed ───────────────────────────────────────────────────────
function updateSpeed(speed) {
    currentSpeed = parseInt(speed);
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'setSpeed', speed: currentSpeed }));
    }
}
function updateSpeedDisplay(speed) {
    document.getElementById('speedValue').textContent  = speed + 'ms';
    document.getElementById('paramSpeed').textContent  = speed + 'ms';
}
function updateSpeedFromServer(speed) {
    currentSpeed = speed;
    document.getElementById('speedSlider').value       = speed;
    document.getElementById('speedValue').textContent  = speed + 'ms';
    document.getElementById('paramSpeed').textContent  = speed + 'ms';
}

// ─── Stride ──────────────────────────────────────────────────────
function updateStride(stride) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'setStride', stride: parseInt(stride) }));
    }
}
function updateStrideDisplay(stride) {
    document.getElementById('strideValue').textContent  = stride + '\u00B0';
    document.getElementById('paramStride').textContent  = stride + '\u00B0';
}
function updateStrideFromServer(stride) {
    document.getElementById('strideSlider').value       = stride;
    document.getElementById('strideValue').textContent  = stride + '\u00B0';
    document.getElementById('paramStride').textContent  = stride + '\u00B0';
}

// ─── Height ──────────────────────────────────────────────────────
function updateHeight(height) {
    height = parseInt(height);
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'setHeight', height: height }));
    }
}
function updateHeightDisplay(height) {
    var h = parseInt(height);
    var display = (h >= 0 ? '+' : '') + h;
    document.getElementById('heightValue').textContent  = display;
    document.getElementById('paramHeight').textContent  = display;
}

// ─── Walk Status ─────────────────────────────────────────────────
function updateWalkStatus(isWalking, step, direction) {
    var el = document.getElementById('walkStatus');
    var lbl = document.getElementById('hexStatusLabel');
    if (isWalking) {
        var dir = 'FORWARD';
        switch (direction) {
            case 1: dir = 'BACKWARD'; break;
            case 2: dir = 'RIGHT';    break;
            case 3: dir = 'LEFT';     break;
        }
        el.style.display = 'block';
        el.textContent   = dir + ' \u2014 STEP ' + (step + 1) + '/4';
        walking = true;
        lbl.textContent = dir;
        lbl.classList.add('active');
        startWalkAnim(direction);
    } else {
        el.style.display = 'none';
        walking = false;
        lbl.textContent = 'IDLE';
        lbl.classList.remove('active');
        stopWalkAnim();
    }
}

// ─── Servo control ───────────────────────────────────────────────
function setServo(servo, angle) {
    angle = parseInt(angle);
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify({ type: 'setServo', servo: servo, angle: angle }));
    }
}

function setAllServos(angle) {
    if (sweeping || walking) return;
    for (var i = 0; i < 18; i++) {
        var slider = document.getElementById('slider' + i);
        if (slider) { slider.value = angle; updateValueDisplay(i, angle); }
        setServo(i, angle);
    }
}

function updateValueDisplay(servo, angle) {
    var el = document.getElementById('value' + servo);
    if (el) el.textContent = angle + '\u00B0';
}

function updateServoDisplay(servo, angle, pwm) {
    servoData[servo] = { angle: angle, pwm: pwm };
    var valEl    = document.getElementById('value'  + servo);
    var pwmEl    = document.getElementById('pwm'    + servo);
    var sliderEl = document.getElementById('slider' + servo);
    if (valEl)    valEl.textContent    = angle + '\u00B0';
    if (pwmEl)    pwmEl.textContent    = 'PWM: ' + pwm;
    if (sliderEl) sliderEl.value       = angle;
}

// ─── Servo modal ─────────────────────────────────────────────────
function openServoModal() {
    document.getElementById('servoModal').classList.add('active');
    if (!document.getElementById('pca2Grid').hasChildNodes()) createServoControls();
}
function closeServoModal() {
    document.getElementById('servoModal').classList.remove('active');
}
document.addEventListener('keydown', function (e) {
    if (e.key === 'Escape') closeServoModal();
});

function createServoControls() {
    var pca1Grid = document.getElementById('pca1Grid');
    var pca2Grid = document.getElementById('pca2Grid');
    var jointNames = ['Coxa', 'Femur', 'Tibia'];

    for (var i = 0; i < 9; i++) {
        var legNum  = Math.floor(i / 3) + 1;
        var jointNum = i % 3;
        var lbl = legLabels[legNum];
        pca2Grid.appendChild(createServoCard(i, 2, i,
            'L' + legNum + ' ' + jointNames[jointNum] + ' \u2014 ' + lbl.name + ' (' + lbl.code + ')'));
    }
    for (var i = 9; i < 18; i++) {
        var legNum  = Math.floor((i - 9) / 3) + 4;
        var jointNum = (i - 9) % 3;
        var lbl = legLabels[legNum];
        pca1Grid.appendChild(createServoCard(i, 1, i - 9,
            'L' + legNum + ' ' + jointNames[jointNum] + ' \u2014 ' + lbl.name + ' (' + lbl.code + ')'));
    }
}

function createServoCard(servoIndex, pcaNum, channelNum, jointName) {
    var card      = document.createElement('div');
    card.className = 'servo-card';
    var initAngle = servoData[servoIndex] ? servoData[servoIndex].angle : 90;
    var initPwm   = servoData[servoIndex] ? servoData[servoIndex].pwm   : 400;
    card.innerHTML =
        '<div class="servo-title">'      + jointName + '</div>' +
        '<div class="controller-info">PCA' + pcaNum + ' Ch' + channelNum + ' (Servo ' + servoIndex + ')</div>' +
        '<div class="servo-controls">' +
          '<input type="range" min="0" max="180" value="' + initAngle + '" class="servo-slider" id="slider' + servoIndex + '"' +
          ' onchange="setServo(' + servoIndex + ',this.value)" oninput="updateValueDisplay(' + servoIndex + ',this.value)">' +
          '<div class="servo-value" id="value' + servoIndex + '">' + initAngle + '\u00B0</div>' +
        '</div>' +
        '<div class="pwm-info">' +
          '<span id="pwm' + servoIndex + '">PWM: ' + initPwm + '</span>' +
          '<span>0x' + (pcaNum === 1 ? '40' : '41') + '</span>' +
        '</div>';
    servoData[servoIndex] = servoData[servoIndex] || { angle: 90, pwm: 400 };
    return card;
}

// ─── Activity Log ─────────────────────────────────────────────────
function logActivity(text, color) {
    var log = document.getElementById('activityLog');
    if (!log) return;
    var now = new Date();
    var ts  = ('0' + now.getHours()).slice(-2)   + ':' +
              ('0' + now.getMinutes()).slice(-2)  + ':' +
              ('0' + now.getSeconds()).slice(-2);
    var entry = document.createElement('div');
    entry.className = 'log-entry';
    entry.style.borderLeftColor = color || '#3b82f6';
    entry.innerHTML =
        '<span class="log-time">' + ts + '</span>' +
        '<span class="log-cmd" style="color:' + (color || '#3b82f6') + '">' + text + '</span>';
    log.appendChild(entry);
    // Keep max 30 entries
    while (log.children.length > 30) log.removeChild(log.firstChild);
    log.scrollTop = log.scrollHeight;
}

// ─── Command Effect (ripple + particles) ─────────────────────────
function triggerCommandEffect(el, color) {
    var layer = document.getElementById('animLayer');
    var rect  = el.getBoundingClientRect();
    var cx    = rect.left + rect.width  / 2;
    var cy    = rect.top  + rect.height / 2;

    // Ripple ring
    var ripple = document.createElement('div');
    ripple.className = 'cmd-ripple';
    ripple.style.cssText =
        'left:' + cx + 'px;top:' + cy + 'px;' +
        'border:2px solid ' + color + ';' +
        'box-shadow:0 0 12px ' + color + ';';
    layer.appendChild(ripple);
    setTimeout(function() { ripple.remove(); }, 600);

    // Floating particles
    var count = 10;
    for (var i = 0; i < count; i++) {
        (function() {
            var p   = document.createElement('div');
            var dx  = (Math.random() - 0.5) * 80;
            var dy  = -(40 + Math.random() * 80);
            var dur = (0.5 + Math.random() * 0.5).toFixed(2);
            p.className = 'cmd-particle';
            p.style.cssText =
                'left:' + cx + 'px;top:' + cy + 'px;' +
                'background:' + color + ';' +
                'box-shadow:0 0 6px ' + color + ';' +
                '--dx:' + dx + 'px;--dy:' + dy + 'px;--dur:' + dur + 's;';
            layer.appendChild(p);
            setTimeout(function() { p.remove(); }, parseFloat(dur) * 1000 + 50);
        })();
    }
}

// ─── Hexapod SVG ground-signal effect ────────────────────────────
// Foot positions in SVG viewBox coords mapped to page coords
var footSVGCoords = [
    [18,18],[12,50],[18,82],
    [102,18],[108,50],[102,82]
];

function triggerHexEffect(cmdType, color) {
    var svg = document.getElementById('hexSvg');
    if (!svg) return;
    var svgRect = svg.getBoundingClientRect();
    var vbW = 120, vbH = 100;
    var scaleX = svgRect.width  / vbW;
    var scaleY = svgRect.height / vbH;
    var layer  = document.getElementById('animLayer');

    // Determine which feet to animate based on command
    var feet;
    if (cmdType === 'startWalk' || cmdType === 'startWalkBackward') {
        feet = [0,1,2,3,4,5]; // all feet
    } else if (cmdType === 'startTurnLeft') {
        feet = [0,2,4];
    } else if (cmdType === 'startTurnRight') {
        feet = [1,3,5];
    } else {
        feet = [0,1,2,3,4,5];
    }

    feet.forEach(function(fi, idx) {
        setTimeout(function() {
            var fc  = footSVGCoords[fi];
            var px  = svgRect.left + fc[0] * scaleX;
            var py  = svgRect.top  + fc[1] * scaleY;
            // Body center in page coords
            var bx  = svgRect.left + 60 * scaleX;
            var by  = svgRect.top  + 50 * scaleY;
            // Direction vector from foot toward body
            var dx  = bx - px;
            var dy  = by - py;

            // Spawn 3 signal dots per foot, each delayed slightly
            for (var j = 0; j < 3; j++) {
                (function(jj) {
                    setTimeout(function() {
                        var dot = document.createElement('div');
                        dot.className = 'cmd-particle';
                        var t   = (0.3 + jj * 0.15).toFixed(2);
                        dot.style.cssText =
                            'left:' + px + 'px;top:' + py + 'px;' +
                            'width:4px;height:4px;' +
                            'background:' + color + ';' +
                            'box-shadow:0 0 8px ' + color + ';' +
                            '--dx:' + (dx * 0.8).toFixed(1) + 'px;' +
                            '--dy:' + (dy * 0.8).toFixed(1) + 'px;' +
                            '--dur:' + t + 's;';
                        layer.appendChild(dot);
                        setTimeout(function() { dot.remove(); }, parseFloat(t)*1000 + 50);
                    }, jj * 100);
                })(j);
            }
        }, idx * 60);
    });
}

// ─── Hex SVG walking animation ────────────────────────────────────
function startWalkAnim() {
    stopWalkAnim();
    setHexLegsActive(false); // reset first
    walkAnimStep = 0;
    walkAnimInterval = setInterval(function() {
        // Alternate leg groups: A up when step is even, B up when odd
        var groupAup = (walkAnimStep % 2 === 0);
        legGroupA.forEach(function(id) {
            var el = document.getElementById(id);
            if (el) el.classList.toggle('active', groupAup);
        });
        legGroupB.forEach(function(id) {
            var el = document.getElementById(id);
            if (el) el.classList.toggle('active', !groupAup);
        });
        // Foot dots
        var footEls = document.querySelectorAll('.hex-foot');
        footGroupA.forEach(function(fi) {
            if (footEls[fi]) footEls[fi].classList.toggle('active', groupAup);
        });
        footGroupB.forEach(function(fi) {
            if (footEls[fi]) footEls[fi].classList.toggle('active', !groupAup);
        });
        walkAnimStep++;
    }, 500);
}

function stopWalkAnim() {
    if (walkAnimInterval) {
        clearInterval(walkAnimInterval);
        walkAnimInterval = null;
    }
    setHexLegsActive(false);
}

function setHexLegsActive(on) {
    for (var i = 1; i <= 6; i++) {
        var el = document.getElementById('leg' + i);
        if (el) el.classList.toggle('active', on);
    }
    var footEls = document.querySelectorAll('.hex-foot');
    footEls.forEach(function(f) { f.classList.toggle('active', on); });
}

// ─── Camera helpers ───────────────────────────────────────────────
var camFpsCount = 0;
var camFpsTimer = Date.now();
var camAlive    = false;
var crosshairOn = false;

function camFrameLoaded() {
    camAlive = true;
    camFpsCount++;
    var now = Date.now();
    if (now - camFpsTimer >= 1000) {
        document.getElementById('camFps').textContent = camFpsCount + ' FPS';
        camFpsCount = 0;
        camFpsTimer = now;
    }
    document.getElementById('camLiveBadge').classList.add('live');
    document.getElementById('camError').style.display = 'none';
    var img = document.getElementById('cameraStream');
    img.style.display = 'block';
}

function camFail() {
    camAlive = false;
    document.getElementById('camLiveBadge').classList.remove('live');
    document.getElementById('camFps').textContent = '-- FPS';
    document.getElementById('cameraStream').style.display = 'none';
    document.getElementById('camError').style.display = 'flex';
}

function toggleCrosshair() {
    crosshairOn = !crosshairOn;
    var ch  = document.getElementById('camCrosshair');
    var btn = document.getElementById('btnCrosshair');
    ch.classList.toggle('show', crosshairOn);
    btn.classList.toggle('active', crosshairOn);
}

function reloadCam() {
    var img = document.getElementById('cameraStream');
    var src = img.src.split('?')[0];
    img.src = src + '?t=' + Date.now();
    document.getElementById('camError').style.display = 'none';
    img.style.display = 'block';
}

// Keep cam overlay angle labels in sync with sliders
function setCamServoWithOverlay(axis, val) {
    val = parseInt(val);
    setCamServo(axis, val);
    if (axis === 'pan')  document.getElementById('camOverlayPan').textContent  = 'PAN '  + val + '\u00B0';
    if (axis === 'tilt') document.getElementById('camOverlayTilt').textContent = 'TILT ' + val + '\u00B0';
}

// ─── D-Pad active-state flash ────────────────────────────────────
var dpadPressTimer = null;
function flashDpad(dir) {
    var id  = 'dbtn-' + dir;
    var btn = document.getElementById(id);
    if (!btn) return;
    document.querySelectorAll('.dpad-btn').forEach(function(b) { b.classList.remove('pressed'); });
    btn.classList.add('pressed');
    clearTimeout(dpadPressTimer);
    dpadPressTimer = setTimeout(function() { btn.classList.remove('pressed'); }, 350);
}

// ─── Robot state badge ───────────────────────────────────────────
function setRobotState(label, active) {
    var b = document.getElementById('robotStateBadge');
    if (!b) return;
    b.textContent = label;
    b.classList.toggle('moving', active);
}

// ─── Keyboard controls ───────────────────────────────────────────
var keyHeld = {};   // prevent repeat-fire for held keys
var CAM_STEP = 5;   // degrees per arrow key press

document.addEventListener('keydown', function(e) {
    // Ignore when typing in an input
    if (e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;

    // Arrow keys → camera pan/tilt (allow key-repeat for smooth movement)
    if (e.code === 'ArrowLeft' || e.code === 'ArrowRight') {
        var panEl = document.getElementById('panSlider');
        var newPan = parseInt(panEl.value) + (e.code === 'ArrowRight' ? CAM_STEP : -CAM_STEP);
        newPan = Math.max(0, Math.min(180, newPan));
        panEl.value = newPan;
        setCamServoWithOverlay('pan', newPan);
        e.preventDefault(); return;
    }
    if (e.code === 'ArrowUp' || e.code === 'ArrowDown') {
        var tiltEl = document.getElementById('tiltSlider');
        var newTilt = parseInt(tiltEl.value) + (e.code === 'ArrowDown' ? CAM_STEP : -CAM_STEP);
        newTilt = Math.max(0, Math.min(180, newTilt));
        tiltEl.value = newTilt;
        setCamServoWithOverlay('tilt', newTilt);
        e.preventDefault(); return;
    }

    // Movement keys — single-fire (no repeat)
    if (keyHeld[e.code]) return;
    keyHeld[e.code] = true;

    switch (e.code) {
        case 'KeyW':
            flashDpad('up');
            sendCmd('startWalk', null, '#3b82f6');
            e.preventDefault(); break;
        case 'KeyS':
            flashDpad('down');
            sendCmd('startWalkBackward', null, '#3b82f6');
            e.preventDefault(); break;
        case 'KeyA':
            flashDpad('left');
            sendCmd('startTurnLeft', null, '#06b6d4');
            e.preventDefault(); break;
        case 'KeyD':
            flashDpad('right');
            sendCmd('startTurnRight', null, '#06b6d4');
            e.preventDefault(); break;
        case 'Space':
            flashDpad('center');
            sendCmd('stopWalk', null, '#ef4444');
            e.preventDefault(); break;
        case 'KeyC':
            toggleCrosshair(); break;
        case 'KeyR':
            reloadCam(); break;
    }
});

document.addEventListener('keyup', function(e) {
    delete keyHeld[e.code];
});

// ─── Override setCamServo calls from HTML to also update overlay ──
// Patch the pan/tilt sliders via oninput after DOM is ready
document.addEventListener('DOMContentLoaded', function() {
    // Assign stream src here — not in HTML — so onload/onerror callbacks are
    // guaranteed to fire only after script.js is fully executed and both
    // camFrameLoaded() and camFail() are defined. Setting src in HTML causes
    // the MJPEG stream to fire onload before the <script> tag at the bottom
    // of <body> has been parsed, producing "camFrameLoaded is not defined".
    document.getElementById('cameraStream').src = 'http://192.168.5.2:81/stream';

    var pan  = document.getElementById('panSlider');
    var tilt = document.getElementById('tiltSlider');
    if (pan)  pan.addEventListener('input',  function() { document.getElementById('camOverlayPan').textContent  = 'PAN '  + this.value + '\u00B0'; });
    if (tilt) tilt.addEventListener('input', function() { document.getElementById('camOverlayTilt').textContent = 'TILT ' + this.value + '\u00B0'; });
});

// Patch updateWalkStatus to also drive the robot state badge
var _origUpdateWalkStatus = updateWalkStatus;
updateWalkStatus = function(isWalking, step, direction) {
    _origUpdateWalkStatus(isWalking, step, direction);
    if (isWalking) {
        var dir = ['FORWARD','BACKWARD','RIGHT','LEFT'][direction] || 'MOVING';
        setRobotState(dir, true);
    } else {
        setRobotState('IDLE', false);
    }
};

// ─── Init ─────────────────────────────────────────────────────────
connectWebSocket();

window.addEventListener('focus', function () {
    if (!ws || ws.readyState !== WebSocket.OPEN) connectWebSocket();
});
