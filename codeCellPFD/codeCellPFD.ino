/*
  CodeCell Wi-Fi PFD (Primary Flight Display) Visualizer

  The CodeCell creates its own Wi-Fi network and hosts a single-page
  webpage styled like an aircraft Primary Flight Display:

    - Attitude indicator (artificial horizon)    <- Roll / Pitch
    - Heading compass rose (tilt-compensated)     <- Magnetometer X/Y/Z
    - Turn coordinator (mini plane + slip ball)   <- Gyro Z (rate) + lateral Accel
    - G-meter with peak/min hold                  <- Accel X/Y/Z

  Proximity, ambient light, and step count are intentionally excluded
  from both the sensor reads and the data sent to the browser.

  A WebSocket sends 12 comma-separated values continuously at about
  30 updates per second:

    roll,pitch,yaw,accelX,accelY,accelZ,gyroX,gyroY,gyroZ,magX,magY,magZ

  The browser receives every message but only redraws the instruments
  and readouts once every 100ms (10 Hz), so the display update rate is
  decoupled from the data rate. See DISPLAY_UPDATE_INTERVAL_MS in the
  <script> block.

  Before compiling, install "WebSockets" by Markus Sattler from:
  Arduino IDE > Tools > Manage Libraries

  Instructions:
  1. Upload this sketch.
  2. Connect to Wi-Fi: CodeCell-IMU
  3. Password: codecell
  4. Open the URL printed on Serial Monitor.

  NOTE ON SIGN CONVENTIONS / CALIBRATION:
  Which way the horizon banks, and which accelerometer axis reads
  "sideways" for the turn-coordinator slip ball, depend on how the
  CodeCell is physically mounted. If the horizon banks the wrong
  direction, or the ball moves backwards relative to a real turn,
  flip ROLL_SIGN / PITCH_SIGN / LATERAL_ACCEL_AXIS near the top of
  the <script> block inside the HTML below. Nothing else needs to
  change.
*/

#include <CodeCell.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

CodeCell myCodeCell;

// Port 80 serves the webpage. Port 81 carries the live IMU data.
WebServer webServer(80);
WebSocketsServer webSocket(81);

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

float AccelX = 0.0;
float AccelY = 0.0;
float AccelZ = 0.0;

float GyroX = 0.0;
float GyroY = 0.0;
float GyroZ = 0.0;

float MagX = 0.0;
float MagY = 0.0;
float MagZ = 0.0;

// Change these if you want the CodeCell to use a different network name.
// Wi-Fi passwords must contain at least eight characters.
const char* wifiName = "CodeCell-IMU";
const char* wifiPassword = "codecell";

// PROGMEM keeps the webpage in flash instead of using normal RAM.
const char webpage[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">

<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>CodeCell PFD</title>

  <style>
    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      overflow-y: auto;
      color: #f0f0f0;
      background: radial-gradient(circle at center, #232323 0%, #0a0a0a 75%);
      font-family: Arial, Helvetica, sans-serif;
    }

    .header {
      padding: 18px 24px 6px;
      text-align: center;
    }

    h1 {
      margin: 0 0 6px;
      font-size: 24px;
      letter-spacing: 1px;
    }

    #status {
      color: #ffb380;
      font-size: 13px;
    }

    .toolbar {
      display: flex;
      justify-content: center;
      gap: 8px;
      margin-top: 10px;
    }

    .toolbar button, .small-button {
      padding: 6px 12px;
      color: inherit;
      background: rgba(255, 102, 0, 0.13);
      border: 1px solid rgba(255, 140, 70, 0.55);
      border-radius: 999px;
      cursor: pointer;
      font-weight: bold;
      font-size: 12px;
    }

    .toolbar button:hover, .small-button:hover { background: rgba(255, 102, 0, 0.28); }

    .panel {
      display: grid;
      grid-template-columns: 200px 300px 200px;
      grid-template-rows: auto auto;
      justify-content: center;
      align-items: start;
      column-gap: 26px;
      row-gap: 22px;
      padding: 30px 16px 50px;
    }

    .instrument {
      display: flex;
      flex-direction: column;
      align-items: center;
    }

    .attitude-instrument { grid-column: 2; grid-row: 1; }
    .turn-instrument      { grid-column: 1; grid-row: 1; margin-top: 40px; }
    .gmeter-instrument    { grid-column: 3; grid-row: 1; margin-top: 40px; }
    .compass-instrument   { grid-column: 2; grid-row: 2; justify-self: center; }

    .instrument-label {
      margin-top: 9px;
      font-size: 11px;
      letter-spacing: 1.5px;
      color: #ffb380;
      text-transform: uppercase;
    }

    .mini-readout {
      margin-top: 3px;
      font-size: 12px;
      color: #cfcfcf;
      font-variant-numeric: tabular-nums;
      text-align: center;
    }

    /* ---------- Attitude indicator ---------- */
    .ai-bezel {
      position: relative;
      width: 280px;
      height: 280px;
      border-radius: 50%;
      overflow: hidden;
      background: #000;
      border: 5px solid #3a3a3a;
      box-shadow: 0 0 0 2px #111, inset 0 0 24px rgba(0,0,0,0.65);
    }

    .ai-rotator {
      position: absolute;
      inset: 0;
      transform-origin: 50% 50%;
    }

    .ai-horizon-mask {
      position: absolute;
      top: 50%;
      left: 50%;
      width: 0;
      height: 0;
    }

    .ai-sky {
      position: absolute;
      left: -500px; right: -500px; bottom: 0; height: 1600px;
      background: linear-gradient(180deg, #0a3d8f, #5fa8f5);
    }

    .ai-ground {
      position: absolute;
      left: -500px; right: -500px; top: 0; height: 1600px;
      background: linear-gradient(180deg, #8a5a2a, #4a2f14);
    }

    .ai-ladder { position: absolute; top: 0; left: 0; width: 0; height: 0; }

    .ladder-line { position: absolute; left: 50%; height: 2px; background: #fff; }
    .ladder-line.major { width: 100px; margin-left: -50px; }
    .ladder-line.minor { width: 56px; margin-left: -28px; background: #ddd; }

    .ladder-label {
      position: absolute;
      width: 20px;
      color: #fff;
      font-size: 12px;
      font-weight: bold;
      text-align: center;
    }

    .ai-moving-pointer {
      position: absolute;
      top: 0; left: 50%;
      width: 0; height: 0;
      margin-left: -7px;
      border-left: 7px solid transparent;
      border-right: 7px solid transparent;
      border-top: 11px solid #fff;
    }

    .ai-fixed-pointer {
      position: absolute;
      top: 2px; left: 50%;
      width: 0; height: 0;
      margin-left: -6px;
      border-left: 6px solid transparent;
      border-right: 6px solid transparent;
      border-bottom: 10px solid #ffd45c;
      z-index: 5;
    }

    .ai-ticks { position: absolute; inset: 0; }

    .bank-tick {
      position: absolute;
      top: 50%; left: 50%;
      width: 2px; height: 14px;
      background: #ddd;
      margin-left: -1px; margin-top: -140px;
      transform-origin: 50% 140px;
    }
    .bank-tick.major { height: 18px; background: #fff; }

    .ai-wings { position: absolute; inset: 0; pointer-events: none; z-index: 4; }
    .ai-wing {
      position: absolute; top: 50%;
      width: 74px; height: 4px;
      background: #ffd45c;
      margin-top: -2px;
    }
    .ai-wing-left  { left: 20px;  clip-path: polygon(0 0, 100% 40%, 100% 60%, 0 100%); }
    .ai-wing-right { right: 20px; clip-path: polygon(0 40%, 100% 0, 100% 100%, 0 60%); }
    .ai-dot {
      position: absolute; top: 50%; left: 50%;
      width: 8px; height: 8px; margin: -4px;
      background: #ffd45c; border-radius: 50%;
    }

    /* ---------- Compass rose ---------- */
    .compass-bezel {
      position: relative;
      width: 220px; height: 220px;
      border-radius: 50%;
      overflow: hidden;
      background: #111;
      border: 5px solid #3a3a3a;
      box-shadow: 0 0 0 2px #111, inset 0 0 20px rgba(0,0,0,0.6);
    }

    .compass-card { position: absolute; inset: 0; transform-origin: 50% 50%; }

    .compass-tick {
      position: absolute;
      width: 2px; height: 12px;
      background: #ccc;
    }
    .compass-tick.major { height: 16px; background: #fff; }

    .compass-label {
      position: absolute;
      color: #fff;
      font-size: 13px;
      font-weight: bold;
    }

    .compass-lubber {
      position: absolute;
      top: 2px; left: 50%;
      width: 0; height: 0;
      margin-left: -6px;
      border-left: 6px solid transparent;
      border-right: 6px solid transparent;
      border-bottom: 10px solid #ffd45c;
      z-index: 5;
    }

    /* ---------- Turn coordinator ---------- */
    .tc-bezel {
      position: relative;
      width: 170px; height: 170px;
      border-radius: 50%;
      background: #111;
      border: 5px solid #3a3a3a;
      box-shadow: 0 0 0 2px #111, inset 0 0 18px rgba(0,0,0,0.6);
      overflow: hidden;
    }

    .tc-horizon-line {
      position: absolute;
      top: 50%; left: 15px; right: 15px;
      height: 1px;
      background: rgba(255,255,255,0.25);
    }

    .tc-plane {
      position: absolute;
      inset: 0;
      transform-origin: 50% 50%;
    }
    .tc-wing {
      position: absolute; top: 50%;
      width: 58px; height: 5px;
      background: #ffd45c;
      margin-top: -2.5px;
    }
    .tc-wing-left  { left: 15px; }
    .tc-wing-right { right: 15px; }
    .tc-fuselage {
      position: absolute; top: 50%; left: 50%;
      width: 5px; height: 34px;
      margin-left: -2.5px; margin-top: -26px;
      background: #ffd45c;
    }

    .tc-doghouse {
      position: absolute;
      bottom: 22px;
      width: 0; height: 0;
      border-left: 7px solid transparent;
      border-right: 7px solid transparent;
      border-bottom: 12px solid #888;
    }
    .tc-doghouse-left  { left: 24px;  transform: rotate(-35deg); }
    .tc-doghouse-right { right: 24px; transform: rotate(35deg); }

    .tc-ball-track {
      position: relative;
      width: 120px; height: 20px;
      margin-top: 10px;
      background: #111;
      border: 2px solid #3a3a3a;
      border-radius: 10px;
      overflow: hidden;
    }
    .tc-ball-track::before {
      content: "";
      position: absolute;
      left: 50%; top: 0; bottom: 0;
      width: 2px;
      margin-left: -1px;
      background: rgba(255,255,255,0.3);
    }
    .tc-ball {
      position: absolute;
      top: 2px; left: 50%;
      width: 16px; height: 16px;
      margin-left: -8px;
      background: radial-gradient(circle at 35% 35%, #fff, #999);
      border-radius: 50%;
      transition: transform 60ms linear;
    }

    /* ---------- G-meter ---------- */
    .gm-bezel {
      position: relative;
      width: 200px; height: 116px;
      overflow: hidden;
    }
    .gm-arc {
      position: absolute;
      left: 0; bottom: 0;
      width: 200px; height: 100px;
      border-radius: 100px 100px 0 0;
      background: conic-gradient(from -90deg at 50% 100%,
        #3fae4a 0deg, #3fae4a 90deg,
        #e0c23f 90deg, #e0c23f 130deg,
        #d94f4f 130deg, #d94f4f 180deg);
      opacity: 0.75;
    }
    .gm-needle {
      position: absolute;
      left: 50%; bottom: 0;
      width: 3px; height: 88px;
      margin-left: -1.5px;
      background: #fff;
      transform-origin: 50% 100%;
      transition: transform 60ms linear;
    }
    .gm-center-dot {
      position: absolute;
      left: 50%; bottom: -7px;
      width: 14px; height: 14px;
      margin-left: -7px;
      background: #ccc;
      border-radius: 50%;
      border: 2px solid #444;
    }
    .gm-digital {
      margin-top: 6px;
      font-size: 20px;
      font-weight: bold;
    }

    .calibrate-bar {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 6px;
      padding: 6px 16px 40px;
    }
    .calibrate-bar button {
      padding: 9px 20px;
      font-size: 13px;
    }
    .calibrate-hint {
      color: #999;
      max-width: 320px;
    }
    .calibrate-hint.calibrated {
      color: #55e6a5;
    }

    @media (max-width: 900px) {
      .panel {
        grid-template-columns: 1fr;
        justify-items: center;
      }
      .attitude-instrument, .turn-instrument, .gmeter-instrument, .compass-instrument {
        grid-column: 1;
      }
      .attitude-instrument { grid-row: 1; }
      .compass-instrument  { grid-row: 2; }
      .turn-instrument     { grid-row: 3; margin-top: 0; }
      .gmeter-instrument   { grid-row: 4; margin-top: 0; }
    }
  </style>
</head>

<body>
  <div class="header">
    <h1>CodeCell PFD</h1>
    <div id="status">Connecting...</div>
    <div class="toolbar">
      <button id="freezeButton" type="button">Freeze</button>
      <button id="fullscreenButton" type="button">Full screen</button>
    </div>
  </div>

  <div class="panel">

    <div class="instrument attitude-instrument">
      <div class="ai-bezel">
        <div class="ai-rotator" id="aiRotator">
          <div class="ai-horizon-mask" id="aiHorizonMask">
            <div class="ai-sky"></div>
            <div class="ai-ground"></div>
            <div class="ai-ladder" id="aiLadder"></div>
          </div>
          <div class="ai-moving-pointer"></div>
        </div>
        <div class="ai-fixed-pointer"></div>
        <div class="ai-ticks" id="aiTicks"></div>
        <div class="ai-wings">
          <div class="ai-wing ai-wing-left"></div>
          <div class="ai-dot"></div>
          <div class="ai-wing ai-wing-right"></div>
        </div>
      </div>
      <div class="instrument-label">Attitude</div>
      <div class="mini-readout" id="rpReadout">R 0&deg; &nbsp; P 0&deg; &nbsp; Y 0&deg;</div>
    </div>

    <div class="instrument turn-instrument">
      <div class="tc-bezel">
        <div class="tc-horizon-line"></div>
        <div class="tc-doghouse tc-doghouse-left"></div>
        <div class="tc-doghouse tc-doghouse-right"></div>
        <div class="tc-plane" id="tcPlane">
          <div class="tc-wing tc-wing-left"></div>
          <div class="tc-fuselage"></div>
          <div class="tc-wing tc-wing-right"></div>
        </div>
      </div>
      <div class="tc-ball-track">
        <div class="tc-ball" id="tcBall"></div>
      </div>
      <div class="instrument-label">Turn coordinator</div>
      <div class="mini-readout" id="gyroReadout">X 0&deg;/s Y 0&deg;/s Z 0&deg;/s</div>
    </div>

    <div class="instrument gmeter-instrument">
      <div class="gm-bezel">
        <div class="gm-arc"></div>
        <div class="gm-needle" id="gmNeedle"></div>
        <div class="gm-center-dot"></div>
      </div>
      <div class="gm-digital" id="gmDigital">1.00 G</div>
      <div class="instrument-label">G-meter</div>
      <div class="mini-readout" id="gmPeaks">MAX 1.00 &nbsp; MIN 1.00</div>
      <button id="gmResetButton" type="button" class="small-button" style="margin-top:6px;">Reset peaks</button>
      <div class="mini-readout" id="accelReadout" style="margin-top:8px;">X 0.00 Y 0.00 Z 1.00 g</div>
    </div>

    <div class="instrument compass-instrument">
      <div class="compass-bezel">
        <div class="compass-card" id="compassCard"></div>
        <div class="compass-lubber"></div>
      </div>
      <div class="instrument-label">Heading (mag)</div>
      <div class="mini-readout" id="headingReadout" style="font-size:16px;font-weight:bold;">000&deg;</div>
      <div class="mini-readout" id="magReadout">X 0.0 Y 0.0 Z 0.0 &micro;T</div>
    </div>

  </div>

  <div class="calibrate-bar">
    <button id="calibrateButton" type="button" class="small-button">Zero Roll / Pitch / Yaw</button>
    <div class="mini-readout calibrate-hint" id="calibrateStatus">Hold the board straight and level, then press to zero the attitude readings.</div>
  </div>

  <script>
    // ---- Calibration constants: flip these if the instruments read backwards ----
    const ROLL_SIGN = 1;              // Flip to -1 if the horizon banks the wrong way
    const PITCH_SIGN = 1;             // Flip to -1 if the horizon moves the wrong way on pitch-up
    const LATERAL_ACCEL_AXIS = 'y';   // 'x' or 'y' -- whichever reads sideways accel on your mount
    const PX_PER_DEG_PITCH = 5.5;
    const TURN_RATE_FULL_SCALE_DEG_S = 6;  // Yaw rate that pins the turn-coordinator plane at full deflection
    const TC_MAX_BANK_DEG = 25;
    const GMETER_MIN = -1;
    const GMETER_MAX = 3;

    // Data arrives from the WebSocket at ~30 Hz, but the page only redraws
    // the instruments/readouts this often, to keep the display update rate
    // steady and independent of the incoming data rate.
    const DISPLAY_UPDATE_INTERVAL_MS = 100;

    const statusText = document.getElementById("status");
    const freezeButton = document.getElementById("freezeButton");
    const fullscreenButton = document.getElementById("fullscreenButton");

    const aiRotator = document.getElementById("aiRotator");
    const aiHorizonMask = document.getElementById("aiHorizonMask");
    const rpReadout = document.getElementById("rpReadout");

    const compassCard = document.getElementById("compassCard");
    const headingReadout = document.getElementById("headingReadout");
    const magReadout = document.getElementById("magReadout");

    const tcPlane = document.getElementById("tcPlane");
    const tcBall = document.getElementById("tcBall");
    const gyroReadout = document.getElementById("gyroReadout");

    const gmNeedle = document.getElementById("gmNeedle");
    const gmDigital = document.getElementById("gmDigital");
    const gmPeaks = document.getElementById("gmPeaks");
    const gmResetButton = document.getElementById("gmResetButton");
    const accelReadout = document.getElementById("accelReadout");

    const calibrateButton = document.getElementById("calibrateButton");
    const calibrateStatus = document.getElementById("calibrateStatus");

    let socket;
    let reconnectTimer;
    let displayTimer;
    let frozen = false;
    let gPeakMax = 1;
    let gPeakMin = 1;

    // Most recent parsed sensor reading, applied to the page by the
    // DISPLAY_UPDATE_INTERVAL_MS render loop rather than on every message.
    let latestReading = null;

    // Zero-reference offsets set by the "Zero Roll / Pitch / Yaw" button.
    let offsetRoll = 0;
    let offsetPitch = 0;
    let offsetYaw = 0;

    // Latest raw (uncalibrated) attitude, kept fresh even while frozen so
    // the calibrate button always zeroes against the current orientation.
    let lastRoll = 0;
    let lastPitch = 0;
    let lastYaw = 0;

    function clamp(value, minimum, maximum) {
      return Math.min(maximum, Math.max(minimum, value));
    }

    // Keeps an angle within -180..180 after an offset is subtracted, so
    // calibrated values don't jump when they cross the wrap point.
    function wrapAngle180(deg) {
      let a = deg % 360;
      if (a > 180) a -= 360;
      if (a < -180) a += 360;
      return a;
    }

    // Standard tilt-compensated compass heading from raw magnetometer data.
    // Not hard/soft-iron calibrated -- treat as an approximate heading.
    function computeHeading(mx, my, mz, rollDeg, pitchDeg) {
      const rollRad = rollDeg * Math.PI / 180;
      const pitchRad = pitchDeg * Math.PI / 180;
      const cosRoll = Math.cos(rollRad), sinRoll = Math.sin(rollRad);
      const cosPitch = Math.cos(pitchRad), sinPitch = Math.sin(pitchRad);
      const xh = mx * cosPitch + mz * sinPitch;
      const yh = mx * sinRoll * sinPitch + my * cosRoll - mz * sinRoll * cosPitch;
      let heading = Math.atan2(-yh, xh) * 180 / Math.PI;
      if (heading < 0) heading += 360;
      return heading;
    }

    function buildLadder() {
      const ladder = document.getElementById("aiLadder");
      for (let d = -90; d <= 90; d += 10) {
        if (d === 0) continue;
        const major = d % 20 === 0;
        const line = document.createElement("div");
        line.className = "ladder-line " + (major ? "major" : "minor");
        line.style.top = (-d * PX_PER_DEG_PITCH) + "px";
        ladder.appendChild(line);

        if (major) {
          const labelL = document.createElement("div");
          labelL.className = "ladder-label";
          labelL.textContent = Math.abs(d);
          labelL.style.top = (-d * PX_PER_DEG_PITCH - 7) + "px";
          labelL.style.left = "-78px";
          ladder.appendChild(labelL);

          const labelR = labelL.cloneNode(true);
          labelR.style.left = "58px";
          ladder.appendChild(labelR);
        }
      }
    }

    function buildBankTicks() {
      const container = document.getElementById("aiTicks");
      const angles = [-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60];
      const majors = [-60, -30, 0, 30, 60];
      angles.forEach(a => {
        const tick = document.createElement("div");
        tick.className = "bank-tick" + (majors.includes(a) ? " major" : "");
        tick.style.transform = `rotate(${a}deg)`;
        container.appendChild(tick);
      });
    }

    function buildCompass() {
      const dirs = { 0: "N", 90: "E", 180: "S", 270: "W" };
      for (let a = 0; a < 360; a += 30) {
        const rad = a * Math.PI / 180;
        const tickRadius = 95;
        const labelRadius = 75;

        const tick = document.createElement("div");
        tick.className = "compass-tick" + (a % 90 === 0 ? " major" : "");
        tick.style.left = (110 + tickRadius * Math.sin(rad)) + "px";
        tick.style.top = (110 - tickRadius * Math.cos(rad)) + "px";
        tick.style.transform = `translate(-50%, -50%) rotate(${a}deg)`;
        compassCard.appendChild(tick);

        const label = document.createElement("div");
        label.className = "compass-label";
        label.textContent = dirs[a] !== undefined ? dirs[a] : String(a / 10);
        label.style.left = (110 + labelRadius * Math.sin(rad)) + "px";
        label.style.top = (110 - labelRadius * Math.cos(rad)) + "px";
        label.style.transform = "translate(-50%, -50%)";
        compassCard.appendChild(label);
      }
    }

    buildLadder();
    buildBankTicks();
    buildCompass();

    function connectWebSocket() {
      clearTimeout(reconnectTimer);

      statusText.textContent = "Connecting...";
      statusText.style.color = "#ffb380";

      socket = new WebSocket("ws://" + window.location.hostname + ":81/");

      socket.onopen = function() {
        statusText.textContent = "Live · 10 Hz display";
        statusText.style.color = "#ff6600";
      };

      socket.onmessage = function(event) {
        const values = event.data.split(",");

        // roll,pitch,yaw,accel xyz,gyro xyz,mag xyz
        if (values.length !== 12) return;

        const data = values.map(Number);
        if (!data.every(Number.isFinite)) return;

        const [
          roll, pitch, yaw,
          ax, ay, az,
          gx, gy, gz,
          mx, my, mz
        ] = data;

        // Keep the latest raw attitude around so the calibrate button can
        // zero against it even when the display is frozen.
        lastRoll = roll;
        lastPitch = pitch;
        lastYaw = yaw;

        // Stash the full reading; renderInstruments() picks it up on the
        // next DISPLAY_UPDATE_INTERVAL_MS tick instead of drawing here.
        latestReading = { roll, pitch, yaw, ax, ay, az, gx, gy, gz, mx, my, mz };
      };

      socket.onerror = function() {
        socket.close();
      };

      socket.onclose = function() {
        statusText.textContent = "Disconnected · reconnecting...";
        statusText.style.color = "#ff8a80";
        reconnectTimer = setTimeout(connectWebSocket, 1000);
      };
    }

    // Applies one sensor reading to every instrument and text readout.
    // Called at most once per DISPLAY_UPDATE_INTERVAL_MS, regardless of how
    // often the WebSocket delivers new data.
    function renderInstruments(reading) {
        const { roll, pitch, yaw, ax, ay, az, gx, gy, gz, mx, my, mz } = reading;

        // Apply the zero-reference offsets set by the calibrate button.
        const calRoll = wrapAngle180(roll - offsetRoll);
        const calPitch = wrapAngle180(pitch - offsetPitch);
        const calYaw = wrapAngle180(yaw - offsetYaw);

        // Attitude indicator
        aiRotator.style.transform = `rotate(${ROLL_SIGN * calRoll}deg)`;
        aiHorizonMask.style.transform = `translateY(${PITCH_SIGN * calPitch * PX_PER_DEG_PITCH}px)`;
        rpReadout.innerHTML = `R ${calRoll.toFixed(0)}&deg; &nbsp; P ${calPitch.toFixed(0)}&deg; &nbsp; Y ${calYaw.toFixed(0)}&deg;`;

        // Compass (tilt-compensated magnetometer heading)
        const headingDeg = computeHeading(mx, my, mz, roll, pitch);
        compassCard.style.transform = `rotate(${-headingDeg}deg)`;
        headingReadout.innerHTML = String(Math.round(headingDeg)).padStart(3, "0") + "&deg;";
        magReadout.innerHTML = `X ${mx.toFixed(1)} Y ${my.toFixed(1)} Z ${mz.toFixed(1)} &micro;T`;

        // Turn coordinator: bank the mini-plane with yaw rate, ball with lateral G
        const gyroZdegS = gz * 180 / Math.PI;
        const tcBank = clamp(
          gyroZdegS / TURN_RATE_FULL_SCALE_DEG_S * TC_MAX_BANK_DEG,
          -TC_MAX_BANK_DEG, TC_MAX_BANK_DEG
        );
        tcPlane.style.transform = `rotate(${tcBank}deg)`;

        const lateral = LATERAL_ACCEL_AXIS === "x" ? ax : ay;
        const ballOffset = clamp(lateral * 45, -22, 22);
        tcBall.style.transform = `translateX(${ballOffset}px)`;

        gyroReadout.innerHTML =
          `X ${(gx * 180 / Math.PI).toFixed(1)}&deg;/s ` +
          `Y ${(gy * 180 / Math.PI).toFixed(1)}&deg;/s ` +
          `Z ${(gz * 180 / Math.PI).toFixed(1)}&deg;/s`;

        // G-meter
        const totalG = Math.sqrt(ax * ax + ay * ay + az * az);
        gPeakMax = Math.max(gPeakMax, totalG);
        gPeakMin = Math.min(gPeakMin, totalG);

        const gClamped = clamp(totalG, GMETER_MIN, GMETER_MAX);
        const needleAngle = -90 + (gClamped - GMETER_MIN) / (GMETER_MAX - GMETER_MIN) * 180;
        gmNeedle.style.transform = `rotate(${needleAngle}deg)`;
        gmDigital.textContent = totalG.toFixed(2) + " G";
        gmPeaks.textContent = `MAX ${gPeakMax.toFixed(2)}   MIN ${gPeakMin.toFixed(2)}`;
        accelReadout.textContent =
          `X ${ax.toFixed(2)} Y ${ay.toFixed(2)} Z ${az.toFixed(2)} g`;
    }

    // Redraws the page from the most recent reading, at most every
    // DISPLAY_UPDATE_INTERVAL_MS, independent of the WebSocket message rate.
    clearInterval(displayTimer);
    displayTimer = setInterval(function() {
      if (frozen || !latestReading) return;
      renderInstruments(latestReading);
    }, DISPLAY_UPDATE_INTERVAL_MS);

    freezeButton.addEventListener("click", function() {
      frozen = !frozen;
      freezeButton.textContent = frozen ? "Resume" : "Freeze";
      statusText.textContent = frozen ? "Frozen · inspect readings" : "Live · 10 Hz display";
    });

    fullscreenButton.addEventListener("click", function() {
      if (!document.fullscreenElement) {
        document.documentElement.requestFullscreen();
      } else {
        document.exitFullscreen();
      }
    });

    gmResetButton.addEventListener("click", function() {
      gPeakMax = 1;
      gPeakMin = 1;
    });

    calibrateButton.addEventListener("click", function() {
      offsetRoll = lastRoll;
      offsetPitch = lastPitch;
      offsetYaw = lastYaw;

      calibrateStatus.textContent = "Zeroed - this orientation is now level.";
      calibrateStatus.classList.add("calibrated");
      setTimeout(function() {
        calibrateStatus.textContent = "Hold the board straight and level, then press to zero the attitude readings.";
        calibrateStatus.classList.remove("calibrated");
      }, 2500);
    });

    connectWebSocket();
  </script>
</body>
</html>
)HTML";

void handleWebpage() {
  // Send the webpage stored above whenever the browser opens the address.
  webServer.send_P(200, "text/html", webpage);
}

void webSocketEvent(
  uint8_t clientNumber,
  WStype_t eventType,
  uint8_t* payload,
  size_t payloadLength) {
  // These messages are useful when checking connections in Serial Monitor.
  switch (eventType) {
    case WStype_CONNECTED:
      Serial.printf("Browser %u connected\n", clientNumber);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("Browser %u disconnected\n", clientNumber);
      break;

    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);

  // Light (proximity/ambient) and the step counter are not used by this
  // visualizer, so they are left out of Init() entirely.
  myCodeCell.Init(MOTION_ROTATION + MOTION_ACCELEROMETER + MOTION_GYRO + MOTION_MAGNETOMETER);

  // Access-point mode lets a phone or computer connect without a router.
  WiFi.mode(WIFI_AP);

  // Wi-Fi sleep saves power but can make live movement appear delayed.
  WiFi.setSleep(false);

  WiFi.softAP(wifiName, wifiPassword);

  Serial.println();
  Serial.println("CodeCell PFD visualizer ready");

  Serial.print("Wi-Fi: ");
  Serial.println(wifiName);

  Serial.print("Password: ");
  Serial.println(wifiPassword);

  Serial.print("Open: http://");
  Serial.println(WiFi.softAPIP());

  // Start the webpage and live-data servers.
  webServer.on("/", handleWebpage);
  webServer.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
}

void loop() {
  // These must run frequently to keep browser connections responsive.
  webServer.handleClient();
  webSocket.loop();

  // Read the sensors and send one CSV message about 30 times per second.
  if (myCodeCell.Run(30)) {
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);
    myCodeCell.Motion_AccelerometerRead(AccelX, AccelY, AccelZ);
    myCodeCell.Motion_GyroRead(GyroX, GyroY, GyroZ);
    myCodeCell.Motion_MagnetometerRead(MagX, MagY, MagZ);

    char sensorData[180];

    snprintf(
      sensorData,
      sizeof(sensorData),
      "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
      Roll,
      Pitch,
      Yaw,
      AccelX,
      AccelY,
      AccelZ,
      GyroX,
      GyroY,
      GyroZ,
      MagX,
      MagY,
      MagZ
    );

    webSocket.broadcastTXT(sensorData);
  }
}
