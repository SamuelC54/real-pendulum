import './style.css';
import Two from 'two.js';

// Configuration
const WEBSOCKET_URL = 'ws://127.0.0.1:8765';
const PENDULUM_LENGTH = 200;  // pixels
const BOB_RADIUS = 25;
const PIVOT_RADIUS = 12;

// State
let currentAngle = 0;  // degrees (0 = up, 180 = down)
let angularVelocity = 0;
let currentPosition = 0;  // stepper position
let motorVelocity = 0;
let limitLeft = false;
let limitRight = false;
let currentMode = 'idle';

// Position tracking - will be calibrated based on observed min/max
let minPosition = -5000;
let maxPosition = 5000;

// DOM elements
const container = document.getElementById('pendulum-container')!;
const statusEl = document.getElementById('status')!;
const angleValueEl = document.getElementById('angle-value')!;
const velocityValueEl = document.getElementById('velocity-value')!;
const motorValueEl = document.getElementById('motor-value');
const positionValueEl = document.getElementById('position-value');
const positionMarkerEl = document.getElementById('position-marker');
const limitLeftEl = document.getElementById('limit-left');
const limitRightEl = document.getElementById('limit-right');
const loadingSpinnerEl = document.getElementById('loading-spinner');
const currentModeEl = document.getElementById('current-mode');

// Initialize Two.js
const two = new Two({
  type: Two.Types.svg,
  width: container.clientWidth,
  height: container.clientHeight,
  autostart: true
}).appendTo(container);

// Center point (pivot location)
const centerX = two.width / 2;
const centerY = two.height / 2 - 50;

// Rail configuration
const RAIL_WIDTH = two.width * 0.7;
const RAIL_Y = centerY - 30;
const CART_WIDTH = 60;
const CART_HEIGHT = 30;

// Create rail
const rail = two.makeLine(
  centerX - RAIL_WIDTH / 2, RAIL_Y,
  centerX + RAIL_WIDTH / 2, RAIL_Y
);
rail.stroke = 'rgba(255, 255, 255, 0.3)';
rail.linewidth = 4;
rail.cap = 'round';

// Limit switch indicators on rail
const limitLeftIndicator = two.makeCircle(centerX - RAIL_WIDTH / 2 + 10, RAIL_Y, 8);
limitLeftIndicator.fill = 'rgba(100, 100, 100, 0.5)';
limitLeftIndicator.stroke = 'rgba(255, 255, 255, 0.2)';
limitLeftIndicator.linewidth = 2;

const limitRightIndicator = two.makeCircle(centerX + RAIL_WIDTH / 2 - 10, RAIL_Y, 8);
limitRightIndicator.fill = 'rgba(100, 100, 100, 0.5)';
limitRightIndicator.stroke = 'rgba(255, 255, 255, 0.2)';
limitRightIndicator.linewidth = 2;

// Cart
const cart = two.makeRoundedRectangle(centerX, RAIL_Y, CART_WIDTH, CART_HEIGHT, 5);
cart.fill = 'rgba(0, 212, 255, 0.3)';
cart.stroke = 'var(--accent-cyan)';
cart.linewidth = 2;

// Pendulum group
const pendulumGroup = two.makeGroup();
pendulumGroup.translation.set(centerX, centerY);

// Rod
const rod = two.makeLine(0, 0, 0, PENDULUM_LENGTH);
rod.stroke = '#4a9eff';
rod.linewidth = 4;
rod.cap = 'round';
pendulumGroup.add(rod);

// Bob glow
const bobGlow = two.makeCircle(0, PENDULUM_LENGTH, BOB_RADIUS + 10);
bobGlow.fill = 'rgba(255, 107, 53, 0.2)';
bobGlow.noStroke();
pendulumGroup.add(bobGlow);

// Bob
const bob = two.makeCircle(0, PENDULUM_LENGTH, BOB_RADIUS);
bob.fill = '#ff6b35';
bob.stroke = '#cc5529';
bob.linewidth = 3;
pendulumGroup.add(bob);

// Pivot glow
const pivotGlow = two.makeCircle(0, 0, PIVOT_RADIUS + 8);
pivotGlow.fill = 'rgba(0, 212, 255, 0.2)';
pivotGlow.noStroke();
pendulumGroup.add(pivotGlow);

// Pivot point
const pivot = two.makeCircle(0, 0, PIVOT_RADIUS);
pivot.fill = '#00d4ff';
pivot.stroke = '#00a8cc';
pivot.linewidth = 2;
pendulumGroup.add(pivot);

// Reference line
const arcRadius = 60;
const refLine = two.makeLine(0, 0, 0, arcRadius + 20);
refLine.stroke = 'rgba(255, 255, 255, 0.2)';
refLine.linewidth = 1;
refLine.dashes = [5, 5];
pendulumGroup.add(refLine);

// Angle text
const angleText = two.makeText('0°', 80, 40);
angleText.fill = '#ff00aa';
angleText.size = 16;
angleText.family = 'JetBrains Mono, monospace';
pendulumGroup.add(angleText);

let currentArc: any = null;

// Update position display and cart visualization
function updatePositionDisplay(position: number, limLeft: boolean, limRight: boolean) {
  if (positionValueEl) {
    positionValueEl.textContent = position.toString();
  }
  
  // Auto-calibrate min/max
  if (position < minPosition) minPosition = position;
  if (position > maxPosition) maxPosition = position;
  
  const range = maxPosition - minPosition;
  let normalizedPos = 0.5;
  if (range > 0) {
    normalizedPos = (position - minPosition) / range;  // Left = small, Right = large
  }
  
  if (positionMarkerEl) {
    const percent = normalizedPos * 100;
    const clampedPercent = Math.max(10, Math.min(90, percent));
    positionMarkerEl.style.left = `${clampedPercent}%`;
  }
  
  if (limitLeftEl) {
    limitLeftEl.classList.toggle('active', limLeft);
  }
  if (limitRightEl) {
    limitRightEl.classList.toggle('active', limRight);
  }
  
  // Update cart position in visualization
  const cartX = centerX - RAIL_WIDTH / 2 + RAIL_WIDTH * normalizedPos;
  cart.translation.x = cartX;
  pendulumGroup.translation.x = cartX;
  
  // Update limit indicators
  if (limLeft) {
    limitLeftIndicator.fill = '#ff4444';
    minPosition = position;
  } else {
    limitLeftIndicator.fill = 'rgba(100, 100, 100, 0.5)';
  }
  
  if (limRight) {
    limitRightIndicator.fill = '#ff4444';
    maxPosition = position;
  } else {
    limitRightIndicator.fill = 'rgba(100, 100, 100, 0.5)';
  }
}

// Update pendulum visualization
function updatePendulum(angleDeg: number) {
  const angleRad = angleDeg * (Math.PI / 180);
  pendulumGroup.rotation = angleRad;
  
  const startAngle = Math.PI / 2;
  const endAngle = startAngle + angleRad;
  
  const currentCartX = pendulumGroup.translation.x;
  
  if (currentArc) {
    two.remove(currentArc);
    currentArc = null;
  }
  
  if (Math.abs(angleDeg) > 0.5) {
    currentArc = two.makeArcSegment(
      currentCartX, centerY,
      arcRadius - 5, arcRadius + 5,
      Math.min(startAngle, endAngle),
      Math.max(startAngle, endAngle)
    );
    currentArc.fill = 'rgba(255, 0, 170, 0.3)';
    currentArc.stroke = '#ff00aa';
    currentArc.linewidth = 2;
  }
  
  const textAngle = startAngle + angleRad / 2;
  angleText.translation.set(
    (arcRadius + 35) * Math.cos(textAngle),
    (arcRadius + 35) * Math.sin(textAngle)
  );
  angleText.value = `${angleDeg.toFixed(1)}°`;
}

// WebSocket connection
let ws: WebSocket | null = null;
let reconnectTimeout: number | null = null;

function sendMode(mode: string) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'MODE', mode }));
    console.log('Set mode:', mode);
  }
}

function sendStop() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'STOP' }));
    console.log('Stop');
  }
}

function sendZero() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'ZERO' }));
    console.log('Zero');
  }
}

function sendConfig(key: string, value: number | string) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'CONFIG', [key]: value }));
    console.log('Config:', key, '=', value);
  }
}

function sendUpload() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'UPLOAD' }));
    console.log('Upload requested');
  }
}

// Upload button handling
const uploadBtn = document.getElementById('btn-upload');
const uploadStatus = document.getElementById('upload-status');

function showUploadStatus(message: string, type: 'uploading' | 'success' | 'error') {
  if (uploadStatus) {
    uploadStatus.textContent = message;
    uploadStatus.className = 'visible ' + type;
    
    if (type !== 'uploading') {
      setTimeout(() => {
        uploadStatus.classList.remove('visible');
      }, 5000);
    }
  }
}

if (uploadBtn) {
  uploadBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      uploadBtn.classList.add('uploading');
      showUploadStatus('Uploading... please wait', 'uploading');
      sendUpload();
    } else {
      showUploadStatus('Not connected to controller', 'error');
    }
  });
}

function updateModeDisplay(mode: string) {
  currentMode = mode;
  if (currentModeEl) {
    currentModeEl.textContent = mode;
    currentModeEl.className = mode === 'idle' ? '' : 'active';
  }
  
  // Update toggle buttons
  document.querySelectorAll('.toggle-btn').forEach(btn => {
    const btnMode = btn.getAttribute('data-mode');
    btn.classList.toggle('active', btnMode === mode);
  });
}

function connect() {
  console.log('Attempting WebSocket connection to', WEBSOCKET_URL);
  statusEl.textContent = 'Connecting...';
  statusEl.classList.remove('connected');
  
  try {
    ws = new WebSocket(WEBSOCKET_URL);
    console.log('WebSocket object created, state:', ws.readyState);
    
    ws.onopen = () => {
      console.log('WebSocket onopen fired!');
      statusEl.textContent = '● Connected';
      statusEl.classList.add('connected');
      container.classList.remove('disconnected');
      container.classList.add('connected');
      if (loadingSpinnerEl) loadingSpinnerEl.classList.add('hidden');
      console.log('Connected to pendulum controller');
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
        
        // Handle different message types
        if (data.type === 'STATE') {
          // Update state from controller
          if (data.angle !== undefined) {
            currentAngle = data.angle;
            angleValueEl.textContent = currentAngle.toFixed(1);
            updatePendulum(currentAngle);
          }
          
          if (data.angular_velocity !== undefined) {
            angularVelocity = data.angular_velocity;
            velocityValueEl.textContent = angularVelocity.toFixed(1);
          }
          
          if (data.position !== undefined) {
            currentPosition = data.position;
            limitLeft = data.limit_left || false;
            limitRight = data.limit_right || false;
            updatePositionDisplay(currentPosition, limitLeft, limitRight);
          }
          
          if (data.velocity !== undefined) {
            motorVelocity = data.velocity;
            if (motorValueEl) {
              motorValueEl.textContent = motorVelocity.toString();
            }
          }
          
          if (data.mode !== undefined) {
            updateModeDisplay(data.mode);
          }
        }
        
        // Handle upload status messages
        if (data.type === 'UPLOAD_STATUS') {
          showUploadStatus(data.message, 'uploading');
        }
        
        if (data.type === 'UPLOAD_RESULT') {
          const uploadBtn = document.getElementById('btn-upload');
          if (uploadBtn) uploadBtn.classList.remove('uploading');
          
          if (data.success) {
            showUploadStatus('✓ Upload successful!', 'success');
          } else {
            showUploadStatus('✗ ' + (data.message || 'Upload failed'), 'error');
          }
        }
        
      } catch (e) {
        console.error('Failed to parse message:', e);
      }
    };
    
    ws.onclose = () => {
      statusEl.textContent = 'Disconnected - Reconnecting...';
      statusEl.classList.remove('connected');
      container.classList.remove('connected');
      container.classList.add('disconnected');
      if (loadingSpinnerEl) loadingSpinnerEl.classList.remove('hidden');
      scheduleReconnect();
    };
    
    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
      statusEl.textContent = 'Connection error';
      statusEl.classList.remove('connected');
      container.classList.remove('connected');
      container.classList.add('disconnected');
      if (loadingSpinnerEl) loadingSpinnerEl.classList.remove('hidden');
    };
  } catch (e) {
    console.error('Failed to create WebSocket:', e);
    statusEl.textContent = 'Failed to connect';
    scheduleReconnect();
  }
}

function scheduleReconnect() {
  if (reconnectTimeout) return;
  reconnectTimeout = window.setTimeout(() => {
    reconnectTimeout = null;
    connect();
  }, 2000);
}

// Setup button controls
function setupControls() {
  // Hold buttons (press to start mode, release to stop)
  document.querySelectorAll('.hold-btn').forEach(btn => {
    const startMode = btn.getAttribute('data-mode-start');
    const stopMode = btn.getAttribute('data-mode-stop');
    
    // Mouse events
    btn.addEventListener('mousedown', () => {
      btn.classList.add('active');
      if (startMode) sendMode(startMode);
    });
    
    btn.addEventListener('mouseup', () => {
      btn.classList.remove('active');
      if (stopMode) sendMode(stopMode);
    });
    
    btn.addEventListener('mouseleave', () => {
      if (btn.classList.contains('active')) {
        btn.classList.remove('active');
        if (stopMode) sendMode(stopMode);
      }
    });
    
    // Touch events
    btn.addEventListener('touchstart', (e) => {
      e.preventDefault();
      btn.classList.add('active');
      if (startMode) sendMode(startMode);
    });
    
    btn.addEventListener('touchend', () => {
      btn.classList.remove('active');
      if (stopMode) sendMode(stopMode);
    });
  });
  
  // Toggle buttons (click to toggle mode on/off)
  document.querySelectorAll('.toggle-btn').forEach(btn => {
    const mode = btn.getAttribute('data-mode');
    
    btn.addEventListener('click', () => {
      if (currentMode === mode) {
        // Turn off - go to idle
        sendMode('idle');
      } else {
        // Turn on
        if (mode) sendMode(mode);
      }
    });
  });
  
  // Stop button
  const stopBtn = document.getElementById('btn-stop');
  if (stopBtn) {
    stopBtn.addEventListener('click', () => {
      sendStop();
    });
  }
  
  // Zero button
  const zeroBtn = document.getElementById('btn-zero');
  if (zeroBtn) {
    zeroBtn.addEventListener('click', () => {
      sendZero();
    });
  }
}

// Setup configuration sliders
function setupConfigSliders() {
  document.querySelectorAll('input[type="range"][data-cfg]').forEach(slider => {
    const input = slider as HTMLInputElement;
    const cfgKey = input.getAttribute('data-cfg');
    const valueDisplay = document.getElementById('val-' + input.id.replace('cfg-', ''));
    
    // Update display on input
    input.addEventListener('input', () => {
      if (valueDisplay) {
        valueDisplay.textContent = input.value;
      }
    });
    
    // Send config on change
    input.addEventListener('change', () => {
      if (cfgKey) {
        const value = input.type === 'range' && input.step && parseFloat(input.step) < 1 
          ? parseFloat(input.value) 
          : parseInt(input.value);
        sendConfig(cfgKey, value);
      }
    });
  });
}

// Handle window resize
window.addEventListener('resize', () => {
  two.width = container.clientWidth;
  two.height = container.clientHeight;
});

// Initial render
updatePendulum(0);

// Setup controls and config sliders
setupControls();
setupConfigSliders();

// Start connection
connect();

console.log('Pendulum Digital Twin initialized (Velocity Streaming Mode)');
console.log('Waiting for WebSocket connection at', WEBSOCKET_URL);
