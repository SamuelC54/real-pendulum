import './style.css';
import Two from 'two.js';

// Configuration
const WEBSOCKET_URL = 'ws://localhost:8765';
const PENDULUM_LENGTH = 200;  // pixels
const BOB_RADIUS = 25;
const PIVOT_RADIUS = 12;

// State
let currentAngle = 0;  // degrees (0 = down, 180 = up)
let currentVelocity = 0;
let currentPosition = 0;  // stepper position
let limitLeft = false;
let limitRight = false;
let queueSize = 0;  // Arduino command queue size
let previousAngle = 0;
let lastUpdateTime = Date.now();

// Position tracking - will be calibrated based on observed min/max
let minPosition = -5000;  // Will adjust as we see data
let maxPosition = 5000;

// DOM elements
const container = document.getElementById('pendulum-container')!;
const statusEl = document.getElementById('status')!;
const angleValueEl = document.getElementById('angle-value')!;
const velocityValueEl = document.getElementById('velocity-value')!;
const queueValueEl = document.getElementById('queue-value');
const positionValueEl = document.getElementById('position-value');
const positionMarkerEl = document.getElementById('position-marker');
const limitLeftEl = document.getElementById('limit-left');
const limitRightEl = document.getElementById('limit-right');
const loadingSpinnerEl = document.getElementById('loading-spinner');

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
const RAIL_Y = centerY - 30;  // Above the pendulum pivot
const CART_WIDTH = 60;
const CART_HEIGHT = 30;

// Create rail (track the cart moves on)
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

// Cart (rectangle that moves on the rail)
const cart = two.makeRoundedRectangle(centerX, RAIL_Y, CART_WIDTH, CART_HEIGHT, 5);
cart.fill = 'rgba(0, 212, 255, 0.3)';
cart.stroke = 'var(--accent-cyan)';
cart.linewidth = 2;

// Create a group for the pendulum (rod + bob) that we can rotate
const pendulumGroup = two.makeGroup();
pendulumGroup.translation.set(centerX, centerY);

// Rod - from pivot (0,0) to bob (0, PENDULUM_LENGTH) in local coords
const rod = two.makeLine(0, 0, 0, PENDULUM_LENGTH);
rod.stroke = '#4a9eff';
rod.linewidth = 4;
rod.cap = 'round';
pendulumGroup.add(rod);

// Bob glow (behind bob)
const bobGlow = two.makeCircle(0, PENDULUM_LENGTH, BOB_RADIUS + 10);
bobGlow.fill = 'rgba(255, 107, 53, 0.2)';
bobGlow.noStroke();
pendulumGroup.add(bobGlow);

// Bob (pendulum mass)
const bob = two.makeCircle(0, PENDULUM_LENGTH, BOB_RADIUS);
bob.fill = '#ff6b35';
bob.stroke = '#cc5529';
bob.linewidth = 3;
pendulumGroup.add(bob);

// Pivot glow (add to pendulum group so it moves with cart)
const pivotGlow = two.makeCircle(0, 0, PIVOT_RADIUS + 8);
pivotGlow.fill = 'rgba(0, 212, 255, 0.2)';
pivotGlow.noStroke();
pendulumGroup.add(pivotGlow);

// Pivot point (add to pendulum group)
const pivot = two.makeCircle(0, 0, PIVOT_RADIUS);
pivot.fill = '#00d4ff';
pivot.stroke = '#00a8cc';
pivot.linewidth = 2;
pendulumGroup.add(pivot);

// Reference line (shows "down" direction) - add to pendulum group
const arcRadius = 60;
const refLine = two.makeLine(0, 0, 0, arcRadius + 20);
refLine.stroke = 'rgba(255, 255, 255, 0.2)';
refLine.linewidth = 1;
refLine.dashes = [5, 5];
pendulumGroup.add(refLine);

// Angle text (add to pendulum group so it moves with cart)
const angleText = two.makeText('0°', 80, 40);
angleText.fill = '#ff00aa';
angleText.size = 16;
angleText.family = 'JetBrains Mono, monospace';
pendulumGroup.add(angleText);

// Store arc reference for updates
let currentArc: any = null;

// Update position display, marker, and cart visualization
function updatePositionDisplay(position: number, limLeft: boolean, limRight: boolean) {
  // Update value display
  if (positionValueEl) {
    positionValueEl.textContent = position.toString();
  }
  
  // Auto-calibrate min/max based on observed positions
  if (position < minPosition) minPosition = position;
  if (position > maxPosition) maxPosition = position;
  
  // Calculate normalized position (0 = left, 1 = right)
  const range = maxPosition - minPosition;
  let normalizedPos = 0.5;
  if (range > 0) {
    normalizedPos = (position - minPosition) / range;
  }
  
  // Update marker position in UI (0% = left, 100% = right)
  if (positionMarkerEl) {
    const percent = normalizedPos * 100;
    const clampedPercent = Math.max(10, Math.min(90, percent));
    positionMarkerEl.style.left = `${clampedPercent}%`;
  }
  
  // Update limit indicators in UI
  if (limitLeftEl) {
    limitLeftEl.classList.toggle('active', limLeft);
  }
  if (limitRightEl) {
    limitRightEl.classList.toggle('active', limRight);
  }
  
  // Update cart position in Two.js visualization
  const cartX = centerX - RAIL_WIDTH / 2 + RAIL_WIDTH * normalizedPos;
  cart.translation.x = cartX;
  pendulumGroup.translation.x = cartX;  // Move pendulum with cart
  
  // Update limit indicators on rail
  if (limLeft) {
    limitLeftIndicator.fill = '#ff4444';
    minPosition = position;  // Calibrate
  } else {
    limitLeftIndicator.fill = 'rgba(100, 100, 100, 0.5)';
  }
  
  if (limRight) {
    limitRightIndicator.fill = '#ff4444';
    maxPosition = position;  // Calibrate
  } else {
    limitRightIndicator.fill = 'rgba(100, 100, 100, 0.5)';
  }
}

// Update pendulum position based on angle
function updatePendulum(angleDeg: number) {
  // Convert angle: 0° = down, 180° = up
  // Rotation: 0 means pointing down (+Y in screen coords)
  const angleRad = angleDeg * (Math.PI / 180);
  
  // Rotate the entire pendulum group
  pendulumGroup.rotation = angleRad;
  
  // Update angle arc (created at current cart position)
  const startAngle = Math.PI / 2;  // Down direction (in Two.js coords)
  const endAngle = startAngle + angleRad;
  
  // Get current cart X position for arc placement
  const currentCartX = pendulumGroup.translation.x;
  
  // Remove old arc
  if (currentArc) {
    two.remove(currentArc);
    currentArc = null;
  }
  
  // Create new arc at current position
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
  
  // Update angle text (relative to pendulum group origin)
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

function sendCommand(cmd: string) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ command: cmd }));
    console.log('Sent command:', cmd);
  } else {
    console.warn('WebSocket not connected, cannot send:', cmd);
  }
}

// Upload button handling
const uploadBtn = document.getElementById('btn-upload');
const uploadStatus = document.getElementById('upload-status');

function showUploadStatus(message: string, type: 'uploading' | 'success' | 'error') {
  if (uploadStatus) {
    uploadStatus.textContent = message;
    uploadStatus.className = 'visible ' + type;
    
    // Auto-hide success/error after 5 seconds
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
      ws.send(JSON.stringify({ command: 'UPLOAD' }));
    } else {
      showUploadStatus('Not connected to controller', 'error');
    }
  });
}

function connect() {
  statusEl.textContent = 'Connecting...';
  statusEl.classList.remove('connected');
  
  try {
    ws = new WebSocket(WEBSOCKET_URL);
    
    ws.onopen = () => {
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
        
        // Handle upload status messages
        if (data.upload_status !== undefined) {
          const uploadBtn = document.getElementById('btn-upload');
          if (uploadBtn) uploadBtn.classList.remove('uploading');
          
          if (data.upload_status === 'success') {
            showUploadStatus('✓ Upload successful!', 'success');
          } else if (data.upload_status === 'error') {
            showUploadStatus('✗ Upload failed: ' + (data.message || 'Unknown error'), 'error');
          } else if (data.upload_status === 'started') {
            showUploadStatus('Compiling and uploading...', 'uploading');
          }
        }
        
        // Handle angle data
        if (data.angle !== undefined) {
          previousAngle = currentAngle;
          currentAngle = data.angle;
          
          // Use velocity from server if provided
          if (data.velocity !== undefined) {
            currentVelocity = data.velocity;
          } else {
            // Calculate velocity locally
            const now = Date.now();
            const dt = (now - lastUpdateTime) / 1000;
            if (dt > 0) {
              let angleDiff = currentAngle - previousAngle;
              if (angleDiff > 180) angleDiff -= 360;
              if (angleDiff < -180) angleDiff += 360;
              currentVelocity = angleDiff / dt;
            }
          }
          lastUpdateTime = Date.now();
          
          // Update displays
          angleValueEl.textContent = currentAngle.toFixed(1);
          velocityValueEl.textContent = currentVelocity.toFixed(1);
          
          // Update visualization
          updatePendulum(currentAngle);
        }
        
        // Handle position data
        if (data.position !== undefined) {
          currentPosition = data.position;
          limitLeft = data.limitLeft || false;
          limitRight = data.limitRight || false;
          updatePositionDisplay(currentPosition, limitLeft, limitRight);
        }
        
        // Handle queue size
        if (data.queueSize !== undefined) {
          queueSize = data.queueSize;
          if (queueValueEl) {
            queueValueEl.textContent = queueSize.toString();
            // Add visual indicator when queue has items
            queueValueEl.parentElement?.classList.toggle('has-items', queueSize > 0);
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
    
    ws.onerror = () => {
      statusEl.textContent = 'Connection error';
      statusEl.classList.remove('connected');
      container.classList.remove('connected');
      container.classList.add('disconnected');
      if (loadingSpinnerEl) loadingSpinnerEl.classList.remove('hidden');
    };
  } catch (e) {
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
  // Simple click buttons (send command once)
  document.querySelectorAll('.control-btn[data-cmd]').forEach(btn => {
    btn.addEventListener('click', () => {
      const cmd = btn.getAttribute('data-cmd');
      if (cmd) sendCommand(cmd);
    });
  });
  
  // Hold buttons (send start on press, stop on release)
  document.querySelectorAll('.control-btn.hold-btn').forEach(btn => {
    const startCmd = btn.getAttribute('data-cmd-start');
    const stopCmd = btn.getAttribute('data-cmd-stop');
    
    // Mouse events
    btn.addEventListener('mousedown', () => {
      btn.classList.add('active');
      if (startCmd) sendCommand(startCmd);
    });
    
    btn.addEventListener('mouseup', () => {
      btn.classList.remove('active');
      if (stopCmd) sendCommand(stopCmd);
    });
    
    btn.addEventListener('mouseleave', () => {
      if (btn.classList.contains('active')) {
        btn.classList.remove('active');
        if (stopCmd) sendCommand(stopCmd);
      }
    });
    
    // Touch events for mobile
    btn.addEventListener('touchstart', (e) => {
      e.preventDefault();
      btn.classList.add('active');
      if (startCmd) sendCommand(startCmd);
    });
    
    btn.addEventListener('touchend', () => {
      btn.classList.remove('active');
      if (stopCmd) sendCommand(stopCmd);
    });
  });
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
    
    // Send config on change (when user releases slider)
    input.addEventListener('change', () => {
      if (cfgKey) {
        const configCmd = `${cfgKey}:${input.value}`;
        sendCommand(configCmd);
        console.log('Config:', configCmd);
      }
    });
  });
}

// Handle window resize
window.addEventListener('resize', () => {
  two.width = container.clientWidth;
  two.height = container.clientHeight;
});

// No demo mode - show loading spinner when disconnected

// Initial render
updatePendulum(0);

// Setup controls and config sliders
setupControls();
setupConfigSliders();

// Start connection
connect();

console.log('Pendulum Digital Twin initialized');
console.log('Waiting for WebSocket connection at', WEBSOCKET_URL);
