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
let previousAngle = 0;
let lastUpdateTime = Date.now();

// DOM elements
const container = document.getElementById('pendulum-container')!;
const statusEl = document.getElementById('status')!;
const angleValueEl = document.getElementById('angle-value')!;
const velocityValueEl = document.getElementById('velocity-value')!;

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

// Pivot glow (behind pivot, but in front of rod)
const pivotGlow = two.makeCircle(centerX, centerY, PIVOT_RADIUS + 8);
pivotGlow.fill = 'rgba(0, 212, 255, 0.2)';
pivotGlow.noStroke();

// Pivot point (on top)
const pivot = two.makeCircle(centerX, centerY, PIVOT_RADIUS);
pivot.fill = '#00d4ff';
pivot.stroke = '#00a8cc';
pivot.linewidth = 2;

// Reference line (shows "down" direction)
const arcRadius = 60;
const refLine = two.makeLine(centerX, centerY, centerX, centerY + arcRadius + 20);
refLine.stroke = 'rgba(255, 255, 255, 0.2)';
refLine.linewidth = 1;
refLine.dashes = [5, 5];

// Angle text
const angleText = two.makeText('0°', centerX + 80, centerY + 40);
angleText.fill = '#ff00aa';
angleText.size = 16;
angleText.family = 'JetBrains Mono, monospace';

// Store arc reference for updates
let currentArc: Two.ArcSegment | null = null;

// Update pendulum position based on angle
function updatePendulum(angleDeg: number) {
  // Convert angle: 0° = down, 180° = up
  // Rotation: 0 means pointing down (+Y in screen coords)
  const angleRad = angleDeg * (Math.PI / 180);
  
  // Rotate the entire pendulum group
  pendulumGroup.rotation = angleRad;
  
  // Update angle arc
  const startAngle = Math.PI / 2;  // Down direction (in Two.js coords)
  const endAngle = startAngle + angleRad;
  
  // Remove old arc
  if (currentArc) {
    two.remove(currentArc);
  }
  
  // Create new arc
  if (Math.abs(angleDeg) > 0.5) {
    currentArc = two.makeArcSegment(
      centerX, centerY,
      arcRadius - 5, arcRadius + 5,
      Math.min(startAngle, endAngle),
      Math.max(startAngle, endAngle)
    );
    currentArc.fill = 'rgba(255, 0, 170, 0.3)';
    currentArc.stroke = '#ff00aa';
    currentArc.linewidth = 2;
  }
  
  // Update angle text position
  const textAngle = startAngle + angleRad / 2;
  angleText.translation.set(
    centerX + (arcRadius + 35) * Math.cos(textAngle),
    centerY + (arcRadius + 35) * Math.sin(textAngle)
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

function connect() {
  statusEl.textContent = 'Connecting...';
  statusEl.classList.remove('connected');
  
  try {
    ws = new WebSocket(WEBSOCKET_URL);
    
    ws.onopen = () => {
      statusEl.textContent = '● Connected';
      statusEl.classList.add('connected');
      console.log('Connected to pendulum controller');
    };
    
    ws.onmessage = (event) => {
      try {
        const data = JSON.parse(event.data);
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
      } catch (e) {
        console.error('Failed to parse message:', e);
      }
    };
    
    ws.onclose = () => {
      statusEl.textContent = 'Disconnected - Reconnecting...';
      statusEl.classList.remove('connected');
      scheduleReconnect();
    };
    
    ws.onerror = () => {
      statusEl.textContent = 'Connection error';
      statusEl.classList.remove('connected');
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

// Handle window resize
window.addEventListener('resize', () => {
  two.width = container.clientWidth;
  two.height = container.clientHeight;
});

// Demo mode - oscillate pendulum if not connected
let demoAngle = 0;
let demoVelocity = 50;
two.bind('update', () => {
  // If not connected, run demo animation
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    demoAngle += demoVelocity * (1/60);
    
    // Simulate pendulum physics (simple harmonic motion)
    const gravity = 200;
    const length = 1;
    const angleRad = demoAngle * (Math.PI / 180);
    demoVelocity -= (gravity / length) * Math.sin(angleRad) * (1/60);
    demoVelocity *= 0.999;  // Damping
    
    currentAngle = demoAngle;
    currentVelocity = demoVelocity;
    
    angleValueEl.textContent = currentAngle.toFixed(1);
    velocityValueEl.textContent = currentVelocity.toFixed(1);
    updatePendulum(currentAngle);
  }
});

// Initial render
updatePendulum(0);

// Setup controls
setupControls();

// Start connection
connect();

console.log('Pendulum Digital Twin initialized');
console.log('Waiting for WebSocket connection at', WEBSOCKET_URL);
