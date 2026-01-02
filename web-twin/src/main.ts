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

// Position tracking - updated from limit switches
let minPosition = -5000;  // Left limit position
let maxPosition = 5000;   // Right limit position

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
  
  // Update limit indicators (positions are updated from state, not here)
  if (limLeft) {
    limitLeftIndicator.fill = '#ff4444';
  } else {
    limitLeftIndicator.fill = 'rgba(100, 100, 100, 0.5)';
  }
  
  if (limRight) {
    limitRightIndicator.fill = '#ff4444';
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

function sendHome() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'HOME' }));
    console.log('Homing requested');
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

// Training progress display
let trainingActive = false;
let selectedTrainingType: 'swing-up' | 'balance' | 'sklearn' | 'evotorch' = 'swing-up';

// Store genome info for both training types
let bestGenomeSwingUp: any = null;
let bestGenomeBalance: any = null;

function sendStartTraining() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    let type = 'START_TRAINING';
    if (selectedTrainingType === 'balance') {
      type = 'START_BALANCE_TRAINING';
    } else if (selectedTrainingType === 'sklearn') {
      type = 'START_SKLEARN_TRAINING';
    } else if (selectedTrainingType === 'evotorch') {
      type = 'START_EVOTORCH_TRAINING';
    }
    ws.send(JSON.stringify({ type }));
  }
}

function sendStopTraining() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'STOP_TRAINING' }));
  }
}

function sendDeleteBest() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'DELETE_BEST', trainer: selectedTrainingType }));
  }
}

function sendNeatConfig() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    const popSize = (document.getElementById('neat-pop-size') as HTMLInputElement)?.value || '100';
    const maxSpeed = (document.getElementById('neat-max-speed') as HTMLInputElement)?.value || '9000';
    const simSteps = (document.getElementById('neat-sim-steps') as HTMLInputElement)?.value || '2000';
    
    const config: any = {
      type: 'NEAT_CONFIG',
      trainer: selectedTrainingType,
      pop_size: parseInt(popSize),
      max_speed: parseInt(maxSpeed),
      sim_steps: parseInt(simSteps)
    };
    
    // Add perturbation parameters for balance trainer
    if (selectedTrainingType === 'balance') {
      const anglePert = (document.getElementById('neat-angle-perturbation') as HTMLInputElement)?.value;
      const velPert = (document.getElementById('neat-velocity-perturbation') as HTMLInputElement)?.value;
      if (anglePert) config.angle_perturbation = parseFloat(anglePert);
      if (velPert) config.velocity_perturbation = parseFloat(velPert);
    }
    
    ws.send(JSON.stringify(config));
  }
}

// Tab switching
function setupTabs() {
  const tabs = document.querySelectorAll('.neat-tab');
  tabs.forEach(tab => {
    tab.addEventListener('click', () => {
      const tabType = tab.getAttribute('data-tab') as 'swing-up' | 'balance' | 'sklearn' | 'evotorch';
      if (tabType) {
        selectedTrainingType = tabType;
        tabs.forEach(t => t.classList.remove('active'));
        tab.classList.add('active');
        
        // Show/hide content sections
        const neatContent = document.getElementById('neat-training-content');
        const sklearnContent = document.getElementById('sklearn-training-content');
        const evotorchContent = document.getElementById('evotorch-training-content');
        if (neatContent) {
          neatContent.style.display = (tabType === 'swing-up' || tabType === 'balance') ? 'block' : 'none';
        }
        if (sklearnContent) {
          sklearnContent.style.display = tabType === 'sklearn' ? 'block' : 'none';
        }
        if (evotorchContent) {
          evotorchContent.style.display = tabType === 'evotorch' ? 'block' : 'none';
          // Request generation history when tab is selected
          if (tabType === 'evotorch' && ws && ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'GET_EVOTORCH_GENERATIONS' }));
          }
        }
        
        // Show/hide balance-specific perturbation config
        const pertConfig = document.getElementById('balance-perturbation-config');
        if (pertConfig) {
          pertConfig.style.display = tabType === 'balance' ? 'block' : 'none';
        }
        
        // Update best genome display for the selected tab (only for NEAT)
        if (tabType !== 'sklearn') {
          const currentGenome = tabType === 'swing-up' ? bestGenomeSwingUp : bestGenomeBalance;
          updateBestGenomeDisplay(currentGenome);
        }
        
        // Request config for this trainer type
        if (ws && ws.readyState === WebSocket.OPEN && tabType !== 'sklearn') {
          ws.send(JSON.stringify({ type: 'GET_NEAT_CONFIG', trainer: tabType }));
        }
      }
    });
  });
}

setupTabs();

function updateBestGenomeDisplay(genome: any) {
  const fitnessEl = document.getElementById('genome-fitness');
  const nodesEl = document.getElementById('genome-nodes');
  const connectionsEl = document.getElementById('genome-connections');
  const savedAtEl = document.getElementById('genome-saved-at');
  
  if (genome) {
    if (container) container.classList.remove('no-genome');
    if (fitnessEl) fitnessEl.textContent = genome.fitness?.toFixed(1) || '-';
    if (nodesEl) nodesEl.textContent = genome.nodes?.toString() || '-';
    if (connectionsEl) connectionsEl.textContent = genome.connections?.toString() || '-';
    if (savedAtEl) savedAtEl.textContent = genome.saved_at || '-';
  } else {
    if (container) container.classList.add('no-genome');
    if (fitnessEl) fitnessEl.textContent = '-';
    if (nodesEl) nodesEl.textContent = '-';
    if (connectionsEl) connectionsEl.textContent = '-';
    if (savedAtEl) savedAtEl.textContent = '-';
  }
}

let neatConfigLoaded = false;
let lastLoadedTrainer: string | null = null;

function updateNeatConfigDisplay(config: any) {
  if (!config) return;
  
  // Allow updating if switching to a different trainer type
  if (neatConfigLoaded && lastLoadedTrainer === selectedTrainingType) {
    return;
  }
  
  const popSizeEl = document.getElementById('neat-pop-size') as HTMLInputElement;
  const maxSpeedEl = document.getElementById('neat-max-speed') as HTMLInputElement;
  const simStepsEl = document.getElementById('neat-sim-steps') as HTMLInputElement;
  
  if (popSizeEl && config.pop_size !== undefined) {
    popSizeEl.value = config.pop_size.toString();
  }
  if (maxSpeedEl && config.max_speed !== undefined) {
    maxSpeedEl.value = config.max_speed.toString();
  }
  if (simStepsEl && config.sim_steps !== undefined) {
    simStepsEl.value = config.sim_steps.toString();
  }
  
  // Update perturbation parameters for balance trainer
  if (selectedTrainingType === 'balance') {
    const anglePertEl = document.getElementById('neat-angle-perturbation') as HTMLInputElement;
    const velPertEl = document.getElementById('neat-velocity-perturbation') as HTMLInputElement;
    
    if (anglePertEl && config.angle_perturbation !== undefined) {
      anglePertEl.value = config.angle_perturbation.toString();
    }
    if (velPertEl && config.velocity_perturbation !== undefined) {
      velPertEl.value = config.velocity_perturbation.toString();
    }
  }
  
  // Track that we've loaded config for this trainer type
  neatConfigLoaded = true;
  lastLoadedTrainer = selectedTrainingType;
}

function updateTrainingDisplay(data: any) {
  // Show training is active
  trainingActive = true;
  const startBtn = document.getElementById('btn-start-training');
  const stopBtn = document.getElementById('btn-stop-training');
  if (startBtn) startBtn.style.display = 'none';
  if (stopBtn) stopBtn.style.display = 'block';
  
  // Update based on training type
  if (data.trainer === 'sklearn' || selectedTrainingType === 'sklearn') {
    updateSklearnTrainingDisplay(data);
    return;
  }
  
  if (data.trainer === 'evotorch' || selectedTrainingType === 'evotorch') {
    updateEvotorchTrainingDisplay(data);
    return;
  }
  
  // Update stats for NEAT
  const genEl = document.getElementById('train-generation');
  const bestEl = document.getElementById('train-best-fitness');
  const avgEl = document.getElementById('train-avg-fitness');
  const speciesEl = document.getElementById('train-species');
  const popEl = document.getElementById('train-population');
  
  if (genEl) genEl.textContent = data.generation?.toString() || '0';
  if (bestEl) bestEl.textContent = data.best_fitness?.toFixed(1) || '0';
  if (avgEl) avgEl.textContent = data.avg_fitness?.toFixed(1) || '0';
  if (speciesEl) speciesEl.textContent = data.species_count?.toString() || '0';
  if (popEl) popEl.textContent = data.population_size?.toString() || '0';
  
  // Update best genome display if new best was saved
  if (data.best_genome) {
    // Store in the correct variable based on selected training type
    if (selectedTrainingType === 'swing-up') {
      bestGenomeSwingUp = data.best_genome;
    } else {
      bestGenomeBalance = data.best_genome;
    }
    updateBestGenomeDisplay(data.best_genome);
  }
  
  // Draw fitness graph
  const canvas = document.getElementById('fitness-canvas') as HTMLCanvasElement;
  if (canvas && data.fitness_history && data.fitness_history.length > 0) {
    const ctx = canvas.getContext('2d');
    if (ctx) {
      const w = canvas.width;
      const h = canvas.height;
      
      // Clear
      ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.fillRect(0, 0, w, h);
      
      // Find min/max for scaling
      const history = data.fitness_history;
      const maxFit = Math.max(...history, 1);
      const minFit = Math.min(...history, 0);
      const range = maxFit - minFit || 1;
      
      // Draw line
      ctx.strokeStyle = '#c864ff';
      ctx.lineWidth = 2;
      ctx.beginPath();
      
      for (let i = 0; i < history.length; i++) {
        const x = (i / (history.length - 1 || 1)) * w;
        const y = h - ((history[i] - minFit) / range) * h * 0.9 - h * 0.05;
        
        if (i === 0) {
          ctx.moveTo(x, y);
        } else {
          ctx.lineTo(x, y);
        }
      }
      ctx.stroke();
      
      // Draw current value
      ctx.fillStyle = '#c864ff';
      ctx.font = '10px monospace';
      ctx.fillText(`${maxFit.toFixed(0)}`, 2, 12);
      ctx.fillText(`${minFit.toFixed(0)}`, 2, h - 2);
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

// Training buttons
const startTrainingBtn = document.getElementById('btn-start-training');
const stopTrainingBtn = document.getElementById('btn-stop-training');

if (startTrainingBtn) {
  startTrainingBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendStartTraining();
      startTrainingBtn.style.display = 'none';
      if (stopTrainingBtn) stopTrainingBtn.style.display = 'block';
    }
  });
}

if (stopTrainingBtn) {
  stopTrainingBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendStopTraining();
      stopTrainingBtn.style.display = 'none';
      if (startTrainingBtn) startTrainingBtn.style.display = 'block';
      trainingActive = false;
    }
  });
}

// Delete best button
const deleteBestBtn = document.getElementById('btn-delete-best');
if (deleteBestBtn) {
  deleteBestBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendDeleteBest();
    }
  });
}

// Apply NEAT config button
const applyNeatConfigBtn = document.getElementById('btn-apply-neat-config');
if (applyNeatConfigBtn) {
  applyNeatConfigBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendNeatConfig();
    }
  });
}

// Apply Sklearn config button
const applySklearnConfigBtn = document.getElementById('btn-apply-sklearn-config');
if (applySklearnConfigBtn) {
  applySklearnConfigBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendSklearnConfig();
    }
  });
}

const applyEvotorchConfigBtn = document.getElementById('btn-apply-evotorch-config');
if (applyEvotorchConfigBtn) {
  applyEvotorchConfigBtn.addEventListener('click', () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      sendEvotorchConfig();
    }
  });
}

const viewPopulationBtn = document.getElementById('btn-view-population');
if (viewPopulationBtn) {
  viewPopulationBtn.addEventListener('click', () => {
    const genInput = document.getElementById('evotorch-view-generation') as HTMLInputElement;
    const generation = parseInt(genInput?.value || '0');
    if (generation > 0 && ws && ws.readyState === WebSocket.OPEN) {
      viewEvotorchPopulation(generation);
    }
  });
}

function updateEvotorchTrainingDisplay(data: any) {
  const genEl = document.getElementById('evotorch-generation');
  const bestEl = document.getElementById('evotorch-best-fitness');
  const currentEl = document.getElementById('evotorch-current-fitness');
  const popEl = document.getElementById('evotorch-population');
  
  if (genEl) genEl.textContent = data.generation?.toString() || '0';
  if (bestEl) bestEl.textContent = data.best_fitness?.toFixed(1) || '0';
  if (currentEl) currentEl.textContent = data.current_fitness?.toFixed(1) || '0';
  if (popEl) popEl.textContent = data.population_size?.toString() || '50';
  
  // Update fitness graph
  const canvas = document.getElementById('evotorch-fitness-canvas') as HTMLCanvasElement;
  if (canvas) {
    const ctx = canvas.getContext('2d');
    if (ctx) {
      const w = canvas.width;
      const h = canvas.height;
      
      // Clear
      ctx.fillStyle = 'rgba(0, 0, 0, 0.3)';
      ctx.fillRect(0, 0, w, h);
      
      // Simple display - just show current and best
      ctx.strokeStyle = '#4a9eff';
      ctx.lineWidth = 2;
      ctx.font = '10px monospace';
      ctx.fillStyle = '#4a9eff';
      ctx.fillText(`Best: ${data.best_fitness?.toFixed(1) || '0'}`, 2, 12);
      ctx.fillText(`Current: ${data.current_fitness?.toFixed(1) || '0'}`, 2, 24);
    }
  }
  
  // Request generation history update
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'GET_EVOTORCH_GENERATIONS' }));
  }
}

function updateEvotorchGenerationList(generations: any) {
  const listEl = document.getElementById('evotorch-generation-list');
  if (!listEl) return;
  
  if (!generations || Object.keys(generations).length === 0) {
    listEl.innerHTML = '<div style="color: #888; font-size: 12px;">No generations yet</div>';
    return;
  }
  
  // Sort by generation number (descending)
  const sorted = Object.values(generations).sort((a: any, b: any) => b.generation - a.generation);
  
  listEl.innerHTML = sorted.map((gen: any) => {
    const date = new Date(gen.timestamp * 1000);
    const dateStr = date.toLocaleTimeString();
    return `
      <div style="display: flex; justify-content: space-between; align-items: center; padding: 4px 0; border-bottom: 1px solid #333; cursor: pointer;" 
           onclick="viewEvotorchGenerationRecordings(${gen.generation})"
           onmouseover="this.style.background='#2a2a2a'" 
           onmouseout="this.style.background='transparent'">
        <div>
          <span style="color: #4a9eff; font-weight: bold;">Gen ${gen.generation}</span>
          <span style="color: #888; margin-left: 8px;">Fitness: ${gen.fitness?.toFixed(1) || '0'}</span>
          <span style="color: #888; margin-left: 8px;">Best: ${gen.best_fitness?.toFixed(1) || '0'}</span>
        </div>
        <div style="color: #666; font-size: 11px;">${dateStr} ▶ View All</div>
      </div>
    `;
  }).join('');
}

function viewEvotorchGenerationRecordings(generation: number) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    // Request all population recordings for this generation
    ws.send(JSON.stringify({ type: 'GET_EVOTORCH_POPULATION_RECORDS', generation }));
  }
}

// State for population visualization
let populationRecordings: any[] = [];
let populationAnimationFrame: number | null = null;
let populationCurrentStep = 0;
let populationMaxSteps = 0;
let isPlayingPopulation = false;
let populationPendulums: any[] = [];

function previewEvotorchGeneration(generation: number) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    // Request all population recordings for this generation
    ws.send(JSON.stringify({ type: 'GET_EVOTORCH_POPULATION_RECORDS', generation }));
  }
}

// Make previewEvotorchGeneration available globally
(window as any).previewEvotorchGeneration = previewEvotorchGeneration;

function viewEvotorchPopulation(generation: number) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'GET_EVOTORCH_POPULATION_RECORDS', generation }));
  }
}

function viewEvotorchPopulationRecording(generation: number, solutionId: number) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: 'GET_EVOTORCH_SOLUTION_RECORDING', generation, solution_id: solutionId }));
  }
}

function updateEvotorchPopulationList(records: any, generation: number) {
  const listEl = document.getElementById('evotorch-population-list');
  if (!listEl) return;
  
  listEl.style.display = 'block';
  
  if (!records || Object.keys(records).length === 0) {
    listEl.innerHTML = '<div style="color: #888; font-size: 12px;">No population records found for this generation</div>';
    return;
  }
  
  // Sort by solution ID
  const sorted = Object.values(records).sort((a: any, b: any) => a.solution_id - b.solution_id);
  
  listEl.innerHTML = `
    <div style="color: #4a9eff; font-weight: bold; margin-bottom: 8px;">Generation ${generation} - Population (${sorted.length} solutions)</div>
    <div style="color: #888; font-size: 12px; margin-bottom: 8px;">Loading all recordings...</div>
  `;
  
  // Request all recordings
  if (ws && ws.readyState === WebSocket.OPEN) {
    sorted.forEach((rec: any) => {
      ws.send(JSON.stringify({ 
        type: 'GET_EVOTORCH_SOLUTION_RECORDING', 
        generation, 
        solution_id: rec.solution_id 
      }));
    });
  }
}

function addPopulationRecording(recording: any, generation: number, solutionId: number) {
  if (!recording || !recording.trajectory) return;
  
  // Store the recording
  populationRecordings.push({
    generation,
    solutionId,
    fitness: recording.fitness,
    trajectory: recording.trajectory
  });
  
  // Update max steps
  populationMaxSteps = Math.max(populationMaxSteps, recording.trajectory.length);
  
  // Once we have all recordings, start visualization
  // Check if we have all recordings by checking the list element
  const listEl = document.getElementById('evotorch-population-list');
  if (listEl) {
    const expectedCount = parseInt(listEl.textContent?.match(/\((\d+) solutions\)/)?.[1] || '0');
    if (populationRecordings.length >= expectedCount && expectedCount > 0) {
      startPopulationVisualization(generation);
    }
  }
}

function startPopulationVisualization(generation: number) {
  // Clear existing pendulums
  clearPopulationPendulums();
  
  // Create a pendulum group for each recording
  populationRecordings.forEach((rec, index) => {
    const group = two.makeGroup();
    group.translation.set(centerX, centerY);
    
    // Rod
    const rod = two.makeLine(0, 0, 0, PENDULUM_LENGTH);
    rod.stroke = `rgba(74, 158, 255, 0.5)`;
    rod.linewidth = 2;
    rod.cap = 'round';
    group.add(rod);
    
    // Bob
    const bob = two.makeCircle(0, PENDULUM_LENGTH, BOB_RADIUS);
    bob.fill = `rgba(74, 158, 255, 0.5)`;
    bob.stroke = `rgba(74, 158, 255, 0.7)`;
    bob.linewidth = 1;
    group.add(bob);
    
    // Cart
    const cart = two.makeRoundedRectangle(0, RAIL_Y - centerY, CART_WIDTH, CART_HEIGHT, 5);
    cart.fill = `rgba(0, 212, 255, 0.3)`;
    cart.stroke = `rgba(0, 212, 255, 0.5)`;
    cart.linewidth = 1;
    group.add(cart);
    
    populationPendulums.push({
      group,
      recording: rec,
      rod,
      bob,
      cart
    });
  });
  
  // Reset animation
  populationCurrentStep = 0;
  isPlayingPopulation = true;
  
  // Update UI
  const listEl = document.getElementById('evotorch-population-list');
  if (listEl) {
    listEl.innerHTML = `
      <div style="color: #4a9eff; font-weight: bold; margin-bottom: 8px;">
        Generation ${generation} - ${populationRecordings.length} solutions
      </div>
      <div style="margin-bottom: 8px;">
        <button id="btn-play-population" style="padding: 4px 8px; margin-right: 4px; background: #4a9eff; border: none; color: white; cursor: pointer;">Play</button>
        <button id="btn-pause-population" style="padding: 4px 8px; margin-right: 4px; background: #666; border: none; color: white; cursor: pointer;">Pause</button>
        <button id="btn-reset-population" style="padding: 4px 8px; background: #666; border: none; color: white; cursor: pointer;">Reset</button>
      </div>
      <div style="color: #888; font-size: 12px;">
        Step: <span id="population-step">0</span> / ${populationMaxSteps}
      </div>
    `;
    
    // Add button listeners
    const playBtn = document.getElementById('btn-play-population');
    const pauseBtn = document.getElementById('btn-pause-population');
    const resetBtn = document.getElementById('btn-reset-population');
    
    if (playBtn) {
      playBtn.addEventListener('click', () => {
        isPlayingPopulation = true;
        animatePopulation();
      });
    }
    if (pauseBtn) {
      pauseBtn.addEventListener('click', () => {
        isPlayingPopulation = false;
        if (populationAnimationFrame) {
          cancelAnimationFrame(populationAnimationFrame);
          populationAnimationFrame = null;
        }
      });
    }
    if (resetBtn) {
      resetBtn.addEventListener('click', () => {
        populationCurrentStep = 0;
        updatePopulationFrame();
      });
    }
  }
  
  // Start animation
  animatePopulation();
}

function clearPopulationPendulums() {
  populationPendulums.forEach(p => {
    two.remove(p.group);
  });
  populationPendulums = [];
}

function animatePopulation() {
  if (!isPlayingPopulation) return;
  
  if (populationCurrentStep < populationMaxSteps) {
    updatePopulationFrame();
    populationCurrentStep++;
    populationAnimationFrame = requestAnimationFrame(() => animatePopulation());
  } else {
    isPlayingPopulation = false;
  }
}

function updatePopulationFrame() {
  const stepEl = document.getElementById('population-step');
  if (stepEl) {
    stepEl.textContent = populationCurrentStep.toString();
  }
  
  populationPendulums.forEach(pendulum => {
    const trajectory = pendulum.recording.trajectory;
    if (populationCurrentStep < trajectory.length) {
      const step = trajectory[populationCurrentStep];
      
      // Update angle
      const angleRad = step.angle * (Math.PI / 180);
      pendulum.group.rotation = angleRad;
      
      // Update cart position
      const railHalf = RAIL_WIDTH / 2;
      const normalizedPos = step.cart_position / 3750; // Approximate rail half
      const cartX = normalizedPos * railHalf;
      pendulum.group.translation.x = centerX + cartX;
    }
  });
  
  two.update();
}

// Make functions available globally
(window as any).viewEvotorchGenerationRecordings = viewEvotorchGenerationRecordings;
(window as any).viewEvotorchPopulationRecording = viewEvotorchPopulationRecording;

function updateSklearnTrainingDisplay(data: any) {
  const episodeEl = document.getElementById('sklearn-episode');
  const bestRewardEl = document.getElementById('sklearn-best-reward');
  const lastRewardEl = document.getElementById('sklearn-last-reward');
  const explorationEl = document.getElementById('sklearn-exploration');
  
  if (episodeEl && data.episode !== undefined) {
    episodeEl.textContent = data.episode.toString();
  }
  if (bestRewardEl && data.best_reward !== undefined) {
    bestRewardEl.textContent = data.best_reward.toFixed(1);
  }
  if (lastRewardEl && data.last_reward !== undefined) {
    lastRewardEl.textContent = data.last_reward.toFixed(1);
  }
  if (explorationEl && data.exploration_rate !== undefined) {
    explorationEl.textContent = data.exploration_rate.toFixed(3);
  }
}

function sendSklearnConfig() {
  if (ws && ws.readyState === WebSocket.OPEN) {
    const episodes = (document.getElementById('sklearn-episodes') as HTMLInputElement)?.value || '1000';
    const maxSpeed = (document.getElementById('sklearn-max-speed') as HTMLInputElement)?.value || '9000';
    const simSteps = (document.getElementById('sklearn-sim-steps') as HTMLInputElement)?.value || '5000';
    
    ws.send(JSON.stringify({
      type: 'SKLEARN_CONFIG',
      episodes: parseInt(episodes),
      max_speed: parseInt(maxSpeed),
      sim_steps: parseInt(simSteps)
    }));
  }
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
          
          // Update config sliders if config data is present
          if (data.config) {
            if (data.config.manual_accel !== undefined) {
              const accelSlider = document.getElementById('cfg-manual-accel') as HTMLInputElement;
              const accelValue = document.getElementById('val-manual-accel');
              if (accelSlider) {
                accelSlider.value = data.config.manual_accel.toString();
                if (accelValue) accelValue.textContent = data.config.manual_accel.toString();
              }
            }
            if (data.config.oscillate_speed !== undefined) {
              const oscSpeedSlider = document.getElementById('cfg-osc-speed') as HTMLInputElement;
              const oscSpeedValue = document.getElementById('val-osc-speed');
              if (oscSpeedSlider) {
                oscSpeedSlider.value = data.config.oscillate_speed.toString();
                if (oscSpeedValue) oscSpeedValue.textContent = data.config.oscillate_speed.toString();
              }
            }
            if (data.config.oscillate_period !== undefined) {
              const oscPeriodSlider = document.getElementById('cfg-osc-period') as HTMLInputElement;
              const oscPeriodValue = document.getElementById('val-osc-period');
              if (oscPeriodSlider) {
                oscPeriodSlider.value = data.config.oscillate_period.toString();
                if (oscPeriodValue) oscPeriodValue.textContent = data.config.oscillate_period.toFixed(1);
              }
            }
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
          
          // Update limit positions and use them for visualization
          if (data.limit_left_pos !== undefined) {
            const el = document.getElementById('limit-left-pos');
            if (el) el.textContent = data.limit_left_pos.toString();
            if (data.limit_left_pos !== 0) {
              minPosition = data.limit_left_pos;
            }
          }
          if (data.limit_right_pos !== undefined) {
            const el = document.getElementById('limit-right-pos');
            if (el) el.textContent = data.limit_right_pos.toString();
            if (data.limit_right_pos !== 0) {
              maxPosition = data.limit_right_pos;
            }
          }
          
          // Store genome info for both training types
          if (data.best_genome_swing_up !== undefined) {
            bestGenomeSwingUp = data.best_genome_swing_up;
          }
          if (data.best_genome_balance !== undefined) {
            bestGenomeBalance = data.best_genome_balance;
          }
          // Display the genome for the selected tab
          const currentGenome = selectedTrainingType === 'swing-up' ? bestGenomeSwingUp : bestGenomeBalance;
          updateBestGenomeDisplay(currentGenome);
          
          // Update NEAT config inputs (once on first connection)
          updateNeatConfigDisplay(data.neat_config);
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
        
        if (data.type === 'DELETE_RESULT') {
          if (data.success) {
            // Clear the genome for the selected tab
            if (selectedTrainingType === 'swing-up') {
              bestGenomeSwingUp = null;
            } else {
              bestGenomeBalance = null;
            }
            updateBestGenomeDisplay(null);
          }
        }
        
        // Handle training progress
        if (data.type === 'TRAINING') {
          updateTrainingDisplay(data);
        }
        
        if (data.type === 'TRAINING_STARTED') {
          console.log('Training started:', data.trainer || 'unknown');
          trainingActive = true;
          const startBtn = document.getElementById('btn-start-training');
          const stopBtn = document.getElementById('btn-stop-training');
          if (startBtn) startBtn.style.display = 'none';
          if (stopBtn) stopBtn.style.display = 'block';
        }
        
        if (data.type === 'TRAINING_STOPPED') {
          console.log('Training stopped');
          trainingActive = false;
          const startBtn = document.getElementById('btn-start-training');
          const stopBtn = document.getElementById('btn-stop-training');
          if (startBtn) startBtn.style.display = 'block';
          if (stopBtn) stopBtn.style.display = 'none';
        }
        
        if (data.type === 'EVOTORCH_GENERATIONS') {
          updateEvotorchGenerationList(data.generations);
        }
        
        if (data.type === 'PREVIEW_RESULT') {
          if (data.success) {
            showUploadStatus(`Previewing generation ${data.generation} (Fitness: ${data.fitness?.toFixed(1) || '0'})`, 'success');
            // Switch to evotorch_balance mode
            sendMode('evotorch_balance');
          } else {
            showUploadStatus(`Preview failed: ${data.message}`, 'error');
          }
        }
        
        if (data.type === 'EVOTORCH_POPULATION_RECORDS') {
          // Reset recordings when starting a new generation
          populationRecordings = [];
          populationMaxSteps = 0;
          updateEvotorchPopulationList(data.records, data.generation);
        }
        
        if (data.type === 'EVOTORCH_SOLUTION_RECORDING') {
          if (data.recording) {
            addPopulationRecording(data.recording, data.generation, data.solution_id);
          }
        }
        
        if (data.type === 'NEAT_CONFIG_DATA') {
          // Update config inputs with data for selected trainer
          const popSizeEl = document.getElementById('neat-pop-size') as HTMLInputElement;
          const maxSpeedEl = document.getElementById('neat-max-speed') as HTMLInputElement;
          const simStepsEl = document.getElementById('neat-sim-steps') as HTMLInputElement;
          
          if (popSizeEl && data.pop_size) popSizeEl.value = data.pop_size.toString();
          if (maxSpeedEl && data.max_speed) maxSpeedEl.value = data.max_speed.toString();
          if (simStepsEl && data.sim_steps) simStepsEl.value = data.sim_steps.toString();
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
      neatConfigLoaded = false;  // Reset so config reloads on reconnect
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
  // Acceleration-only control (always uses acceleration)
  
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
  
  // Home button
  const homeBtn = document.getElementById('btn-home');
  if (homeBtn) {
    homeBtn.addEventListener('click', () => {
      sendHome();
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
