import { useState, useEffect, useRef } from 'react'
import { useAtomValue, useSetAtom } from 'jotai'
import { getDefaultStore } from 'jotai'
import { pendulumStateAtom, controlConfigAtom, websocketAtom, manualAccelAtom, oscillateSpeedAtom, oscillatePeriodAtom, lqrQAngleAtom, lqrQAngleVelAtom, lqrQPositionAtom, lqrQVelocityAtom, lqrRControlAtom } from '../store/atoms'
import { useSendCommand } from '../store/hooks'

export default function InfoPanel() {
  const state = useAtomValue(pendulumStateAtom)
  const config = useAtomValue(controlConfigAtom)
  const ws = useAtomValue(websocketAtom)
  const sendCommand = useSendCommand()
  const setManualAccel = useSetAtom(manualAccelAtom)
  const setOscillateSpeed = useSetAtom(oscillateSpeedAtom)
  const setOscillatePeriod = useSetAtom(oscillatePeriodAtom)
  const setLqrQAngle = useSetAtom(lqrQAngleAtom)
  const setLqrQAngleVel = useSetAtom(lqrQAngleVelAtom)
  const setLqrQPosition = useSetAtom(lqrQPositionAtom)
  const setLqrQVelocity = useSetAtom(lqrQVelocityAtom)
  const setLqrRControl = useSetAtom(lqrRControlAtom)
  const [uploadStatus, setUploadStatus] = useState<{ message: string; type: 'uploading' | 'success' | 'error' } | null>(null)
  const positionMarkerRef = useRef<HTMLDivElement>(null)
  const minPosition = state.limitLeftPos || -5000
  const maxPosition = state.limitRightPos || 5000

  useEffect(() => {
    if (!ws) return

    const handleMessage = (event: MessageEvent) => {
      const data = JSON.parse(event.data)
      
      if (data.type === 'UPLOAD_STATUS') {
        setUploadStatus({ message: data.message, type: 'uploading' })
      } else if (data.type === 'UPLOAD_RESULT') {
        setUploadStatus({
          message: data.success ? '✓ Upload successful!' : `✗ ${data.message || 'Upload failed'}`,
          type: data.success ? 'success' : 'error'
        })
        setTimeout(() => setUploadStatus(null), 5000)
      }
    }

    ws.addEventListener('message', handleMessage)
    return () => ws.removeEventListener('message', handleMessage)
  }, [ws])

  useEffect(() => {
    if (!positionMarkerRef.current) return
    
    const range = maxPosition - minPosition
    let normalizedPos = 0.5
    if (range > 0) {
      normalizedPos = (state.position - minPosition) / range
    }
    const percent = normalizedPos * 100
    const clampedPercent = Math.max(10, Math.min(90, percent))
    positionMarkerRef.current.style.left = `${clampedPercent}%`
  }, [state.position, minPosition, maxPosition])

  const handleModeChange = (mode: string) => {
    sendCommand({ type: 'MODE', mode })
  }

  const handleStop = () => {
    sendCommand({ type: 'STOP' })
  }

  const handleZero = () => {
    sendCommand({ type: 'ZERO' })
  }

  const handleHome = () => {
    sendCommand({ type: 'HOME' })
  }

  const handleUpload = () => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      setUploadStatus({ message: 'Uploading... please wait', type: 'uploading' })
      sendCommand({ type: 'UPLOAD' })
    } else {
      setUploadStatus({ message: 'Not connected to controller', type: 'error' })
    }
  }

  const handleConfigChange = (key: 'manualAccel' | 'oscillateSpeed' | 'oscillatePeriod' | 'lqrQAngle' | 'lqrQAngleVel' | 'lqrQPosition' | 'lqrQVelocity' | 'lqrRControl', value: number) => {
    if (key === 'manualAccel') {
      setManualAccel(value)
      sendCommand({ type: 'CONFIG', manual_accel: value })
    } else if (key === 'oscillateSpeed') {
      setOscillateSpeed(value)
      sendCommand({ type: 'CONFIG', oscillate_speed: value })
    } else if (key === 'oscillatePeriod') {
      setOscillatePeriod(value)
      sendCommand({ type: 'CONFIG', oscillate_period: value })
    } else if (key === 'lqrQAngle') {
      setLqrQAngle(value)
      sendCommand({ type: 'CONFIG', lqr_q_angle: value })
    } else if (key === 'lqrQAngleVel') {
      setLqrQAngleVel(value)
      sendCommand({ type: 'CONFIG', lqr_q_angle_vel: value })
    } else if (key === 'lqrQPosition') {
      setLqrQPosition(value)
      sendCommand({ type: 'CONFIG', lqr_q_position: value })
    } else if (key === 'lqrQVelocity') {
      setLqrQVelocity(value)
      sendCommand({ type: 'CONFIG', lqr_q_velocity: value })
    } else if (key === 'lqrRControl') {
      setLqrRControl(value)
      sendCommand({ type: 'CONFIG', lqr_r_control: value })
    }
  }

  return (
    <div className="w-[340px] bg-bg-panel border-l border-white/10 p-3 flex flex-col gap-2.5 overflow-y-auto">
      <h1 className="text-2xl font-semibold bg-gradient-to-br from-accent-cyan to-accent-magenta bg-clip-text text-transparent">
        Pendulum Digital Twin
      </h1>

      <div className={`font-mono text-xs px-2.5 py-1.5 rounded-md ${
        state.connected
          ? 'bg-accent-cyan/10 border border-accent-cyan/30 text-accent-cyan'
          : 'bg-accent-orange/10 border border-accent-orange/30 text-accent-orange'
      }`}>
        {state.connected ? '● Connected' : 'Connecting...'}
      </div>

      <div className="font-mono text-sm px-2.5 py-1.5 rounded-md bg-white/3 border border-white/10 text-text-dim">
        Mode: <span className={`font-bold uppercase ${state.mode !== 'idle' ? 'text-accent-green' : 'text-text-dim'}`}>
          {state.mode}
        </span>
      </div>

      <div className="flex flex-col gap-1">
        <div className="flex justify-between items-center font-mono px-3 py-0.5 bg-white/2 rounded-md border border-white/5">
          <span className="text-xs text-text-dim block mb-1">Angle</span>
          <span className="text-2xl font-bold text-accent-cyan">{state.angle.toFixed(1)}°</span>
        </div>
        <div className="flex justify-between items-center font-mono px-3 py-0.5 bg-white/2 rounded-md border border-white/5">
          <span className="text-xs text-text-dim block mb-1">Velocity</span>
          <span className="text-2xl font-bold text-accent-cyan">{state.angularVelocity.toFixed(1)}°/s</span>
        </div>
        <div className="flex justify-between items-center font-mono px-3 py-0.5 bg-white/2 rounded-md border border-white/5">
          <span className="text-xs text-text-dim block mb-1">Motor</span>
          <span className="text-2xl font-bold text-accent-cyan">{state.velocity} st/s</span>
        </div>
      </div>

      <div className="p-2.5 bg-white/2 rounded-lg border border-white/5">
        <div className="flex justify-between items-center mb-1">
          <span className="text-xs text-text-dim">Cart Position</span>
          <span className="font-mono text-lg font-bold text-accent-cyan">{state.position}</span>
        </div>
        <div className="relative h-6 bg-white/5 rounded-xl my-2 overflow-visible">
          <div
            className={`absolute top-1/2 -translate-y-1/2 w-2 h-8 rounded bg-gray-500/50 transition-colors ${
              state.limitLeft ? 'bg-red-500 shadow-[0_0_10px_#ff4444]' : ''
            }`}
            style={{ left: '4px' }}
          />
          <div
            ref={positionMarkerRef}
            className="absolute top-1/2 -translate-y-1/2 w-4 h-4 bg-accent-cyan rounded-full shadow-[0_0_10px_#00d4ff] transition-all duration-100"
          />
          <div
            className={`absolute top-1/2 -translate-y-1/2 w-2 h-8 rounded bg-gray-500/50 transition-colors ${
              state.limitRight ? 'bg-red-500 shadow-[0_0_10px_#ff4444]' : ''
            }`}
            style={{ right: '4px' }}
          />
        </div>
        <div className="flex justify-between text-xs text-text-dim">
          <span>← Left Limit</span>
          <span>Right Limit →</span>
        </div>
        <div className="flex justify-between font-mono text-xs text-accent-cyan mt-1">
          <span>{state.limitLeftPos}</span>
          <span>{state.limitRightPos}</span>
        </div>
      </div>

      <div className="flex flex-col gap-1.5">
        <h2 className="text-xs font-semibold text-text-dim uppercase tracking-wide mb-1.5">Controls</h2>
        
        <div className="flex gap-1.5">
          <button
            onMouseDown={() => handleModeChange('manual_left')}
            onMouseUp={() => handleModeChange('idle')}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-accent-cyan/10 border border-accent-cyan/30 text-accent-cyan hover:bg-accent-cyan/15 transition-all hover:-translate-y-0.5 active:translate-y-0"
          >
            ← Left
          </button>
          <button
            onMouseDown={() => handleModeChange('manual_right')}
            onMouseUp={() => handleModeChange('idle')}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-accent-cyan/10 border border-accent-cyan/30 text-accent-cyan hover:bg-accent-cyan/15 transition-all hover:-translate-y-0.5 active:translate-y-0"
          >
            Right →
          </button>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={() => handleModeChange('oscillate')}
            className={`flex-1 px-3 py-2 text-sm font-semibold rounded-lg transition-all ${
              state.mode === 'oscillate'
                ? 'bg-accent-green/20 border border-accent-green text-accent-green shadow-[0_0_15px_rgba(0,255,136,0.3)]'
                : 'bg-white/8 border border-white/15 text-text-primary hover:bg-white/12 hover:-translate-y-0.5'
            }`}
          >
            ↕ Oscillate
          </button>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={() => handleModeChange('evotorch_balance')}
            className={`flex-1 px-3 py-2 text-sm font-semibold rounded-lg transition-all ${
              state.mode === 'evotorch_balance'
                ? 'bg-neat-purple/40 border border-neat-purple text-neat-purple shadow-[0_0_15px_rgba(200,100,255,0.5)]'
                : 'bg-neat-purple/15 border border-neat-purple/40 text-neat-purple hover:bg-neat-purple/25 hover:-translate-y-0.5'
            }`}
          >
            🧬 EvoTorch
          </button>
          <button
            onClick={() => handleModeChange('lqr_balance')}
            className={`flex-1 px-3 py-2 text-sm font-semibold rounded-lg transition-all ${
              state.mode === 'lqr_balance'
                ? 'bg-blue-500/40 border border-blue-500 text-blue-300 shadow-[0_0_15px_rgba(59,130,246,0.5)]'
                : 'bg-blue-500/15 border border-blue-500/40 text-blue-400 hover:bg-blue-500/25 hover:-translate-y-0.5'
            }`}
          >
            📐 LQR
          </button>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={handleStop}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-red-500/15 border border-red-500/40 text-red-400 hover:bg-red-500/25 transition-all hover:-translate-y-0.5"
          >
            ⏹ Stop
          </button>
          <button
            onClick={handleZero}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-white/5 border border-white/20 text-text-dim hover:bg-white/10 hover:text-text-primary transition-all hover:-translate-y-0.5"
          >
            🎯 Zero
          </button>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={handleHome}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-yellow-500/15 border border-yellow-500/40 text-yellow-400 hover:bg-yellow-500/25 transition-all hover:-translate-y-0.5"
          >
            🏠 Home
          </button>
        </div>

        <div className="flex gap-1.5">
          <button
            onClick={handleUpload}
            className="flex-1 px-3 py-2 text-sm font-semibold rounded-lg bg-accent-green/12 border border-accent-green/40 text-accent-green hover:bg-accent-green/20 transition-all hover:-translate-y-0.5 mt-2"
          >
            📤 Upload Arduino Code
          </button>
        </div>

        {uploadStatus && (
          <div className={`font-mono text-xs px-3 py-2 rounded-md mt-2 ${
            uploadStatus.type === 'success'
              ? 'bg-accent-green/10 border border-accent-green/30 text-accent-green'
              : uploadStatus.type === 'error'
              ? 'bg-red-500/10 border border-red-500/30 text-red-400'
              : 'bg-accent-cyan/10 border border-accent-cyan/30 text-accent-cyan'
          }`}>
            {uploadStatus.message}
          </div>
        )}
      </div>

      <div className="mt-2.5">
        <h2 className="text-xs font-semibold text-text-dim uppercase tracking-wide mb-1.5">Configuration</h2>
        
        <div className="mb-2 p-2.5 bg-white/2 rounded-lg border border-white/5">
          <h3 className="text-xs font-semibold text-accent-cyan uppercase tracking-wide mb-1.5">Manual Mode</h3>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Acceleration</label>
            <input
              type="range"
              min="1000"
              max="50000"
              step="1000"
              value={config.manualAccel}
              onChange={(e) => handleConfigChange('manualAccel', parseInt(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-accent-cyan"
            />
            <span className="font-mono text-xs text-accent-cyan w-[50px] text-right flex-shrink-0">
              {config.manualAccel}
            </span>
          </div>
        </div>

        <div className="p-2.5 bg-white/2 rounded-lg border border-white/5">
          <h3 className="text-xs font-semibold text-accent-cyan uppercase tracking-wide mb-1.5">Oscillate Mode</h3>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Speed</label>
            <input
              type="range"
              min="500"
              max="20000"
              value={config.oscillateSpeed}
              onChange={(e) => handleConfigChange('oscillateSpeed', parseInt(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-accent-cyan"
            />
            <span className="font-mono text-xs text-accent-cyan w-[50px] text-right flex-shrink-0">
              {config.oscillateSpeed}
            </span>
          </div>
          <div className="flex items-center gap-2">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Period (s)</label>
            <input
              type="range"
              min="0.5"
              max="5"
              step="0.1"
              value={config.oscillatePeriod}
              onChange={(e) => handleConfigChange('oscillatePeriod', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-accent-cyan"
            />
            <span className="font-mono text-xs text-accent-cyan w-[50px] text-right flex-shrink-0">
              {config.oscillatePeriod.toFixed(1)}
            </span>
          </div>
        </div>

        <div className="p-2.5 bg-white/2 rounded-lg border border-white/5">
          <h3 className="text-xs font-semibold text-blue-400 uppercase tracking-wide mb-1.5">LQR Mode</h3>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Q Angle</label>
            <input
              type="range"
              min="1"
              max="100"
              step="1"
              value={config.lqrQAngle}
              onChange={(e) => handleConfigChange('lqrQAngle', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-blue-400"
            />
            <span className="font-mono text-xs text-blue-400 w-[50px] text-right flex-shrink-0">
              {config.lqrQAngle.toFixed(1)}
            </span>
          </div>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Q Ang Vel</label>
            <input
              type="range"
              min="0.1"
              max="10"
              step="0.1"
              value={config.lqrQAngleVel}
              onChange={(e) => handleConfigChange('lqrQAngleVel', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-blue-400"
            />
            <span className="font-mono text-xs text-blue-400 w-[50px] text-right flex-shrink-0">
              {config.lqrQAngleVel.toFixed(1)}
            </span>
          </div>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Q Position</label>
            <input
              type="range"
              min="1"
              max="200"
              step="1"
              value={config.lqrQPosition}
              onChange={(e) => handleConfigChange('lqrQPosition', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-blue-400"
            />
            <span className="font-mono text-xs text-blue-400 w-[50px] text-right flex-shrink-0">
              {config.lqrQPosition.toFixed(1)}
            </span>
          </div>
          <div className="flex items-center gap-2 mb-1">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">Q Velocity</label>
            <input
              type="range"
              min="0.1"
              max="5"
              step="0.1"
              value={config.lqrQVelocity}
              onChange={(e) => handleConfigChange('lqrQVelocity', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-blue-400"
            />
            <span className="font-mono text-xs text-blue-400 w-[50px] text-right flex-shrink-0">
              {config.lqrQVelocity.toFixed(1)}
            </span>
          </div>
          <div className="flex items-center gap-2">
            <label className="text-xs text-text-dim w-[55px] flex-shrink-0">R Control</label>
            <input
              type="range"
              min="0.5"
              max="20"
              step="0.5"
              value={config.lqrRControl}
              onChange={(e) => handleConfigChange('lqrRControl', parseFloat(e.target.value))}
              className="flex-1 h-1.5 bg-white/10 rounded appearance-none cursor-pointer accent-blue-400"
            />
            <span className="font-mono text-xs text-blue-400 w-[50px] text-right flex-shrink-0">
              {config.lqrRControl.toFixed(1)}
            </span>
          </div>
        </div>
      </div>
    </div>
  )
}

