import { useEffect, useRef } from 'react'
import { useAtomValue } from 'jotai'
import { getDefaultStore } from 'jotai'
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Legend, ResponsiveContainer } from 'recharts'
import {
  trainingActiveAtom,
  trainingStatsAtom,
  fitnessHistoryAtom,
  generationsAtom,
  populationRecordsAtom,
  viewingGenerationAtom,
  evotorchConfigAtom,
  websocketAtom,
} from '../store/atoms'
import { useSendCommand } from '../store/hooks'
import {
  setTrainingActive,
  updateTrainingStats,
  setGenerations,
  setPopulationRecords,
  setViewingGeneration,
} from '../store/actions'

export default function TrainingPanel() {
  const trainingActive = useAtomValue(trainingActiveAtom)
  const stats = useAtomValue(trainingStatsAtom)
  const fitnessHistory = useAtomValue(fitnessHistoryAtom)
  const generations = useAtomValue(generationsAtom)
  const populationRecords = useAtomValue(populationRecordsAtom)
  const viewingGeneration = useAtomValue(viewingGenerationAtom)
  const evotorchConfig = useAtomValue(evotorchConfigAtom)
  const ws = useAtomValue(websocketAtom)
  const sendCommand = useSendCommand()
  
  const store = getDefaultStore()
  const viewGenerationInputRef = useRef<HTMLInputElement>(null)

  useEffect(() => {
    if (!ws) return

    const handleMessage = (event: MessageEvent) => {
      const data = JSON.parse(event.data)
      
      if (data.type === 'TRAINING') {
        updateTrainingStats(data, store.set)
        // Update fitness history
        const currentHistory = store.get(fitnessHistoryAtom)
        const newEntry = {
          generation: data.generation || 0,
          best: data.best_fitness || 0,
          current: data.current_fitness || 0,
        }
        // Add new entry or update if generation already exists
        const existingIndex = currentHistory.findIndex((e: any) => e.generation === newEntry.generation)
        if (existingIndex >= 0) {
          const updated = [...currentHistory]
          updated[existingIndex] = newEntry
          store.set(fitnessHistoryAtom, updated)
        } else {
          store.set(fitnessHistoryAtom, [...currentHistory, newEntry])
        }
      } else if (data.type === 'TRAINING_STARTED') {
        setTrainingActive(true, store.set)
      } else if (data.type === 'TRAINING_STOPPED') {
        setTrainingActive(false, store.set)
      } else if (data.type === 'EVOTORCH_GENERATIONS') {
        setGenerations(data.generations || {}, store.set)
      } else if (data.type === 'EVOTORCH_POPULATION_RECORDS') {
        setPopulationRecords([], store.set)
        setViewingGeneration(data.generation, store.set)
        // Request individual recordings
        if (data.records && Object.keys(data.records).length > 0) {
          Object.values(data.records).forEach((rec: any) => {
            sendCommand({
              type: 'GET_EVOTORCH_SOLUTION_RECORDING',
              generation: data.generation,
              solution_id: rec.solution_id
            })
          })
        }
      } else if (data.type === 'EVOTORCH_SOLUTION_RECORDING') {
        if (data.recording) {
          const current = store.get(populationRecordsAtom)
          store.set(populationRecordsAtom, [...current, {
            generation: data.generation,
            solution_id: data.solution_id,
            fitness: data.recording.fitness,
            trajectory: data.recording.trajectory
          }])
        }
      }
    }

    ws.addEventListener('message', handleMessage)
    
    // Request generations on mount
    if (ws.readyState === WebSocket.OPEN) {
      sendCommand({ type: 'GET_EVOTORCH_GENERATIONS' })
    }

    return () => {
      ws.removeEventListener('message', handleMessage)
    }
  }, [ws, sendCommand, store])

  const handleStartTraining = () => {
    sendCommand({ type: 'START_EVOTORCH_TRAINING' })
  }

  const handleStopTraining = () => {
    sendCommand({ type: 'STOP_TRAINING' })
  }

  const handleApplyConfig = () => {
    sendCommand({
      type: 'EVOTORCH_CONFIG',
      generations: evotorchConfig.generations,
      population_size: evotorchConfig.populationSize,
      max_speed: evotorchConfig.maxSpeed,
      sim_steps: evotorchConfig.simSteps,
    })
  }

  const handleViewGeneration = (generation: number) => {
    setPopulationRecords([], store.set)
    setViewingGeneration(generation, store.set)
    sendCommand({ type: 'GET_EVOTORCH_POPULATION_RECORDS', generation })
  }

  const handleViewPopulationInput = () => {
    const gen = parseInt(viewGenerationInputRef.current?.value || '0')
    if (gen > 0) {
      handleViewGeneration(gen)
    }
  }

  const sortedGenerations = Object.values(generations).sort((a: any, b: any) => b.generation - a.generation)

  return (
    <div className="w-[280px] bg-bg-panel border-r border-purple-500/20 p-3 flex flex-col gap-2 overflow-y-auto">
      <h2 className="text-base text-neat-purple mb-1">🧬 Training</h2>
      
      <div className="flex gap-1 mb-2">
        <button className="flex-1 px-2 py-1.5 border border-purple-500/30 bg-purple-500/10 text-purple-500/70 rounded text-xs uppercase tracking-wide">
          🧬 EvoTorch
        </button>
      </div>

      <div className="flex gap-1.5">
        <button
          onClick={handleStartTraining}
          disabled={trainingActive}
          className={`flex-1 px-2.5 py-1.5 text-xs rounded transition-all ${
            trainingActive
              ? 'hidden'
              : 'bg-purple-500/15 border border-purple-500/40 text-neat-purple hover:bg-purple-500/25'
          }`}
        >
          ▶ Start
        </button>
        <button
          onClick={handleStopTraining}
          disabled={!trainingActive}
          className={`flex-1 px-2.5 py-1.5 text-xs rounded transition-all ${
            !trainingActive
              ? 'hidden'
              : 'bg-red-500/15 border border-red-500/40 text-red-400 hover:bg-red-500/25'
          }`}
        >
          ⏹ Stop
        </button>
      </div>

      <div>
        <div className="bg-purple-500/8 border border-purple-500/20 rounded-md p-2 mb-2">
          <div className="flex justify-between text-xs py-0.5">
            <span className="text-white/50">Generation:</span>
            <span className="text-neat-purple font-bold font-mono">{stats.generation || 0}</span>
          </div>
          <div className="flex justify-between text-xs py-0.5">
            <span className="text-white/50">Best Fitness:</span>
            <span className="text-neat-purple font-bold font-mono">{stats.best_fitness?.toFixed(1) || '0'}</span>
          </div>
          <div className="flex justify-between text-xs py-0.5">
            <span className="text-white/50">Current Fitness:</span>
            <span className="text-neat-purple font-bold font-mono">{stats.current_fitness?.toFixed(1) || '0'}</span>
          </div>
          <div className="flex justify-between text-xs py-0.5">
            <span className="text-white/50">Population:</span>
            <span className="text-neat-purple font-bold font-mono">{stats.population_size || 50}</span>
          </div>
        </div>

        <div className="bg-black/30 rounded p-2 mb-2">
          <div className="w-full" style={{ height: '120px' }}>
            <ResponsiveContainer width="100%" height="100%">
              <LineChart 
                data={fitnessHistory.length > 0 ? fitnessHistory : [{ generation: 0, best: 0, current: 0 }]}
                margin={{ top: 5, right: 5, bottom: 5, left: 5 }}
                syncId="fitness-chart"
              >
                <CartesianGrid strokeDasharray="3 3" stroke="rgba(255, 255, 255, 0.1)" />
                <XAxis 
                  dataKey="generation" 
                  stroke="#4a9eff"
                  style={{ fontSize: '10px' }}
                  tick={{ fill: '#4a9eff' }}
                />
                <YAxis 
                  stroke="#4a9eff"
                  style={{ fontSize: '10px' }}
                  tick={{ fill: '#4a9eff' }}
                  width={40}
                />
                <Legend 
                  wrapperStyle={{ fontSize: '10px', paddingTop: '10px' }}
                  iconType="line"
                />
                <Line 
                  type="monotone" 
                  dataKey="best" 
                  stroke="#4a9eff" 
                  strokeWidth={2}
                  dot={false}
                  activeDot={false}
                  isAnimationActive={false}
                  name="Best Fitness"
                />
                <Line 
                  type="monotone" 
                  dataKey="current" 
                  stroke="#ff6b35" 
                  strokeWidth={2}
                  dot={false}
                  activeDot={false}
                  isAnimationActive={false}
                  name="Current Fitness"
                />
              </LineChart>
            </ResponsiveContainer>
          </div>
        </div>

        <h3 className="text-xs text-purple-500/80 mt-2 mb-1">Generation History</h3>
        <div className="max-h-[200px] overflow-y-auto border border-gray-700 p-2 mb-3 bg-[#1a1a1a]">
          {sortedGenerations.length === 0 ? (
            <div className="text-gray-500 text-xs">No generations yet</div>
          ) : (
            sortedGenerations.map((gen: any) => {
              const date = new Date(gen.timestamp * 1000)
              return (
                <div
                  key={gen.generation}
                  onClick={() => handleViewGeneration(gen.generation)}
                  className="flex justify-between items-center py-1 border-b border-gray-800 cursor-pointer hover:bg-gray-800/50"
                >
                  <div>
                    <span className="text-accent-cyan font-bold">Gen {gen.generation}</span>
                    <span className="text-gray-500 ml-2">Fitness: {gen.fitness?.toFixed(1) || '0'}</span>
                    <span className="text-gray-500 ml-2">Best: {gen.best_fitness?.toFixed(1) || '0'}</span>
                  </div>
                  <div className="text-gray-600 text-[11px]">{date.toLocaleTimeString()}</div>
                </div>
              )
            })
          )}
        </div>

        <h3 className="text-xs text-purple-500/80 mt-2 mb-1">Population Records</h3>
        <div className="mb-2">
          <input
            ref={viewGenerationInputRef}
            type="number"
            placeholder="Generation"
            min="1"
            className="w-20 px-1.5 py-1 mr-2 font-mono text-xs bg-white/5 border border-white/10 rounded text-accent-cyan text-right focus:outline-none focus:border-accent-cyan"
          />
          <button
            onClick={handleViewPopulationInput}
            className="px-2 py-1 text-xs rounded bg-white/8 border border-white/10 text-text-primary hover:bg-white/12 transition-all inline-block"
          >
            View Population
          </button>
        </div>
        {viewingGeneration && (
          <div className="max-h-[300px] overflow-y-auto border border-gray-700 p-2 mb-3 bg-[#1a1a1a]">
            {populationRecords.length === 0 ? (
              <div className="text-gray-500 text-xs">Loading recordings...</div>
            ) : (
              <div>
                <div className="text-accent-cyan font-bold mb-2 text-xs">
                  Generation {viewingGeneration} - {populationRecords.length} solutions
                </div>
                <div className="text-gray-500 text-xs mb-2">
                  Click on a generation above to view all population recordings
                </div>
              </div>
            )}
          </div>
        )}

        <h3 className="text-xs text-purple-500/80 mt-2 mb-1">Config</h3>
        <div className="space-y-1">
          <div className="flex justify-between items-center py-1">
            <label className="text-xs text-text-dim">Generations</label>
            <input
              type="number"
              value={evotorchConfig.generations}
              onChange={(e) => {
                const current = store.get(evotorchConfigAtom)
                store.set(evotorchConfigAtom, { ...current, generations: parseInt(e.target.value) || 0 })
              }}
              min="10"
              max="10000"
              step="10"
              className="w-20 px-1.5 py-1 font-mono text-xs bg-white/5 border border-white/10 rounded text-accent-cyan text-right focus:outline-none focus:border-accent-cyan"
            />
          </div>
          <div className="flex justify-between items-center py-1">
            <label className="text-xs text-text-dim">Population Size</label>
            <input
              type="number"
              value={evotorchConfig.populationSize}
              onChange={(e) => {
                const current = store.get(evotorchConfigAtom)
                store.set(evotorchConfigAtom, { ...current, populationSize: parseInt(e.target.value) || 0 })
              }}
              min="10"
              max="500"
              step="10"
              className="w-20 px-1.5 py-1 font-mono text-xs bg-white/5 border border-white/10 rounded text-accent-cyan text-right focus:outline-none focus:border-accent-cyan"
            />
          </div>
          <div className="flex justify-between items-center py-1">
            <label className="text-xs text-text-dim">Max Speed</label>
            <input
              type="number"
              value={evotorchConfig.maxSpeed}
              onChange={(e) => {
                const current = store.get(evotorchConfigAtom)
                store.set(evotorchConfigAtom, { ...current, maxSpeed: parseInt(e.target.value) || 0 })
              }}
              min="1000"
              max="500000"
              step="1000"
              className="w-20 px-1.5 py-1 font-mono text-xs bg-white/5 border border-white/10 rounded text-accent-cyan text-right focus:outline-none focus:border-accent-cyan"
            />
          </div>
          <div className="flex justify-between items-center py-1">
            <label className="text-xs text-text-dim">Sim Steps</label>
            <input
              type="number"
              value={evotorchConfig.simSteps}
              onChange={(e) => {
                const current = store.get(evotorchConfigAtom)
                store.set(evotorchConfigAtom, { ...current, simSteps: parseInt(e.target.value) || 0 })
              }}
              min="100"
              max="10000"
              step="100"
              className="w-20 px-1.5 py-1 font-mono text-xs bg-white/5 border border-white/10 rounded text-accent-cyan text-right focus:outline-none focus:border-accent-cyan"
            />
          </div>
        </div>

        <button
          onClick={handleApplyConfig}
          className="w-full mt-2 px-2.5 py-1.5 text-xs rounded bg-white/8 border border-white/10 text-text-primary hover:bg-white/12 transition-all"
        >
          Apply Config
        </button>
      </div>
    </div>
  )
}

