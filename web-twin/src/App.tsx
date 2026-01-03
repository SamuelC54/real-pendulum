import TrainingPanel from './components/TrainingPanel'
import PendulumVisualization from './components/PendulumVisualization'
import InfoPanel from './components/InfoPanel'
import { useWebSocket } from './hooks/useWebSocket'
import { updatePendulumState, updateControlConfig } from './store/actions'

function App() {
  useWebSocket('ws://127.0.0.1:8765', (data, set) => {
    if (data.type === 'STATE') {
      updatePendulumState(data, set)
      updateControlConfig(data, set)
    }
  })

  return (
    <div className="flex h-screen w-screen overflow-hidden">
      <TrainingPanel />
      <div className="flex-1 min-w-0">
        <PendulumVisualization />
      </div>
      <InfoPanel />
    </div>
  )
}

export default App

