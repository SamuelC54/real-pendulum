import { useAtomValue } from 'jotai'
import { LineChart, Line, XAxis, YAxis, CartesianGrid, Legend, ResponsiveContainer } from 'recharts'
import { fitnessHistoryAtom } from '../store/atoms'

export default function FitnessChart() {
  const fitnessHistory = useAtomValue(fitnessHistoryAtom)

  return (
    <div className="bg-black/30 rounded p-2 mb-2">
      <div className="w-full" style={{ height: '120px' }}>
        <ResponsiveContainer width="100%" height="100%">
          <LineChart 
            data={fitnessHistory.length > 0 ? fitnessHistory : [{ generation: 0, best: 0, current: 0 }]}
            margin={{ top: 5, right: 5, bottom: 5, left: 5 }}
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
  )
}

