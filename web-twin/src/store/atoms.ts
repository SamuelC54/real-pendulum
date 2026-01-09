import { atom } from 'jotai'
import type { PendulumState, ControlConfig, TrainingStats } from '../types'

// Pendulum state atoms
export const angleAtom = atom(0)
export const angularVelocityAtom = atom(0)
export const positionAtom = atom(0)
export const velocityAtom = atom(0)
export const limitLeftAtom = atom(false)
export const limitRightAtom = atom(false)
export const modeAtom = atom<string>('idle')
export const connectedAtom = atom(false)
export const limitLeftPosAtom = atom(0)
export const limitRightPosAtom = atom(0)

// Combined pendulum state atom (read-only, computed from individual atoms)
export const pendulumStateAtom = atom((get) => ({
  angle: get(angleAtom),
  angularVelocity: get(angularVelocityAtom),
  position: get(positionAtom),
  velocity: get(velocityAtom),
  limitLeft: get(limitLeftAtom),
  limitRight: get(limitRightAtom),
  mode: get(modeAtom),
  connected: get(connectedAtom),
  limitLeftPos: get(limitLeftPosAtom),
  limitRightPos: get(limitRightPosAtom),
}))

// Control config atoms
export const manualAccelAtom = atom(50000)
export const oscillateSpeedAtom = atom(3000)
export const oscillatePeriodAtom = atom(2.0)
export const lqrQAngleAtom = atom(20.0) // Optimized via tuning
export const lqrQAngleVelAtom = atom(2.0) // Optimized via tuning
export const lqrQPositionAtom = atom(100.0) // Optimized via tuning
export const lqrQVelocityAtom = atom(0.2)
export const lqrRControlAtom = atom(5.0) // Optimized via tuning

// Combined control config atom (read-only, computed from individual atoms)
export const controlConfigAtom = atom((get) => ({
  manualAccel: get(manualAccelAtom),
  oscillateSpeed: get(oscillateSpeedAtom),
  oscillatePeriod: get(oscillatePeriodAtom),
  lqrQAngle: get(lqrQAngleAtom),
  lqrQAngleVel: get(lqrQAngleVelAtom),
  lqrQPosition: get(lqrQPositionAtom),
  lqrQVelocity: get(lqrQVelocityAtom),
  lqrRControl: get(lqrRControlAtom),
}))

// Training state atoms
export const trainingActiveAtom = atom(false)
export const trainingStatsAtom = atom<TrainingStats>({})
export const fitnessHistoryAtom = atom<Array<{ generation: number; best: number; current: number }>>([])
export const generationsAtom = atom<Record<string, any>>({})
export const populationRecordsAtom = atom<any[]>([])
export const viewingGenerationAtom = atom<number | null>(null)
export const evotorchConfigAtom = atom({
  generations: 100,
  populationSize: 50,
  maxSpeed: 9000,
  simSteps: 5000,
})

// WebSocket atom
export const websocketAtom = atom<WebSocket | null>(null)

