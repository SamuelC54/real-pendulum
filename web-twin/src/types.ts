export interface PendulumState {
  angle: number
  angularVelocity: number
  position: number
  velocity: number
  limitLeft: boolean
  limitRight: boolean
  mode: string
  connected: boolean
  limitLeftPos: number
  limitRightPos: number
}

export interface ControlConfig {
  manualAccel: number
  oscillateSpeed: number
  oscillatePeriod: number
}

export interface TrainingStats {
  trainer?: string
  generation?: number
  best_fitness?: number
  current_fitness?: number
  population_size?: number
}

