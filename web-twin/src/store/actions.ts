import {
  angleAtom,
  angularVelocityAtom,
  positionAtom,
  velocityAtom,
  limitLeftAtom,
  limitRightAtom,
  modeAtom,
  connectedAtom,
  limitLeftPosAtom,
  limitRightPosAtom,
  manualAccelAtom,
  oscillateSpeedAtom,
  oscillatePeriodAtom,
  lqrQAngleAtom,
  lqrQAngleVelAtom,
  lqrQPositionAtom,
  lqrQVelocityAtom,
  lqrRControlAtom,
  trainingActiveAtom,
  trainingStatsAtom,
  generationsAtom,
  populationRecordsAtom,
  viewingGenerationAtom,
  evotorchConfigAtom,
  websocketAtom,
} from './atoms'

// Pendulum state updaters
export const updatePendulumState = (data: any, set: any) => {
  if (data.angle !== undefined) set(angleAtom, data.angle)
  if (data.angular_velocity !== undefined) set(angularVelocityAtom, data.angular_velocity)
  if (data.position !== undefined) {
    set(positionAtom, data.position)
    if (data.limit_left !== undefined) set(limitLeftAtom, data.limit_left)
    if (data.limit_right !== undefined) set(limitRightAtom, data.limit_right)
  }
  if (data.velocity !== undefined) set(velocityAtom, data.velocity)
  if (data.mode !== undefined) set(modeAtom, data.mode)
  if (data.connected !== undefined) set(connectedAtom, data.connected)
  if (data.limit_left_pos !== undefined) set(limitLeftPosAtom, data.limit_left_pos)
  if (data.limit_right_pos !== undefined) set(limitRightPosAtom, data.limit_right_pos)
}

// Control config updaters
export const updateControlConfig = (data: any, set: any) => {
  if (data.config) {
    if (data.config.manual_accel !== undefined) set(manualAccelAtom, data.config.manual_accel)
    if (data.config.oscillate_speed !== undefined) set(oscillateSpeedAtom, data.config.oscillate_speed)
    if (data.config.oscillate_period !== undefined) set(oscillatePeriodAtom, data.config.oscillate_period)
    if (data.config.lqr_q_angle !== undefined) set(lqrQAngleAtom, data.config.lqr_q_angle)
    if (data.config.lqr_q_angle_vel !== undefined) set(lqrQAngleVelAtom, data.config.lqr_q_angle_vel)
    if (data.config.lqr_q_position !== undefined) set(lqrQPositionAtom, data.config.lqr_q_position)
    if (data.config.lqr_q_velocity !== undefined) set(lqrQVelocityAtom, data.config.lqr_q_velocity)
    if (data.config.lqr_r_control !== undefined) set(lqrRControlAtom, data.config.lqr_r_control)
  }
}

// Training state updaters
export const updateTrainingStats = (data: any, set: any) => {
  set(trainingStatsAtom, data)
  set(trainingActiveAtom, true)
}

export const setTrainingActive = (active: boolean, set: any) => {
  set(trainingActiveAtom, active)
}

export const setGenerations = (generations: Record<string, any>, set: any) => {
  set(generationsAtom, generations)
}

export const setPopulationRecords = (records: any[], set: any) => {
  set(populationRecordsAtom, records)
}

export const setViewingGeneration = (generation: number | null, set: any) => {
  set(viewingGenerationAtom, generation)
}

export const setEvotorchConfig = (config: any, set: any) => {
  set(evotorchConfigAtom, config)
}

export const setWebSocket = (ws: WebSocket | null, set: any) => {
  set(websocketAtom, ws)
}

