import { useState, useEffect } from 'react'
import { Stage, Layer, Group, Line, Circle, Rect, Text, Shape } from 'react-konva'
import { useAtomValue } from 'jotai'
import { pendulumStateAtom } from '../store/atoms'

const PENDULUM_LENGTH = 200
const BOB_RADIUS = 25
const PIVOT_RADIUS = 12

export default function PendulumVisualization() {
  const state = useAtomValue(pendulumStateAtom)
  const [dimensions, setDimensions] = useState({ width: 800, height: 600 })

  // Update dimensions on resize
  useEffect(() => {
    const updateDimensions = () => {
      const container = document.querySelector('[data-pendulum-container]') as HTMLElement
      if (container) {
        const rect = container.getBoundingClientRect()
        setDimensions({
          width: rect.width || window.innerWidth * 0.6 || 800,
          height: rect.height || window.innerHeight || 600
        })
      } else {
        setDimensions({
          width: window.innerWidth * 0.6 || 800,
          height: window.innerHeight || 600
        })
      }
    }

    updateDimensions()
    window.addEventListener('resize', updateDimensions)
    return () => window.removeEventListener('resize', updateDimensions)
  }, [])

  const centerX = dimensions.width / 2
  const centerY = dimensions.height / 2 - 50
  const RAIL_WIDTH = dimensions.width * 0.7
  const RAIL_Y = centerY - 30
  const CART_WIDTH = 60
  const CART_HEIGHT = 30

  // Calculate cart position
  const range = state.limitRightPos - state.limitLeftPos
  let normalizedPos = 0.5
  if (range > 0) {
    normalizedPos = (state.position - state.limitLeftPos) / range
  }
  const cartX = centerX - RAIL_WIDTH / 2 + RAIL_WIDTH * normalizedPos

  // Calculate pendulum angle
  const angleRad = state.angle * (Math.PI / 180)

  // Calculate angle text position
  const startAngle = Math.PI / 2
  const textAngle = startAngle + angleRad / 2
  const arcRadius = 60
  const angleTextX = cartX + (arcRadius + 35) * Math.cos(textAngle)
  const angleTextY = centerY + (arcRadius + 35) * Math.sin(textAngle)

  // Calculate arc for angle visualization
  const showArc = Math.abs(state.angle) > 0.5
  const endAngle = startAngle + angleRad
  const arcStart = Math.min(startAngle, endAngle)
  const arcEnd = Math.max(startAngle, endAngle)

  return (
    <div
      data-pendulum-container
      className={`flex-1 flex items-center justify-center relative ${
        state.connected ? 'opacity-100' : 'opacity-0'
      } transition-opacity duration-300`}
      style={{ width: '100%', height: '100%', minWidth: 0, minHeight: 0 }}
    >
      <Stage width={dimensions.width} height={dimensions.height}>
        <Layer>
          {/* Rail */}
          <Line
            points={[centerX - RAIL_WIDTH / 2, RAIL_Y, centerX + RAIL_WIDTH / 2, RAIL_Y]}
            stroke="rgba(255, 255, 255, 0.3)"
            strokeWidth={4}
            lineCap="round"
          />

          {/* Limit indicators */}
          <Circle
            x={centerX - RAIL_WIDTH / 2 + 10}
            y={RAIL_Y}
            radius={8}
            fill={state.limitLeft ? '#ff4444' : 'rgba(100, 100, 100, 0.5)'}
            stroke="rgba(255, 255, 255, 0.2)"
            strokeWidth={2}
          />
          <Circle
            x={centerX + RAIL_WIDTH / 2 - 10}
            y={RAIL_Y}
            radius={8}
            fill={state.limitRight ? '#ff4444' : 'rgba(100, 100, 100, 0.5)'}
            stroke="rgba(255, 255, 255, 0.2)"
            strokeWidth={2}
          />

          {/* Cart */}
          <Rect
            x={cartX - CART_WIDTH / 2}
            y={RAIL_Y - CART_HEIGHT / 2}
            width={CART_WIDTH}
            height={CART_HEIGHT}
            cornerRadius={5}
            fill="rgba(0, 212, 255, 0.3)"
            stroke="#00d4ff"
            strokeWidth={2}
          />

          {/* Pendulum group */}
          <Group x={cartX} y={centerY} rotation={angleRad * (180 / Math.PI)}>
            {/* Rod */}
            <Line
              points={[0, 0, 0, PENDULUM_LENGTH]}
              stroke="#4a9eff"
              strokeWidth={4}
              lineCap="round"
            />

            {/* Bob glow */}
            <Circle
              x={0}
              y={PENDULUM_LENGTH}
              radius={BOB_RADIUS + 10}
              fill="rgba(255, 107, 53, 0.2)"
            />

            {/* Bob */}
            <Circle
              x={0}
              y={PENDULUM_LENGTH}
              radius={BOB_RADIUS}
              fill="#ff6b35"
              stroke="#cc5529"
              strokeWidth={3}
            />

            {/* Pivot glow */}
            <Circle
              x={0}
              y={0}
              radius={PIVOT_RADIUS + 8}
              fill="rgba(0, 212, 255, 0.2)"
            />

            {/* Pivot */}
            <Circle
              x={0}
              y={0}
              radius={PIVOT_RADIUS}
              fill="#00d4ff"
              stroke="#00a8cc"
              strokeWidth={2}
            />

            {/* Reference line */}
            <Line
              points={[0, 0, 0, 80]}
              stroke="rgba(255, 255, 255, 0.2)"
              strokeWidth={1}
              dash={[5, 5]}
            />
          </Group>

          {/* Angle text */}
          <Text
            x={angleTextX}
            y={angleTextY}
            text={`${state.angle.toFixed(1)}°`}
            fill="#ff00aa"
            fontSize={16}
            fontFamily="JetBrains Mono, monospace"
            align="center"
            offsetX={20}
            offsetY={8}
          />

          {/* Angle arc visualization */}
          {showArc && (
            <Shape
              sceneFunc={(context) => {
                context.beginPath()
                context.arc(
                  cartX,
                  centerY,
                  arcRadius - 5,
                  arcStart,
                  arcEnd,
                  false
                )
                context.arc(
                  cartX,
                  centerY,
                  arcRadius + 5,
                  arcEnd,
                  arcStart,
                  true
                )
                context.closePath()
                context.fillStyle = 'rgba(255, 0, 170, 0.3)'
                context.fill()
                context.strokeStyle = '#ff00aa'
                context.lineWidth = 2
                context.stroke()
              }}
            />
          )}
        </Layer>
      </Stage>

      {!state.connected && (
        <div className="absolute flex flex-col items-center gap-5 z-10">
          <div className="w-[60px] h-[60px] border-4 border-accent-cyan/15 border-t-accent-cyan rounded-full animate-spin" />
          <span className="font-mono text-sm text-text-dim animate-pulse">Waiting for connection...</span>
        </div>
      )}
    </div>
  )
}
