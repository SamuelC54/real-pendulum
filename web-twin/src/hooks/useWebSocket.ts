import { useEffect, useRef } from 'react'
import { useSetAtom, useAtomValue } from 'jotai'
import { getDefaultStore } from 'jotai'
import { websocketAtom, connectedAtom } from '../store/atoms'

export function useWebSocket(url: string, onMessage: (data: any, set: any) => void) {
  const setWs = useSetAtom(websocketAtom)
  const setConnected = useSetAtom(connectedAtom)
  const reconnectTimeoutRef = useRef<number | null>(null)
  const onMessageRef = useRef(onMessage)
  const wsRef = useRef<WebSocket | null>(null)
  const store = getDefaultStore()

  useEffect(() => {
    onMessageRef.current = onMessage
  }, [onMessage])

  useEffect(() => {
    let mounted = true

    function connect() {
      if (!mounted) return

      try {
        const websocket = new WebSocket(url)
        wsRef.current = websocket
        
        websocket.onopen = () => {
          if (mounted) {
            setWs(websocket)
            setConnected(true)
            console.log('WebSocket connected')
          }
        }

        websocket.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data)
            onMessageRef.current(data, store.set)
          } catch (e) {
            console.error('Failed to parse message:', e)
          }
        }

        websocket.onclose = () => {
          if (mounted) {
            setWs(null)
            setConnected(false)
            reconnectTimeoutRef.current = window.setTimeout(() => {
              if (mounted) connect()
            }, 2000)
          }
        }

        websocket.onerror = (error) => {
          console.error('WebSocket error:', error)
          setConnected(false)
        }
      } catch (e) {
        console.error('Failed to create WebSocket:', e)
        reconnectTimeoutRef.current = window.setTimeout(() => {
          if (mounted) connect()
        }, 2000)
      }
    }

    connect()

    return () => {
      mounted = false
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current)
      }
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [url, setWs, setConnected, store])

  return useAtomValue(websocketAtom)
}

