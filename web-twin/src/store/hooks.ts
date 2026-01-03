import { useAtomValue, useSetAtom } from 'jotai'
import { websocketAtom } from './atoms'

export function useSendCommand() {
  const ws = useAtomValue(websocketAtom)
  
  return (cmd: any) => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(cmd))
    }
  }
}

