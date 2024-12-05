import React, { createContext, useContext, useEffect, useState } from 'react';

interface DronePosition {
  x: number;
  y: number;
  z: number;
}

interface DroneOrientation {
  x: number;
  y: number;
  z: number;
  w: number;
}

interface MissionStatus {
  mission_id: string;
  state: string;
  progress: number;
  battery: number;
  current_waypoint: number;
  total_waypoints: number;
}

interface WebSocketContextType {
  connected: boolean;
  position: DronePosition | null;
  orientation: DroneOrientation | null;
  missionStatus: MissionStatus | null;
}

const WebSocketContext = createContext<WebSocketContextType>({
  connected: false,
  position: null,
  orientation: null,
  missionStatus: null,
});

export const useWebSocket = () => useContext(WebSocketContext);

export const WebSocketProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [socket, setSocket] = useState<WebSocket | null>(null);
  const [connected, setConnected] = useState(false);
  const [position, setPosition] = useState<DronePosition | null>(null);
  const [orientation, setOrientation] = useState<DroneOrientation | null>(null);
  const [missionStatus, setMissionStatus] = useState<MissionStatus | null>(null);

  useEffect(() => {
    // Connect to WebSocket server
    const ws = new WebSocket('ws://localhost:8000/ws');

    ws.onopen = () => {
      console.log('WebSocket connected');
      setConnected(true);
      setSocket(ws);
    };

    ws.onclose = () => {
      console.log('WebSocket disconnected');
      setConnected(false);
      setSocket(null);
    };

    ws.onerror = (error) => {
      console.error('WebSocket error:', error);
    };

    ws.onmessage = (event) => {
      const message = JSON.parse(event.data);

      switch (message.type) {
        case 'drone_pose':
          setPosition(message.data.position);
          setOrientation(message.data.orientation);
          break;

        case 'mission_status':
          setMissionStatus(message.data);
          break;

        default:
          console.warn('Unknown message type:', message.type);
      }
    };

    // Cleanup on unmount
    return () => {
      if (ws) {
        ws.close();
      }
    };
  }, []);

  return (
    <WebSocketContext.Provider
      value={{
        connected,
        position,
        orientation,
        missionStatus,
      }}
    >
      {children}
    </WebSocketContext.Provider>
  );
}; 