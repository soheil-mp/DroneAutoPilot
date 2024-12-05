import React, { useEffect, useState } from 'react';
import { MapContainer, TileLayer, Marker, Polyline, useMap } from 'react-leaflet';
import L from 'leaflet';
import 'leaflet/dist/leaflet.css';
import { Box, Paper } from '@mui/material';
import { useWebSocket } from '../contexts/WebSocketContext';

// Fix for default marker icons
delete (L.Icon.Default.prototype as any)._getIconUrl;
L.Icon.Default.mergeOptions({
  iconRetinaUrl: require('leaflet/dist/images/marker-icon-2x.png'),
  iconUrl: require('leaflet/dist/images/marker-icon.png'),
  shadowUrl: require('leaflet/dist/images/marker-shadow.png'),
});

// Custom drone icon
const droneIcon = new L.Icon({
  iconUrl: '/drone-icon.png',
  iconSize: [32, 32],
  iconAnchor: [16, 16],
});

interface MapCenterProps {
  position: [number, number];
}

const MapCenter: React.FC<MapCenterProps> = ({ position }) => {
  const map = useMap();
  
  useEffect(() => {
    map.setView(position);
  }, [map, position]);
  
  return null;
};

const Map: React.FC = () => {
  const { position, missionStatus } = useWebSocket();
  const [path, setPath] = useState<[number, number][]>([]);
  
  // Update path when position changes
  useEffect(() => {
    if (position) {
      setPath(prev => [...prev, [position.y, position.x]]);
    }
  }, [position]);
  
  // Clear path when mission changes
  useEffect(() => {
    if (missionStatus?.state === 'IDLE') {
      setPath([]);
    }
  }, [missionStatus?.state]);
  
  return (
    <Paper
      elevation={3}
      sx={{
        height: '100%',
        overflow: 'hidden',
        '& .leaflet-container': {
          height: '100%',
          width: '100%',
        },
      }}
    >
      <MapContainer
        center={[0, 0]}
        zoom={18}
        style={{ height: '100%' }}
      >
        {/* Base map layer */}
        <TileLayer
          attribution='&copy; <a href="https://www.openstreetmap.org/copyright">OpenStreetMap</a> contributors'
          url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
        />
        
        {/* Drone marker */}
        {position && (
          <>
            <Marker
              position={[position.y, position.x]}
              icon={droneIcon}
              rotationAngle={0}  // TODO: Calculate from orientation
            />
            <MapCenter position={[position.y, position.x]} />
          </>
        )}
        
        {/* Flight path */}
        {path.length > 1 && (
          <Polyline
            positions={path}
            color="#2196f3"
            weight={3}
            opacity={0.7}
          />
        )}
        
        {/* Mission waypoints */}
        {missionStatus?.waypoints?.map((waypoint, index) => (
          <Marker
            key={index}
            position={[waypoint.y, waypoint.x]}
            icon={new L.Icon({
              iconUrl: index < missionStatus.current_waypoint
                ? '/waypoint-completed.png'
                : index === missionStatus.current_waypoint
                ? '/waypoint-current.png'
                : '/waypoint-pending.png',
              iconSize: [24, 24],
              iconAnchor: [12, 12],
            })}
          />
        ))}
      </MapContainer>
    </Paper>
  );
};

export default Map; 