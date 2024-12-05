import React from 'react';
import {
  Box,
  Chip,
  IconButton,
  Menu,
  MenuItem,
  Typography,
} from '@mui/material';
import {
  Battery20,
  Battery30,
  Battery50,
  Battery60,
  Battery80,
  Battery90,
  BatteryFull,
  SignalWifi4Bar,
  SignalWifi3Bar,
  SignalWifi2Bar,
  SignalWifi1Bar,
  SignalWifiOff,
  FlightTakeoff,
  Warning,
} from '@mui/icons-material';
import { useWebSocket } from '../contexts/WebSocketContext';

const DroneStatus: React.FC = () => {
  const { connected, missionStatus } = useWebSocket();
  const [anchorEl, setAnchorEl] = React.useState<null | HTMLElement>(null);
  
  const handleClick = (event: React.MouseEvent<HTMLElement>) => {
    setAnchorEl(event.currentTarget);
  };
  
  const handleClose = () => {
    setAnchorEl(null);
  };
  
  const getBatteryIcon = (level: number) => {
    if (level > 90) return <BatteryFull />;
    if (level > 80) return <Battery90 />;
    if (level > 60) return <Battery80 />;
    if (level > 50) return <Battery60 />;
    if (level > 30) return <Battery50 />;
    if (level > 20) return <Battery30 />;
    return <Battery20 color="error" />;
  };
  
  const getConnectionIcon = () => {
    if (!connected) return <SignalWifiOff color="error" />;
    const strength = Math.random(); // TODO: Replace with actual signal strength
    if (strength > 0.75) return <SignalWifi4Bar />;
    if (strength > 0.5) return <SignalWifi3Bar />;
    if (strength > 0.25) return <SignalWifi2Bar />;
    return <SignalWifi1Bar />;
  };
  
  const getStatusColor = () => {
    if (!connected) return 'error';
    if (missionStatus?.state === 'EXECUTING') return 'success';
    if (missionStatus?.state === 'PAUSED') return 'warning';
    return 'default';
  };
  
  return (
    <>
      <Box sx={{ display: 'flex', alignItems: 'center', gap: 1 }}>
        {/* Connection status */}
        <Chip
          icon={getConnectionIcon()}
          label={connected ? 'Connected' : 'Disconnected'}
          color={connected ? 'success' : 'error'}
          size="small"
        />
        
        {/* Battery status */}
        {missionStatus && (
          <Chip
            icon={getBatteryIcon(missionStatus.battery)}
            label={`${missionStatus.battery.toFixed(1)}%`}
            color={missionStatus.battery < 20 ? 'error' : 'default'}
            size="small"
          />
        )}
        
        {/* Mission status */}
        {missionStatus && (
          <Chip
            icon={<FlightTakeoff />}
            label={missionStatus.state}
            color={getStatusColor()}
            size="small"
          />
        )}
        
        {/* Status details button */}
        <IconButton
          color="inherit"
          onClick={handleClick}
          size="small"
        >
          <Warning />
        </IconButton>
      </Box>
      
      {/* Status details menu */}
      <Menu
        anchorEl={anchorEl}
        open={Boolean(anchorEl)}
        onClose={handleClose}
      >
        <MenuItem>
          <Typography variant="body2">
            Mission: {missionStatus?.mission_id || 'None'}
          </Typography>
        </MenuItem>
        <MenuItem>
          <Typography variant="body2">
            Progress: {missionStatus ? `${(missionStatus.progress * 100).toFixed(1)}%` : 'N/A'}
          </Typography>
        </MenuItem>
        <MenuItem>
          <Typography variant="body2">
            Waypoint: {missionStatus ? `${missionStatus.current_waypoint + 1}/${missionStatus.total_waypoints}` : 'N/A'}
          </Typography>
        </MenuItem>
        {missionStatus?.position && (
          <MenuItem>
            <Typography variant="body2">
              Position: ({missionStatus.position.x.toFixed(2)}, {missionStatus.position.y.toFixed(2)}, {missionStatus.position.z.toFixed(2)})
            </Typography>
          </MenuItem>
        )}
      </Menu>
    </>
  );
};

export default DroneStatus; 