import React from 'react';
import {
  List,
  ListItem,
  ListItemIcon,
  ListItemText,
  Divider,
  Typography,
  Box,
} from '@mui/material';
import {
  Map,
  FlightTakeoff,
  Settings,
  Timeline,
  Battery90,
  Speed,
  Navigation,
  Warning,
} from '@mui/icons-material';
import { useWebSocket } from '../contexts/WebSocketContext';

const Sidebar: React.FC = () => {
  const { position, orientation, missionStatus } = useWebSocket();
  
  const calculateSpeed = () => {
    // TODO: Calculate actual speed from position changes
    return '0.0 m/s';
  };
  
  const calculateHeading = () => {
    if (!orientation) return '0°';
    // Convert quaternion to Euler angles
    const { x, y, z, w } = orientation;
    const heading = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return `${(heading * 180 / Math.PI).toFixed(1)}°`;
  };
  
  return (
    <Box sx={{ width: '100%' }}>
      <List>
        {/* Map Section */}
        <ListItem>
          <ListItemIcon>
            <Map />
          </ListItemIcon>
          <ListItemText
            primary="Map View"
            secondary={position ? `(${position.x.toFixed(2)}, ${position.y.toFixed(2)})` : 'No position'}
          />
        </ListItem>
        
        <Divider />
        
        {/* Mission Section */}
        <ListItem>
          <ListItemIcon>
            <FlightTakeoff />
          </ListItemIcon>
          <ListItemText
            primary="Mission"
            secondary={missionStatus?.state || 'No active mission'}
          />
        </ListItem>
        
        {missionStatus && (
          <>
            <ListItem>
              <ListItemIcon>
                <Timeline />
              </ListItemIcon>
              <ListItemText
                primary="Progress"
                secondary={`${(missionStatus.progress * 100).toFixed(1)}%`}
              />
            </ListItem>
            
            <ListItem>
              <ListItemIcon>
                <Battery90 />
              </ListItemIcon>
              <ListItemText
                primary="Battery"
                secondary={`${missionStatus.battery.toFixed(1)}%`}
              />
            </ListItem>
          </>
        )}
        
        <Divider />
        
        {/* Telemetry Section */}
        <ListItem>
          <ListItemText>
            <Typography variant="subtitle2" color="text.secondary">
              Telemetry
            </Typography>
          </ListItemText>
        </ListItem>
        
        <ListItem>
          <ListItemIcon>
            <Speed />
          </ListItemIcon>
          <ListItemText
            primary="Speed"
            secondary={calculateSpeed()}
          />
        </ListItem>
        
        <ListItem>
          <ListItemIcon>
            <Navigation />
          </ListItemIcon>
          <ListItemText
            primary="Heading"
            secondary={calculateHeading()}
          />
        </ListItem>
        
        <ListItem>
          <ListItemIcon>
            <Warning />
          </ListItemIcon>
          <ListItemText
            primary="Altitude"
            secondary={position ? `${position.z.toFixed(1)}m` : 'N/A'}
          />
        </ListItem>
        
        <Divider />
        
        {/* Settings Section */}
        <ListItem button>
          <ListItemIcon>
            <Settings />
          </ListItemIcon>
          <ListItemText primary="Settings" />
        </ListItem>
      </List>
    </Box>
  );
};

export default Sidebar; 