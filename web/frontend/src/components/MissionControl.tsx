import React, { useEffect, useState } from 'react';
import {
  Box,
  Button,
  Card,
  CardContent,
  CircularProgress,
  FormControl,
  InputLabel,
  MenuItem,
  Paper,
  Select,
  TextField,
  Typography,
} from '@mui/material';
import { useMission } from '../contexts/MissionContext';
import { useWebSocket } from '../contexts/WebSocketContext';

const MissionControl: React.FC = () => {
  const { loadMission, abortMission, getMissionTypes, loading, error } = useMission();
  const { missionStatus } = useWebSocket();
  
  const [missionTypes, setMissionTypes] = useState<string[]>([]);
  const [selectedType, setSelectedType] = useState('');
  const [parameters, setParameters] = useState<any>({});
  
  // Load mission types on mount
  useEffect(() => {
    getMissionTypes().then(setMissionTypes);
  }, [getMissionTypes]);
  
  const handleTypeChange = (event: React.ChangeEvent<{ value: unknown }>) => {
    setSelectedType(event.target.value as string);
    setParameters({});
  };
  
  const handleParameterChange = (key: string, value: any) => {
    setParameters(prev => ({ ...prev, [key]: value }));
  };
  
  const handleLoadMission = async () => {
    try {
      await loadMission(selectedType, parameters);
    } catch (err) {
      console.error('Failed to load mission:', err);
    }
  };
  
  const handleAbortMission = async () => {
    if (!missionStatus?.mission_id) return;
    
    try {
      await abortMission(missionStatus.mission_id);
    } catch (err) {
      console.error('Failed to abort mission:', err);
    }
  };
  
  const renderParameterFields = () => {
    switch (selectedType) {
      case 'waypoint_navigation':
        return (
          <TextField
            fullWidth
            multiline
            rows={4}
            label="Waypoints (JSON)"
            value={JSON.stringify(parameters.waypoints || [], null, 2)}
            onChange={(e) => {
              try {
                const waypoints = JSON.parse(e.target.value);
                handleParameterChange('waypoints', waypoints);
              } catch (err) {
                // Invalid JSON, ignore
              }
            }}
            margin="normal"
          />
        );
        
      case 'area_coverage':
        return (
          <TextField
            fullWidth
            multiline
            rows={4}
            label="Boundary Points (JSON)"
            value={JSON.stringify(parameters.boundary_points || [], null, 2)}
            onChange={(e) => {
              try {
                const points = JSON.parse(e.target.value);
                handleParameterChange('boundary_points', points);
              } catch (err) {
                // Invalid JSON, ignore
              }
            }}
            margin="normal"
          />
        );
        
      case 'point_of_interest':
        return (
          <TextField
            fullWidth
            label="Point [x, y, z]"
            value={JSON.stringify(parameters.point || [0, 0, 0])}
            onChange={(e) => {
              try {
                const point = JSON.parse(e.target.value);
                handleParameterChange('point', point);
              } catch (err) {
                // Invalid JSON, ignore
              }
            }}
            margin="normal"
          />
        );
        
      default:
        return null;
    }
  };
  
  return (
    <Paper elevation={3} sx={{ height: '100%', p: 2, overflow: 'auto' }}>
      <Typography variant="h6" gutterBottom>
        Mission Control
      </Typography>
      
      {/* Mission Status */}
      <Card sx={{ mb: 2 }}>
        <CardContent>
          <Typography variant="subtitle1" gutterBottom>
            Status
          </Typography>
          <Box sx={{ display: 'grid', gap: 1 }}>
            <Typography variant="body2">
              State: {missionStatus?.state || 'N/A'}
            </Typography>
            <Typography variant="body2">
              Progress: {missionStatus ? `${(missionStatus.progress * 100).toFixed(1)}%` : 'N/A'}
            </Typography>
            <Typography variant="body2">
              Battery: {missionStatus ? `${missionStatus.battery.toFixed(1)}%` : 'N/A'}
            </Typography>
          </Box>
        </CardContent>
      </Card>
      
      {/* Mission Configuration */}
      <Card>
        <CardContent>
          <Typography variant="subtitle1" gutterBottom>
            New Mission
          </Typography>
          
          <FormControl fullWidth margin="normal">
            <InputLabel>Mission Type</InputLabel>
            <Select
              value={selectedType}
              onChange={handleTypeChange}
              label="Mission Type"
            >
              {missionTypes.map(type => (
                <MenuItem key={type} value={type}>
                  {type.replace(/_/g, ' ').toUpperCase()}
                </MenuItem>
              ))}
            </Select>
          </FormControl>
          
          {renderParameterFields()}
          
          {error && (
            <Typography color="error" variant="body2" sx={{ mt: 1 }}>
              {error}
            </Typography>
          )}
          
          <Box sx={{ mt: 2, display: 'flex', gap: 2 }}>
            <Button
              variant="contained"
              color="primary"
              onClick={handleLoadMission}
              disabled={loading || !selectedType}
              startIcon={loading && <CircularProgress size={20} />}
            >
              Load Mission
            </Button>
            
            <Button
              variant="contained"
              color="secondary"
              onClick={handleAbortMission}
              disabled={loading || !missionStatus?.mission_id}
            >
              Abort Mission
            </Button>
          </Box>
        </CardContent>
      </Card>
    </Paper>
  );
};

export default MissionControl; 