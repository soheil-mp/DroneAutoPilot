import React, { createContext, useContext, useState } from 'react';
import axios from 'axios';

interface Waypoint {
  x: number;
  y: number;
  z: number;
}

interface MissionParameters {
  waypoints?: Waypoint[];
  boundary_points?: [number, number][];
  point?: [number, number, number];
  [key: string]: any;
}

interface MissionContextType {
  loadMission: (type: string, parameters: MissionParameters) => Promise<void>;
  abortMission: (missionId: string, returnToHome?: boolean) => Promise<void>;
  getMissionTypes: () => Promise<string[]>;
  loading: boolean;
  error: string | null;
}

const MissionContext = createContext<MissionContextType>({
  loadMission: async () => {},
  abortMission: async () => {},
  getMissionTypes: async () => [],
  loading: false,
  error: null,
});

export const useMission = () => useContext(MissionContext);

export const MissionProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const loadMission = async (type: string, parameters: MissionParameters) => {
    setLoading(true);
    setError(null);

    try {
      const response = await axios.post('/api/mission/load', {
        mission_type: type,
        parameters,
      });

      if (!response.data.success) {
        throw new Error(response.data.message);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to load mission');
      throw err;
    } finally {
      setLoading(false);
    }
  };

  const abortMission = async (missionId: string, returnToHome: boolean = true) => {
    setLoading(true);
    setError(null);

    try {
      const response = await axios.post('/api/mission/abort', {
        mission_id: missionId,
        return_to_home: returnToHome,
      });

      if (!response.data.success) {
        throw new Error(response.data.message);
      }
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to abort mission');
      throw err;
    } finally {
      setLoading(false);
    }
  };

  const getMissionTypes = async (): Promise<string[]> => {
    try {
      const response = await axios.get('/api/mission/types');
      return response.data.mission_types;
    } catch (err) {
      setError(err instanceof Error ? err.message : 'Failed to get mission types');
      return [];
    }
  };

  return (
    <MissionContext.Provider
      value={{
        loadMission,
        abortMission,
        getMissionTypes,
        loading,
        error,
      }}
    >
      {children}
    </MissionContext.Provider>
  );
}; 