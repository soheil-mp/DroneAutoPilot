import React from 'react';
import { ThemeProvider, createTheme, CssBaseline } from '@mui/material';
import { BrowserRouter as Router } from 'react-router-dom';
import Layout from './components/Layout';
import { WebSocketProvider } from './contexts/WebSocketContext';
import { MissionProvider } from './contexts/MissionContext';

const theme = createTheme({
  palette: {
    mode: 'dark',
    primary: {
      main: '#2196f3',
    },
    secondary: {
      main: '#f50057',
    },
  },
});

function App() {
  return (
    <ThemeProvider theme={theme}>
      <CssBaseline />
      <WebSocketProvider>
        <MissionProvider>
          <Router>
            <Layout />
          </Router>
        </MissionProvider>
      </WebSocketProvider>
    </ThemeProvider>
  );
}

export default App; 