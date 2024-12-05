#!/usr/bin/env python3

import os
import yaml
import json
import asyncio
import rclpy
from rclpy.node import Node
from fastapi import FastAPI, WebSocket, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from typing import List, Dict, Optional
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from drone_nav_rl_interfaces.msg import MissionStatus
from drone_nav_rl_interfaces.srv import LoadMission, AbortMission
from drone_nav_rl_interfaces.action import ExecuteMission

class WebInterface(Node):
    def __init__(self):
        super().__init__('web_interface')
        
        # Initialize ROS2 subscribers
        self.mission_status_sub = self.create_subscription(
            MissionStatus,
            '/mission/status',
            self.mission_status_callback,
            10
        )
        
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/drone/pose',
            self.pose_callback,
            10
        )
        
        # Initialize ROS2 service clients
        self.load_mission_client = self.create_client(
            LoadMission,
            '/mission/load'
        )
        
        self.abort_mission_client = self.create_client(
            AbortMission,
            '/mission/abort'
        )
        
        # Store latest data
        self.latest_status = None
        self.latest_pose = None
        self.connected_clients = set()
        
    def mission_status_callback(self, msg: MissionStatus):
        """Handle mission status updates"""
        self.latest_status = msg
        asyncio.create_task(self.broadcast_status())
        
    def pose_callback(self, msg: PoseStamped):
        """Handle drone pose updates"""
        self.latest_pose = msg
        asyncio.create_task(self.broadcast_pose())
        
    async def broadcast_status(self):
        """Broadcast mission status to all connected clients"""
        if not self.latest_status:
            return
            
        message = {
            'type': 'mission_status',
            'data': {
                'mission_id': self.latest_status.mission_id,
                'state': self.latest_status.state,
                'progress': self.latest_status.mission_progress,
                'battery': self.latest_status.battery_remaining,
                'current_waypoint': self.latest_status.current_waypoint,
                'total_waypoints': self.latest_status.total_waypoints
            }
        }
        
        await self.broadcast_message(message)
        
    async def broadcast_pose(self):
        """Broadcast drone pose to all connected clients"""
        if not self.latest_pose:
            return
            
        message = {
            'type': 'drone_pose',
            'data': {
                'position': {
                    'x': self.latest_pose.pose.position.x,
                    'y': self.latest_pose.pose.position.y,
                    'z': self.latest_pose.pose.position.z
                },
                'orientation': {
                    'x': self.latest_pose.pose.orientation.x,
                    'y': self.latest_pose.pose.orientation.y,
                    'z': self.latest_pose.pose.orientation.z,
                    'w': self.latest_pose.pose.orientation.w
                }
            }
        }
        
        await self.broadcast_message(message)
        
    async def broadcast_message(self, message: Dict):
        """Send message to all connected WebSocket clients"""
        for client in self.connected_clients:
            try:
                await client.send_json(message)
            except:
                self.connected_clients.remove(client)
                
    async def load_mission(self, mission_type: str, parameters: Dict) -> Dict:
        """Load a new mission"""
        if not self.load_mission_client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(status_code=503, detail="Mission service unavailable")
            
        request = LoadMission.Request()
        request.mission_type = mission_type
        request.parameters = json.dumps(parameters)
        
        response = await self.load_mission_client.call_async(request)
        
        if not response.success:
            raise HTTPException(status_code=400, detail=response.message)
            
        return {
            'success': True,
            'message': response.message,
            'waypoints': [
                {
                    'x': pose.position.x,
                    'y': pose.position.y,
                    'z': pose.position.z
                }
                for pose in response.waypoints.poses
            ]
        }
        
    async def abort_mission(self, mission_id: str, return_to_home: bool) -> Dict:
        """Abort current mission"""
        if not self.abort_mission_client.wait_for_service(timeout_sec=1.0):
            raise HTTPException(status_code=503, detail="Abort service unavailable")
            
        request = AbortMission.Request()
        request.mission_id = mission_id
        request.return_to_home = return_to_home
        
        response = await self.abort_mission_client.call_async(request)
        
        if not response.success:
            raise HTTPException(status_code=400, detail=response.message)
            
        return {
            'success': True,
            'message': response.message
        }

# Initialize FastAPI app
app = FastAPI(title="Drone Navigation Web Interface")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Initialize ROS2 node
rclpy.init()
web_interface = WebInterface()

# Serve static files (React frontend)
app.mount("/", StaticFiles(directory="../frontend/build", html=True), name="static")

# API Models
class MissionRequest(BaseModel):
    mission_type: str
    parameters: Dict

class AbortRequest(BaseModel):
    mission_id: str
    return_to_home: bool = True

# WebSocket endpoint
@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    web_interface.connected_clients.add(websocket)
    
    try:
        while True:
            # Keep connection alive and handle incoming messages
            data = await websocket.receive_text()
            # Process any incoming WebSocket messages if needed
    except:
        web_interface.connected_clients.remove(websocket)

# REST endpoints
@app.post("/api/mission/load")
async def load_mission(request: MissionRequest):
    """Load a new mission"""
    return await web_interface.load_mission(
        request.mission_type,
        request.parameters
    )

@app.post("/api/mission/abort")
async def abort_mission(request: AbortRequest):
    """Abort current mission"""
    return await web_interface.abort_mission(
        request.mission_id,
        request.return_to_home
    )

@app.get("/api/mission/types")
async def get_mission_types():
    """Get available mission types"""
    config_path = os.path.join(
        os.path.dirname(__file__),
        '../../../config/mission_params.yaml'
    )
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return {
        'mission_types': config['mission_control']['mission_types']
    }

# Shutdown handler
@app.on_event("shutdown")
def shutdown_event():
    web_interface.destroy_node()
    rclpy.shutdown()

def main():
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)

if __name__ == "__main__":
    main() 