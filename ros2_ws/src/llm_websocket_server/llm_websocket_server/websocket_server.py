#!/usr/bin/env python3
"""
WebSocket server for LLM robot control integration.
Combines the robot_server.py WebSocket framework with ROS2 functionality
from openai_bridge for real robot control.
"""

import asyncio
import json
import logging
import websockets
import os
import subprocess
import psutil
import platform
from datetime import datetime
from functools import partial
from typing import Union
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.client import Client
from geometry_msgs.msg import Twist
from controller_manager_msgs.srv import SwitchController

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
logger = logging.getLogger(__name__)

# Available controllers and mapping from openai_bridge
AVAILABLE_CONTROLLERS = {"neural_controller", "neural_controller_three_legged"}
CONTROLLER_NAME_MAP = {
    "4-legged": "neural_controller",
    "3-legged": "neural_controller_three_legged",
}

PORT = 8008


class RobotState:
    def __init__(self):
        self.is_active = False
        self.last_command = None
        self.command_count = 0
        self.current_gait = "4-legged"
        self.command_queue = deque()  # Queue for commands
        self.queue_processor_task = None  # Task for processing queue
        self.is_processing_queue = False


class WebSocketRobotServer(Node):
    def __init__(self):
        super().__init__("llm_websocket_server")

        # Initialize robot state
        self.robot_state = RobotState()

        # Create ROS2 publishers and clients
        self.twist_pub = self.create_publisher(Twist, "/llm_cmd_vel", 10)
        self.switch_controller_client = self.create_client(SwitchController, "/controller_manager/switch_controller")

        # Wait for controller manager service to be available
        self.get_logger().info("Waiting for controller manager service...")
        self.switch_controller_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info("Controller manager service available")

    async def handle_client(self, websocket):
        """Handle incoming WebSocket connections and messages."""
        client_addr = websocket.remote_address
        logger.info(f"🔌 New client connected: {client_addr}")

        # Start the queue processor if not already running
        if self.robot_state.queue_processor_task is None or self.robot_state.queue_processor_task.done():
            self.robot_state.queue_processor_task = asyncio.create_task(self.process_command_queue(websocket))

        try:
            async for message in websocket:
                try:
                    # Parse incoming message
                    data = json.loads(message)
                    command_name = data.get("name", "unknown")
                    command_args = data.get("args", {})
                    request_id = data.get("request_id")

                    self.robot_state.command_count += 1
                    self.robot_state.last_command = command_name

                    logger.info(f"📨 Received command from {client_addr}: {command_name}")

                    # Add command to queue
                    self.robot_state.command_queue.append({
                        "name": command_name,
                        "args": command_args,
                        "request_id": request_id,
                        "websocket": websocket
                    })

                    # Send immediate acknowledgment for queued command
                    ack_response = self.add_request_id({
                        "status": "queued",
                        "message": f"Command '{command_name}' queued for execution",
                        "queue_position": len(self.robot_state.command_queue),
                        "timestamp": datetime.now().isoformat()
                    }, request_id)
                    await websocket.send(json.dumps(ack_response))

                except json.JSONDecodeError:
                    error_response = {
                        "status": "error",
                        "message": "Invalid JSON format",
                        "timestamp": datetime.now().isoformat(),
                    }
                    await websocket.send(json.dumps(error_response))
                    logger.error(f"❌ Invalid JSON received from {client_addr}: {message}")

                except Exception as e:
                    error_response = {
                        "status": "error",
                        "message": f"Server error: {str(e)}",
                        "timestamp": datetime.now().isoformat(),
                    }
                    await websocket.send(json.dumps(error_response))
                    logger.error(f"❌ Error handling message from {client_addr}: {e}")

        except websockets.exceptions.ConnectionClosed:
            logger.info(f"🔌 Client disconnected: {client_addr}")
        except Exception as e:
            logger.error(f"❌ Connection error with {client_addr}: {e}")

    async def process_command_queue(self, websocket):
        """Process commands from the queue sequentially."""
        while True:
            try:
                if self.robot_state.command_queue and not self.robot_state.is_processing_queue:
                    self.robot_state.is_processing_queue = True
                    command_data = self.robot_state.command_queue.popleft()
                    
                    # Execute the command
                    response = await self.handle_command(
                        command_data["name"],
                        command_data["args"],
                        command_data["request_id"]
                    )
                    
                    # Send response
                    try:
                        if command_data["websocket"]:
                            await command_data["websocket"].send(json.dumps(response))
                            logger.info(f"📤 Executed queued command: {command_data['name']}")
                    except Exception as e:
                        logger.warning(f"⚠️ Could not send response for {command_data['name']}: {e}")
                    
                    self.robot_state.is_processing_queue = False
                else:
                    await asyncio.sleep(0.1)  # Small delay when queue is empty
            except Exception as e:
                logger.error(f"❌ Error processing command queue: {e}")
                self.robot_state.is_processing_queue = False
                await asyncio.sleep(0.1)

    def add_request_id(self, response: dict, request_id: str = None) -> dict:
        """Add request_id to response if provided."""
        if request_id:
            response["request_id"] = request_id
        return response

    async def handle_command(self, command_name: str, command_args: dict, request_id: str = None) -> dict:
        """Handle specific robot commands and return appropriate responses."""

        if command_name == "activate":
            success, message = await self.activate_robot()
            return self.add_request_id({
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }, request_id)

        elif command_name == "deactivate":
            success, message = await self.deactivate_robot()
            return self.add_request_id({
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }, request_id)

        elif command_name == "move":
            vx = command_args.get("vx", 0.0)
            vy = command_args.get("vy", 0.0)
            wz = command_args.get("wz", 0.0)

            success, message, warnings = await self.move_robot(vx, vy, wz)

            response = {
                "status": "success" if success else "error",
                "message": message,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "velocities": {"vx": vx, "vy": vy, "wz": wz},
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }

            if warnings:
                response["warnings"] = warnings
                response["constraints"] = {"max_vx": 0.75, "max_vy": 0.5, "max_wz": 2.0, "min_movement_threshold": 0.2}

            return self.add_request_id(response, request_id)

        elif command_name == "get_battery":
            battery_percentage, battery_voltage = await self.get_battery_info()

            battery_status = "normal"
            if battery_percentage <= 15:
                battery_status = "critical"
            elif battery_percentage <= 30:
                battery_status = "low"

            logger.info(f"🔋 Battery level: {battery_percentage}% ({battery_voltage}V) ({battery_status})")
            return self.add_request_id({
                "status": "success",
                "message": f"Battery at {battery_percentage}% ({battery_voltage}V)",
                "battery_percentage": battery_percentage,
                "battery_voltage": battery_voltage,
                "battery_status": battery_status,
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }, request_id)

        elif command_name == "get_cpu_usage":
            try:
                # Get CPU usage (instant measurement, no blocking)
                cpu_usage = psutil.cpu_percent()
                system_info = {
                    "platform": platform.system(),
                    "cpu_count": psutil.cpu_count(),
                    "cpu_count_logical": psutil.cpu_count(logical=True)
                }
                
                logger.info(f"💻 CPU usage: {cpu_usage}%")
                return self.add_request_id({
                    "status": "success",
                    "message": f"CPU usage at {cpu_usage}%",
                    "cpu_usage": round(cpu_usage, 1),
                    "system_info": system_info,
                    "timestamp": datetime.now().isoformat(),
                    "command_count": self.robot_state.command_count
                }, request_id)
            except Exception as e:
                logger.error(f"❌ Failed to get CPU usage: {e}")
                return self.add_request_id({
                    "status": "error",
                    "message": f"Failed to get CPU usage: {str(e)}",
                    "timestamp": datetime.now().isoformat(),
                    "command_count": self.robot_state.command_count
                }, request_id)

        elif command_name == "wait":
            duration = command_args.get("duration", 1.0)
            duration = min(max(duration, 0.1), 60.0)  # Clamp between 0.1 and 60 seconds
            
            logger.info(f"⏱️ Waiting for {duration} seconds")
            await asyncio.sleep(duration)
            
            return self.add_request_id({
                "status": "success",
                "message": f"Waited for {duration} seconds",
                "duration": duration,
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }, request_id)

        elif command_name == "status":
            battery_percentage, battery_voltage = await self.get_battery_info()
            return self.add_request_id({
                "status": "success",
                "message": "Status retrieved",
                "robot_state": "active" if self.robot_state.is_active else "inactive",
                "battery_percentage": battery_percentage,
                "battery_voltage": battery_voltage,
                "current_gait": self.robot_state.current_gait,
                "last_command": self.robot_state.last_command,
                "command_count": self.robot_state.command_count,
                "queue_length": len(self.robot_state.command_queue),
                "timestamp": datetime.now().isoformat(),
            }, request_id)

        else:
            return self.add_request_id({
                "status": "error",
                "message": f"Unknown command: {command_name}",
                "available_commands": ["activate", "deactivate", "move", "wait", "get_battery", "get_cpu_usage", "status"],
                "timestamp": datetime.now().isoformat(),
                "command_count": self.robot_state.command_count,
            }, request_id)

    async def activate_robot(self) -> tuple[bool, str]:
        """Activate the robot by switching to the default controller."""
        try:
            # Use the same logic as openai_bridge activate function
            req = SwitchController.Request()
            controller_name = CONTROLLER_NAME_MAP[self.robot_state.current_gait]

            req.activate_controllers = [controller_name]
            req.deactivate_controllers = list(AVAILABLE_CONTROLLERS - {controller_name})
            req.strictness = 1

            # Call the service synchronously in the async context
            future = self.switch_controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done() and future.result().ok:
                self.robot_state.is_active = True
                logger.info("🤖 Robot ACTIVATED")
                return True, f"Robot activated successfully with {self.robot_state.current_gait} gait"
            else:
                logger.error("❌ Failed to activate robot")
                return False, "Failed to activate robot - controller switch failed"

        except Exception as e:
            logger.error(f"❌ Error activating robot: {e}")
            return False, f"Failed to activate robot: {str(e)}"

    async def deactivate_robot(self) -> tuple[bool, str]:
        """Deactivate the robot by stopping all controllers."""
        try:
            req = SwitchController.Request()
            req.activate_controllers = []
            req.deactivate_controllers = list(AVAILABLE_CONTROLLERS)
            req.strictness = 1

            future = self.switch_controller_client.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done() and future.result().ok:
                self.robot_state.is_active = False
                logger.info("🤖 Robot DEACTIVATED")
                return True, "Robot deactivated successfully"
            else:
                logger.error("❌ Failed to deactivate robot")
                return False, "Failed to deactivate robot - controller switch failed"

        except Exception as e:
            logger.error(f"❌ Error deactivating robot: {e}")
            return False, f"Failed to deactivate robot: {str(e)}"

    async def move_robot(self, vx: float, vy: float, wz: float) -> tuple[bool, str, list]:
        """Move the robot with the specified velocities."""
        try:
            warnings = []

            # Validate velocity constraints (same as openai_bridge)
            if abs(vx) > 0.75:
                vx = 0.75 if vx > 0 else -0.75
                warnings.append(f"vx clamped to {vx} (max: ±0.75)")

            if abs(vy) > 0.5:
                vy = 0.5 if vy > 0 else -0.5
                warnings.append(f"vy clamped to {vy} (max: ±0.5)")

            if abs(wz) > 2.0:
                wz = 2.0 if wz > 0 else -2.0
                warnings.append(f"wz clamped to {wz} (max: ±2.0)")

            # Check minimum movement thresholds
            if 0 < abs(vx) < 0.2:
                warnings.append(f"vx={vx} is below movement threshold (0.2), robot may not move")
            if 0 < abs(vy) < 0.2:
                warnings.append(f"vy={vy} is below movement threshold (0.2), robot may not move")

            # Create and publish Twist message
            twist = Twist()
            twist.linear.x = float(vx)
            twist.linear.y = float(vy)
            twist.angular.z = float(wz)

            self.twist_pub.publish(twist)

            message = f"Robot moving with velocities vx={vx}, vy={vy}, wz={wz}"
            if warnings:
                message += f". Warnings: {'; '.join(warnings)}"

            logger.info(f"🤖 Robot MOVING - vx: {vx}, vy: {vy}, wz: {wz}")
            if warnings:
                logger.warning(f"🤖 Movement warnings: {'; '.join(warnings)}")

            return True, message, warnings

        except Exception as e:
            logger.error(f"❌ Error moving robot: {e}")
            return False, f"Failed to move robot: {str(e)}", []

    async def get_battery_info(self) -> tuple[int, float]:
        """Get battery percentage and voltage using the existing battery check script."""
        try:
            # Use the same battery check script as openai_bridge
            result = subprocess.run(
                ["python3", "/home/pi/quadrapetv3-monorepo/robot/utils/check_batt_voltage.py"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )

            if result.returncode == 0:
                # Parse the output: "Battery:	85%	16.5V"
                output = result.stdout.strip()
                if "Battery:" in output:
                    parts = output.split("\t")
                    if len(parts) >= 3:
                        percentage_str = parts[1].replace("%", "")
                        voltage_str = parts[2].replace("V", "")
                        percentage = int(percentage_str)
                        voltage = float(voltage_str)
                        return percentage, voltage

            # Fallback to mock values if script fails
            logger.warning("🔋 Battery script failed, using fallback values")
            return 0, 0

        except Exception as e:
            logger.error(f"❌ Error getting battery info: {e}")
            return 0, 0  # Fallback values

    async def start_websocket_server(self):
        """Start the WebSocket server."""
        host = "localhost"
        port = PORT

        logger.info(f"🚀 Starting LLM WebSocket server on {host}:{port}")
        logger.info(f"🤖 Initial robot state: {'ACTIVE' if self.robot_state.is_active else 'INACTIVE'}")
        logger.info("📡 Waiting for connections...")

        try:
            async with websockets.serve(self.handle_client, host, port):
                await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            logger.info("🛑 Server stopped by user")
        except Exception as e:
            logger.error(f"❌ Server error: {e}")


def main():
    """Main entry point."""
    rclpy.init()

    try:
        server_node = WebSocketRobotServer()

        # Run the WebSocket server in an async event loop
        asyncio.run(server_node.start_websocket_server())

    except KeyboardInterrupt:
        logger.info("🛑 Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
