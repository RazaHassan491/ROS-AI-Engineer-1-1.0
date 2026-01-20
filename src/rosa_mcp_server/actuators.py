"""
Simulated Actuators for ROSA MCP Server.

This module provides a simple actuator simulation that works without ROS.
It allows testing actuator control via Gemini CLI.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, Optional
from enum import Enum


class ActuatorType(Enum):
    SERVO = "servo"
    MOTOR = "motor"
    LINEAR = "linear"
    GRIPPER = "gripper"


@dataclass
class Actuator:
    """Represents a simulated actuator."""

    name: str
    actuator_type: ActuatorType
    position: float = 0.0  # Current position (degrees for servo, % for others)
    min_position: float = 0.0
    max_position: float = 180.0  # Default for servo
    speed: float = 50.0  # Movement speed (units per second)
    is_moving: bool = False
    target_position: float = 0.0
    last_update: float = field(default_factory=time.time)

    def move_to(self, target: float) -> str:
        """Move actuator to target position."""
        # Clamp target to valid range
        target = max(self.min_position, min(self.max_position, target))

        # Simulate instant movement for simplicity
        old_position = self.position
        self.position = target
        self.target_position = target
        self.last_update = time.time()

        return (
            f"{self.name} ({self.actuator_type.value}): "
            f"Moved from {old_position:.1f} to {self.position:.1f}"
        )

    def get_status(self) -> dict:
        """Get current actuator status."""
        return {
            "name": self.name,
            "type": self.actuator_type.value,
            "position": self.position,
            "min": self.min_position,
            "max": self.max_position,
            "speed": self.speed,
            "is_moving": self.is_moving,
        }


class ActuatorManager:
    """Manages a collection of simulated actuators."""

    def __init__(self):
        self._actuators: Dict[str, Actuator] = {}
        self._initialize_default_actuators()

    def _initialize_default_actuators(self):
        """Create some default actuators for demonstration."""
        self.add_actuator(
            Actuator(
                name="arm_base",
                actuator_type=ActuatorType.SERVO,
                position=90.0,
                min_position=0.0,
                max_position=180.0,
            )
        )
        self.add_actuator(
            Actuator(
                name="arm_shoulder",
                actuator_type=ActuatorType.SERVO,
                position=45.0,
                min_position=0.0,
                max_position=180.0,
            )
        )
        self.add_actuator(
            Actuator(
                name="arm_elbow",
                actuator_type=ActuatorType.SERVO,
                position=90.0,
                min_position=0.0,
                max_position=180.0,
            )
        )
        self.add_actuator(
            Actuator(
                name="gripper",
                actuator_type=ActuatorType.GRIPPER,
                position=0.0,  # 0 = open, 100 = closed
                min_position=0.0,
                max_position=100.0,
            )
        )
        self.add_actuator(
            Actuator(
                name="wheel_left",
                actuator_type=ActuatorType.MOTOR,
                position=0.0,  # Speed percentage
                min_position=-100.0,
                max_position=100.0,
            )
        )
        self.add_actuator(
            Actuator(
                name="wheel_right",
                actuator_type=ActuatorType.MOTOR,
                position=0.0,
                min_position=-100.0,
                max_position=100.0,
            )
        )

    def add_actuator(self, actuator: Actuator):
        """Add an actuator to the manager."""
        self._actuators[actuator.name] = actuator

    def get_actuator(self, name: str) -> Optional[Actuator]:
        """Get an actuator by name."""
        return self._actuators.get(name)

    def list_actuators(self) -> list:
        """List all actuators."""
        return [a.get_status() for a in self._actuators.values()]

    def move_actuator(self, name: str, position: float) -> str:
        """Move an actuator to a position."""
        actuator = self.get_actuator(name)
        if actuator is None:
            return f"Error: Actuator '{name}' not found. Available: {list(self._actuators.keys())}"
        return actuator.move_to(position)

    def get_status(self, name: str) -> dict:
        """Get status of a specific actuator."""
        actuator = self.get_actuator(name)
        if actuator is None:
            return {"error": f"Actuator '{name}' not found"}
        return actuator.get_status()

    def stop_all(self) -> str:
        """Stop all motors (set speed to 0)."""
        stopped = []
        for name, actuator in self._actuators.items():
            if actuator.actuator_type == ActuatorType.MOTOR:
                actuator.position = 0.0
                stopped.append(name)
        return f"Stopped motors: {stopped}" if stopped else "No motors to stop"


# Global instance for the MCP server
actuator_manager = ActuatorManager()
