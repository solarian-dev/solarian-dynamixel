"""
Dynamixel Servo Actuator

Actuator class for Dynamixel motors (e.g., XL330, XM430, etc.).
"""

from solarian.core.servo import Servo
from solarian.core.data import PosData, SpeedData

from solarian_dynamixel.bus import DynamixelBus


class DynamixelServo(Servo):
    """
    Servo actuator for Dynamixel motors (e.g., XL330, XM430, etc.).
    
    Retrieve this class from the plugin registry:
    
        from solarian.plugins import actuators
        DynamixelServo = actuators.get("dynamixel")
        
        class MyArmModule(Module):
            shoulder: DynamixelServo = DynamixelServo(id=1, model="xl330-m288")
            elbow: DynamixelServo = DynamixelServo(id=2, model="xl330-m288")
    
    The corresponding DynamixelBus will be automatically used for communication.
    """
    
    bus_class = DynamixelBus

    pos = PosData()
    speed = SpeedData()