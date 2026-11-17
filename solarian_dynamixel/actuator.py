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
            shoulder = DynamixelServo(model="xl330-m288")
            elbow = DynamixelServo(model="xl330-m288")
    
    The corresponding DynamixelBus will be automatically used for communication.
    """
    
    bus_class = DynamixelBus
    default_mode = "range_m100_100"

    pos = PosData()
    speed = SpeedData()