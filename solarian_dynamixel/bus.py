"""
Dynamixel Actuator Bus

Bus implementation for communicating with Dynamixel servos.
"""

import logging
import time
from typing import Dict, Any

import serial.tools.list_ports

from .lerobot.dynamixel import DynamixelMotorsBus
from .lerobot.dynamixel.dynamixel import NORMALIZED_DATA
from .lerobot.dynamixel.tables import MODEL_NUMBER_TABLE, MODEL_RESOLUTION
from solarian.core.calibration import Motor, MotorCalibration, MotorNormMode

from solarian.core.component import Component
from solarian.core.data import PosData, SpeedData, TorqueEnableData, Data
from solarian.core.actuator_bus import ActuatorBus

logger = logging.getLogger(__name__)

#: Position control. X-series Operating_Mode 3 = joint (single-turn) position.
OPERATING_MODE_POSITION = 3

WRITE_MAP = {
    PosData: "Goal_Position",
    SpeedData: "Goal_Speed",
    TorqueEnableData: "Torque_Enable",
}

READ_MAP = {
    PosData: "Present_Position",
    SpeedData: "Present_Speed",
}

def map_write(data_type: type[Data]):
    if data_type in WRITE_MAP:
        return WRITE_MAP[data_type]
    else:
        raise ValueError(f"Unsupported data type: {data_type}")

def map_read(data_type: type[Data]):
    if data_type in READ_MAP:
        return READ_MAP[data_type]
    else:
        raise ValueError(f"Unsupported data type: {data_type}")

def move_to_mid_encoder(
    bus: DynamixelMotorsBus,
    motor: str,
    resolution: int,
    *,
    settle_s: float = 3.0,
    tol: int = 8,
) -> int:
    """Drive *motor* to mid-encoder, leave torque off, return the reading reached.

    Homing_Offset is cleared and the position limits opened first, so Goal mid
    means *absolute* mid. Only for a free shaft: the goal can be most of a turn
    away from where the shaft sits, so a mounted linkage would hit a hard stop.
    """
    half = (resolution - 1) // 2
    bus.disable_torque([motor])
    bus.write("Homing_Offset", motor, 0, normalize=False)
    bus.write("Min_Position_Limit", motor, 0, normalize=False)
    bus.write("Max_Position_Limit", motor, resolution - 1, normalize=False)
    bus.write("Operating_Mode", motor, OPERATING_MODE_POSITION, normalize=False)

    bus.enable_torque([motor])
    try:
        bus.write("Goal_Position", motor, half, normalize=False)
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            if abs(int(bus.read("Present_Position", motor, normalize=False)) - half) <= tol:
                break
            time.sleep(0.05)
    finally:
        bus.disable_torque([motor])

    return int(bus.read("Present_Position", motor, normalize=False))


def map_motor_mode(mode: str):
    if mode == "range_0_100":
        return MotorNormMode.RANGE_0_100
    elif mode == "range_m100_100":
        return MotorNormMode.RANGE_M100_100
    elif mode == "degrees":
        return MotorNormMode.DEGREES
    else:
        raise ValueError(f"Unknown motor mode: {mode}")


class DynamixelBus(ActuatorBus):
    """
    Actuator bus for Dynamixel servos.
    
    Handles communication with Dynamixel motors via serial port.
    Automatically used when DynamixelServo actuators are defined in a module.
    """

    def __init__(self, port: str, components: Dict[str, Component], calibration: Dict[str, MotorCalibration]):
        super().__init__(port, components, calibration)
        
        self.bus = DynamixelMotorsBus(
            port=port,
            motors={
                name: Motor(component.cfg.id, component.cfg.model, map_motor_mode(component.cfg.mode)) 
                for name, component in components.items()
            },
            calibration=calibration
        )

    def is_connected(self) -> bool:
        return self.bus.is_connected
    
    def connect(self):
        self.bus.connect()
        
    def disconnect(self):
        self.bus.disconnect()

    def read(self, data_type: type[Data], component_names: list[str], normalize: bool = True):
        # Retry a few times: the XL330 chain occasionally drops a single status
        # packet at high baud, and without retries one glitch kills the whole
        # control loop.
        return self.bus.sync_read(
            map_read(data_type), motors=component_names, normalize=normalize, num_retry=3
        )

    def write(self, values: Dict[str, Data], normalize: bool = True):
        # group values by class type, extracting raw values from Data objects
        values_by_class = {}
        for name, value in values.items():
            if value.__class__ not in values_by_class:
                values_by_class[value.__class__] = {}
            values_by_class[value.__class__][name] = value.value
        for class_type, class_values in values_by_class.items():
            data_name = map_write(class_type)
            if not normalize or data_name not in NORMALIZED_DATA:
                class_values = {k: int(v) for k, v in class_values.items()}
            self.bus.sync_write(data_name, values=class_values, normalize=normalize)

    def counts_per_revolution(self, name: str) -> int | None:
        """Encoder resolution (counts/rev) for motor ``name`` from its model.

        XL330/XM430 are 4096 counts/rev per :data:`MODEL_RESOLUTION`.
        """
        try:
            model = self.bus.motors[name].model
            return int(self.bus.model_resolution_table[model])
        except Exception:
            return None

    def center_offset_in_hardware(self, name: str) -> bool:
        del name
        return True

    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated

    def reset_calibration(self, component_names: list[str]):
        self.bus.reset_calibration(component_names)

    def read_calibration(self) -> dict[str, MotorCalibration]:
        return self.bus.read_calibration()
        
    def write_calibration(self, calibration_dict: dict[str, MotorCalibration]) -> dict[str, MotorCalibration]:
        # bound min and max to range of motion
        for name, calibration in calibration_dict.items():
            calibration.range_min = max(calibration.range_min, 0)
            calibration.range_max = min(calibration.range_max, 4095)
        self.bus.write_calibration(calibration_dict)
        return calibration_dict

    def configure(self, params: dict[str, int], component_names: list[str]):
        for name in component_names:
            for param_name, value in params.items():
                self.bus.write(param_name, name, value, normalize=False)

    def enable_torque(self, component_names: list[str]):
        self.bus.enable_torque(component_names)

    def disable_torque(self, component_names: list[str]):
        self.bus.disable_torque(component_names)

    def start_calibration_mode(self):
        """Sets the actuators where the arm can be moved manually freely"""
        self.bus.disable_torque(list(self.components.keys()))

    def end_calibration_mode(self):
        pass
        
    def set_half_turn_homings(self, component_names: list[str]) -> Dict[str, int]:
        """Set half turn homings for the actuators. Read present position and set/store"""
        return self.bus.set_half_turn_homings(component_names)

    def drive_to_encoder_center(self, component_names: list[str]) -> Dict[str, int]:
        reached: Dict[str, int] = {}
        for name in component_names:
            resolution = self.counts_per_revolution(name) or 4096
            present = move_to_mid_encoder(self.bus, name, resolution)
            reached[name] = present
            half = (resolution - 1) // 2
            if abs(present - half) > 8:
                logger.warning(
                    "%s: reached %d, wanted ~%d. Shaft may be blocked.", name, present, half
                )
        return reached

    # -- provisioning (no declared components: servos are addressed by ID) --
    @classmethod
    def available_ports(cls) -> list[str]:
        return [p.device for p in serial.tools.list_ports.comports()]

    @classmethod
    def scan_port(cls, port: str) -> list[dict]:
        model_by_number = {nb: name for name, nb in MODEL_NUMBER_TABLE.items()}
        bus = DynamixelMotorsBus(port, {})
        bus._connect(handshake=False)
        try:
            for baudrate in bus.available_baudrates:
                bus.set_baudrate(baudrate)
                ids_models = bus.broadcast_ping() or {}
                if not ids_models:
                    continue
                # Motors always share one baud-rate, so the first hit is the bus.
                return [
                    {
                        "id": motor_id,
                        "model": model_by_number.get(model_nb, f"unknown({model_nb})"),
                        "baudrate": baudrate,
                    }
                    for motor_id, model_nb in sorted(ids_models.items())
                ]
        finally:
            bus.port_handler.closePort()
        return []

    @classmethod
    def provision_servo(cls, port: str, found: dict) -> None:
        model = str(found["model"])
        if model not in MODEL_RESOLUTION:
            raise ValueError(f"Unknown Dynamixel model {model!r}; cannot provision it.")

        current_id = int(found["id"])
        raw = input(f"New ID for the servo at ID {current_id} (Enter to keep it): ").strip()
        target_id = int(raw) if raw.isdigit() else current_id

        # setup_motor writes the ID declared here, so the motor is named at its
        # target ID and found at its current one.
        bus = DynamixelMotorsBus(
            port, {"servo": Motor(target_id, model, MotorNormMode.RANGE_M100_100)}
        )
        bus._connect(handshake=False)
        try:
            bus.setup_motor(
                "servo", initial_baudrate=int(found["baudrate"]), initial_id=current_id
            )
            print(f"  Servo is at ID {target_id}, baud-rate {bus.default_baudrate}.")

            resolution = MODEL_RESOLUTION[model]
            half = (resolution - 1) // 2
            answer = input(
                f"Drive shaft to mid-encoder ({half}) for horn mounting?\n"
                "  WARNING: Only if the output is free to rotate (no linkage / hard "
                "stops). [Y/n] "
            ).strip().lower()
            if answer in ("", "y", "yes"):
                present = move_to_mid_encoder(bus, "servo", resolution)
                print(f"  Present={present} (wanted ~{half}).")
        finally:
            bus.port_handler.closePort()

    def setup_component(self, component_name: str, configs: Dict[str, Any]):
        """Setup config for a component"""
        print("Setting the following configs for component:", component_name)
        print(configs)
        input("Connect the servo to the controller, make sure only the servo to be configured is connected. Press Enter when ready.")
        self.bus.setup_motor(component_name, initial_baudrate=configs["baudrate"], initial_id=configs["id"])
        print("Configuration completed.")
