# Solarian-Dynamixel

Dynamixel servo actuator and bus plugin for the Solarian robotics framework.

## Installation

```bash
pip install solarian-dynamixel
```

Or install from source:

```bash
cd solarian-dynamixel
pip install -e .
```

## Usage

After installation, the plugin is automatically discovered by Solarian:

```python
from solarian.core.module import Module
from solarian.plugins import actuators

# Get the Dynamixel actuator class
DynamixelServo = actuators.get("dynamixel")

class MyArmModule(Module):
    shoulder: DynamixelServo = DynamixelServo(id=1, model="xl330-m077")
    elbow: DynamixelServo = DynamixelServo(id=2, model="xl330-m077")
```

The `DynamixelBus` is automatically used for communication - no configuration needed!

## Supported Models

- XL330 series
- XM430 series
- XH series
- Other Dynamixel servo models

## Requirements

- Solarian framework (>=0.1.0)
- dynamixel-sdk (>=3.7.31)

## Repository

This plugin should be maintained in its own repository:
- GitHub: `https://github.com/yourorg/solarian-dynamixel`
- Can be published to PyPI for easy installation

## License

[Apache License 2.0](https://github.com/solarian-dev/solarian-dynamixel/blob/main/LICENSE). Copyright 2026 PrimaryKey Inc. The driver in `solarian_dynamixel/lerobot/` is derived from [LeRobot](https://github.com/huggingface/lerobot) (Apache 2.0); see [NOTICE](https://github.com/solarian-dev/solarian-dynamixel/blob/main/NOTICE).

Solarian and the Solarian logo are trademarks of PrimaryKey Inc. See the [trademark policy](https://github.com/solarian-dev/solarian/blob/main/TRADEMARKS.md).
