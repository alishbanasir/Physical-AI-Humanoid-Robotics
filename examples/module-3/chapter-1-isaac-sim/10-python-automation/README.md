# Example 10: Python Automation

## Overview

Advanced automation demonstrating parameter sweeps, multi-robot spawning, and ML data collection - showcasing omni.isaac.core API power.

## Usage

```bash
# Parameter sweep (test different PhysX configs)
./python.sh main.py --mode sweep --robot /path/to/humanoid.usd

# Multi-robot spawning (5 robots in array)
./python.sh main.py --mode multi-robot --robot /path/to/humanoid.usd --num-robots 5

# Data collection (100 trajectory samples for ML)
./python.sh main.py --mode data-collection --robot /path/to/humanoid.usd --num-samples 100
```

## See Also

- Module 3, Chapter 1, Section 1-6: Python automation API comprehensive guide
