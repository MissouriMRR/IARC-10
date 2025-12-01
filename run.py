#!/usr/bin/env python3
"""
Main runnable file for the codebase

If running for competition, make sure that the following is set:
- Waypoints in flight/data/competition_data.json
"""

import asyncio
import logging
from state_machine.flight_manager import FlightManager
from state_machine.flight_settings import FlightSettings


if __name__ == "__main__":
    # Run multiprocessing function
    try:
        logging.basicConfig(level=logging.INFO)
        logging.info("Starting processes")
        flight_manager: FlightManager = FlightManager()
        asyncio.run(flight_manager.run_manager(FlightSettings.from_mission_config()))
    finally:
        logging.info("Done!")
