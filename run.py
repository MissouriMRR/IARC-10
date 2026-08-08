#!/usr/bin/env python3
"""
Main runnable file for the codebase

If running for competition, make sure that the following is set:
- Waypoints in flight/data/competition_data.json
"""

import asyncio
import logging

import flight.flight_log as flight_log
from state_machine.flight_manager import FlightManager
from state_machine.flight_settings import FlightSettings

if __name__ == "__main__":
    # Run multiprocessing function
    try:
        logging.basicConfig(level=logging.INFO)
        settings: FlightSettings = FlightSettings.from_mission_config()

        log_dir = flight_log.configure(settings.current_drone_ID)
        print(f"Logging drone {settings.current_drone_ID} to {log_dir}")

        logging.info("Starting processes")
        flight_manager: FlightManager = FlightManager()
        asyncio.run(flight_manager.run_manager(settings))
    finally:
        logging.info("Done!")
        flight_log.close()
