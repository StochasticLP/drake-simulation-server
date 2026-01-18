import os
import asyncio
import time
import numpy as np
import importlib
import logging
from simulations.drake_simulation_base import DrakeSimulationBase

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

def import_sim(sim_name: str) -> DrakeSimulationBase:
    """Import a simulation module.

    Args:
        sim_name: Name of the simulation module to import

    Returns:
        DrakeSimulationBase: Instance of the simulation class, which inherits from DrakeSimulationBase
        None: If module could not be loaded or doesn't have required class
    """
    try:
        module = importlib.import_module(f'simulations.{sim_name}.{sim_name}', package='Simulation')
        if hasattr(module, 'Simulation'):
            return module.Simulation()
        else:
            raise AttributeError(f"Module {sim_name} has no 'Simulation' class")
    except (ModuleNotFoundError, AttributeError) as e:
        logging.error(f"Error loading simulation {sim_name}: {e}. Falling back to empty simulation.")
        empty_module = importlib.import_module('simulations.drake_simulation_base', package='Simulation')
        return empty_module.Simulation()

def get_available_simulations():
    sim_dir = 'simulations'
    files = os.listdir(sim_dir)
    return [f[:-3] for f in files if f.endswith('.py') and f != '__init__.py' and f != 'base_simulation.py' and f != 'simulate_system.py']


class Simulation:
    def __init__(self, sid: str, params: dict | None = None):
        self.sid: str = sid
        self.params: dict = params or {}
        self.url: str = ""  # Meshcat URL
        self.running: bool = False
        self.input_queue: asyncio.Queue = asyncio.Queue()
        self.output_data: dict = {}
        self.task: asyncio.Task | None = None
        self.ready_event: asyncio.Event = asyncio.Event()  # For awaiting readiness
        self.state: np.ndarray | None = None  # Drake uses numpy arrays for states
        # Import the specific simulation
        sim_name = self.params.get("sim_type", "")
        self.sim_instance: DrakeSimulationBase = import_sim(sim_name)

    def build_diagram(self):
        """Build or rebuild the Drake diagram (MAIN THREAD ONLY)."""
        logging.info(f"Building diagram for simulation {self.sid}")
        if self.state is None:
            self.state = self.sim_instance.get_default_state()
        try:
            self.sim_instance.build_diagram(self.params, self.state)
            self.sim_instance.initialize_simulation()
            self.url = self.sim_instance.get_meshcat_url()
        except Exception as e:
            logging.error(f"Error building simulation {self.sid}: {e}")
            raise

    def handle_input(self, input_data):
        """Handle input events from the client (MAIN THREAD ONLY)."""
        try:
            input_type = input_data.get("type")
            if input_type == "keyboard_down":
                key = input_data.get("key")
                self.sim_instance.handle_keyboard_down(key)
            elif input_type == "keyboard_up":
                key = input_data.get("key")
                self.sim_instance.handle_keyboard_up(key)
            elif input_type == "ui_input":
                action = input_data.get("action")
                value = input_data.get("value")
                self.sim_instance.handle_ui_input(action, value)
            elif input_type == "update_param":
                self.params.update(input_data.get("params", {}))
                self.build_diagram()
            elif input_type == "reset":
                try:
                    self.sim_instance.reset_simulation()
                    self.state = self.sim_instance.get_positions_and_velocities()
                    self.output_data = self.sim_instance.send_data()
                except Exception as e:
                    logging.error(f"Error during reset for {self.sid}: {e}")
                    self.build_diagram()  # Attempt recovery
        except Exception as e:
            logging.error(f"Error handling input for {self.sid}: {e}")


    async def run_loop(self):
        """Main simulation loop - runs entirely on main thread."""
        # Build diagram on main thread (Meshcat initialization)
        self.build_diagram()
        self.sim_instance.set_target_realtime_rate(1.0)
        self.ready_event.set()  # Signal ready (url and sim are set)
        logging.info(f"Starting simulation loop for {self.sid}")
        self.running = True
        step_size = 1 / 60.0 # 60Hz update rate

        while self.running:
            start = time.time()
            try:
                # Process inputs on main thread
                while not self.input_queue.empty():
                    input_data = await self.input_queue.get()
                    self.handle_input(input_data)

                current_time = self.sim_instance.get_context_time()
                
                self.sim_instance.advance_simulation(current_time + step_size)
                self.state = self.sim_instance.get_positions_and_velocities()
                self.output_data = self.sim_instance.send_data()

            except Exception as e:
                logging.error(f"Error in simulation loop for {self.sid}: {e}")
                self.stop()
            
            elapsed = time.time() - start
            await asyncio.sleep(max(0.001, step_size - elapsed))

    async def start(self):
        """Start the simulation as an async task."""
        if not self.running:
            self.task = asyncio.create_task(self.run_loop())
            logging.info(f"Started simulation task for {self.sid}")

    def stop(self):
        """Stop the simulation."""
        if self.running:
            self.running = False
            if self.task:
                self.task.cancel()
            self.sim_instance.stop_simulation()
            logging.info(f"Simulation {self.sid} stopped")
