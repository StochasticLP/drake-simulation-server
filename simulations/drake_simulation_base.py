from abc import ABC, abstractmethod
import numpy as np
from numpy.typing import NDArray
import logging
from pydrake.all import (
    Meshcat,
    DiagramBuilder,
    Simulator,
    AddMultibodyPlantSceneGraph,
    MeshcatVisualizer,
)

# Configure logging
logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")

class DrakeSimulationBase:
    """
    Base class for Drake simulations.
    
    Each specific simulation (cartpole, pendulum, etc.) should inherit from this
    and implement the abstract methods. The simulate_system.py will call these methods.
    """
    
    def __init__(self):
        self.meshcat: Meshcat = Meshcat(port=0)  # Auto-assign port
        self.visualizer = None  # MeshcatVisualizer, set in build_diagram
        self.builder = None
        self.plant = None
        self.scene_graph = None
        self.diagram = None
        self.simulator = None
        
    def get_default_state(self) -> NDArray[np.float64]:
        """Return the default initial state as a numpy array."""
        return np.array([])

    def build_diagram(self, params, initial_state=None):
        """Build the Drake diagram with Meshcat visualization."""
        logging.info("Building diagram for simulation")
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.0)
        # Subclasses should add their specific plant components here
        self.plant.Finalize()
        # Add MeshcatVisualizer
        self.visualizer = MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)
        self.diagram = self.builder.Build()
        self.simulator = Simulator(self.diagram)
        self.context = self.simulator.get_mutable_context()
        self.configure_visualization(self.meshcat, params)

    def get_meshcat_url(self) -> str:
        """Return the Meshcat URL."""
        return self.meshcat.web_url()

    def initialize_simulation(self):
        """Initialize the simulator."""
        if self.simulator is not None:
            try:
                self.simulator.Initialize()
                logging.info("Simulator initialized")
            except Exception as e:
                logging.error(f"Error initializing simulator: {e}")
                raise

    def reset_simulation(self):
        """Reset the simulation to initial state."""
        pass

    def set_target_realtime_rate(self, rate: float):
        if self.simulator is not None:
            self.simulator.set_target_realtime_rate(rate)

    def get_context_time(self) -> float:
        """Get the current simulation time."""
        return 0.0

    def advance_simulation(self, target_time: float):
        """Advance the simulation to the target time."""
        pass
    
    def get_positions_and_velocities(self) -> NDArray[np.float64]:
        """Get the positions and velocities of the plant/system."""
        return np.array([])

    def stop_simulation(self):
        """Stop/cleanup the simulation."""
        pass

    def send_data(self):
        """Send arbitrary data to the client (to be implemented in subclass). Should return a dict."""
        return {}
    
    def configure_visualization(self, meshcat, params):
        """
        Configure meshcat visualization (optional override).
        Default behavior clears meshcat and sets 2D mode if requested.
        
        Args:
            meshcat: Meshcat instance
            params: Simulation parameters dict
        """
        meshcat.Delete()
        if params.get("2d_mode", False):
            xmin = params.get("xmin", -2.5)
            xmax = params.get("xmax", 2.5)
            ymin = params.get("ymin", -1.0)
            ymax = params.get("ymax", 2.5)
            meshcat.Set2dRenderMode(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax)
    
    def handle_keyboard_down(self, key):
        """
        Handle keyboard down events (optional override).
        
        Args:
            key: The key that was pressed
        """
        pass
    
    def handle_keyboard_up(self, key):
        """
        Handle keyboard up events (optional override).
        
        Args:
            key: The key that was released
        """
        pass
    
    def handle_ui_input(self, action, value):
        """
        Handle UI input events (optional override).
        
        Args:
            action: The action type
            value: The value for the action
        """
        pass

Simulation = DrakeSimulationBase  # Alias
