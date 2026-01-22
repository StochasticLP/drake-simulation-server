
import os
import numpy as np
from pydrake.all import (
    DiagramBuilder, 
    AddMultibodyPlantSceneGraph, 
    Parser, 
    MeshcatVisualizer, 
    Simulator,
    wrap_to,
)
from simulations.drake_simulation_base import DrakeSimulationBase
from simulations.cartpole.switchable_controller import SwitchableController

class CartpoleSimulation(DrakeSimulationBase):
    def build_diagram(self, params, initial_state=None):

        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=0.0)

        parser = Parser(self.plant)
        self.urdf_path = self.get_urdf_path()
        parser.AddModelsFromUrl(f"file://{self.urdf_path}")
        self.configure_plant(self.plant, params)
        self.plant.Finalize()

        self.controllers = self.add_controllers(self.builder, self.plant, params)

        if self.meshcat:
            self.configure_visualization(self.meshcat, params)
            MeshcatVisualizer.AddToBuilder(self.builder, self.scene_graph, self.meshcat)

        self.diagram = self.builder.Build()
        self.simulator = Simulator(self.diagram)
        self.context = self.simulator.get_mutable_context()
        self.plant_context = self.plant.GetMyMutableContextFromRoot(self.context)
        initial_state = initial_state if initial_state is not None else self.get_default_state()
        self.plant.SetPositionsAndVelocities(self.plant_context, initial_state)
        self.paused = False

    def reset_simulation(self):
        #self.simulator.Reset()
        self.context.SetTime(0.0)
        self.plant.SetPositionsAndVelocities(self.plant_context, self.get_default_state())
        self.simulator.Initialize()
        if hasattr(self, 'meshcat'):
            self.meshcat.Flush()

    def advance_simulation(self, target_time: float):
        if not self.paused:
            current_time = self.context.get_time()
            self.simulator.AdvanceTo(current_time + target_time)

    def get_positions_and_velocities(self):
        state = self.plant.GetPositionsAndVelocities(self.plant_context)
        
        # Check simulation bounds
        if (abs(state[0]) > 2.5 or  # Cart position
            abs(state[2]) > 10.0 or  # Cart velocity
            abs(state[3]) > 20.0):   # Pole angular velocity
            self.reset_simulation()
            state = self.plant.GetPositionsAndVelocities(self.plant_context)
        
        # Normalize angle to [-π, π]
        state[1] = wrap_to(state[1], 0., 2 * np.pi) - np.pi
        return state
    def stop_simulation(self):
        
        if self.meshcat:
            self.meshcat.Delete()
            self.meshcat.Flush()
            del self.meshcat

    def send_data(self):
        # Send system state (positions and velocities)
        state = self.get_positions_and_velocities()
        
        return {"state": state.tolist(), "time": self.get_context_time()}

    def handle_ui_input(self, action, value):
        print("UI action:", action, "value:", value)
        controller = self.controllers.get("switchController", None)
        if controller is not None:
            if action == "set_force":
                controller.set_manual_force(float(value))
            elif action == "toggle_controller":
                controller.toggle_controller(value.get("controller"))
            elif action == "play":
                self.paused = False
            elif action == "pause":
                self.paused = True
    """
    Cartpole simulation with multiple control modes:
    - LQR: Linear Quadratic Regulator for balancing
    - Manual: Direct force control via keyboard
    - PID: PID controller (placeholder)
    """
    
    def __init__(self):
        super().__init__()
        self.controller = None
    
    def get_urdf_path(self):
        return os.path.join(os.path.dirname(__file__), "cartpole.urdf")
    
    def get_default_state(self):
        """Return default upright state with small perturbation."""
        # [x, theta, x_dot, theta_dot]
        # Upright is theta = pi
        offset = np.random.normal(np.pi, 0.1)
        return np.array([0, 0, 0, 0]) + np.array([0, offset, 0, 0])
    
    def configure_plant(self, plant, params):
        """Configure plant properties (optional for cartpole)."""
        pass
    
    def add_controllers(self, builder, plant, params):
        """Add controllers based on control_type parameter."""
        control_type = params.get("controllers", ["manual"])
        controllers = {}
        
        switchController = SwitchableController(plant, control_type)
        controller = builder.AddSystem(switchController)
        builder.Connect(plant.get_state_output_port(), controller.get_input_port(0))
        builder.Connect(controller.get_output_port(0), plant.get_actuation_input_port())

        controllers["switchController"] = switchController

        return controllers
    
    def configure_visualization(self, meshcat, params):
        """Configure 2D visualization for cartpole."""
        meshcat.Delete()
        meshcat.Set2dRenderMode(xmin=-2.5, xmax=2.5, ymin=-1.0, ymax=2.5)
    
    def handle_keyboard_down(self, key):
        """Handle keyboard press for manual control."""
        controller = self.controllers.get("switchController", None)
        force_magnitude = 10.0  # Adjust as needed
        if controller is not None:
            if key == "ArrowLeft" or key == "a":
                controller.set_manual_force(-force_magnitude)
            elif key == "ArrowRight" or key == "d":
                controller.set_manual_force(force_magnitude)
    
    def handle_keyboard_up(self, key):
        """Handle keyboard release for manual control."""
        controller = self.controllers.get("switchController", None)

        if controller is not None:
            if key in ["ArrowLeft", "ArrowRight", "a", "d"]:
                controller.set_manual_force(0.0)

# Create a singleton instance that will be imported by simulate_system.py
Simulation = CartpoleSimulation
