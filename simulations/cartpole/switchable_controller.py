import os
import numpy as np
from pydrake.all import (
    LinearQuadraticRegulator,
    LeafSystem,
    BasicVector,
    DiagramBuilder, 
    AddMultibodyPlantSceneGraph, 
    Parser, 
    Simulator,
    wrap_to,
    FittedValueIteration,
    DynamicProgrammingOptions,
    BarycentricMeshSystem,
)

class SwitchableController(LeafSystem):
    """
    A controller that can switch between multiple control modes at runtime.
    
    Modes:
    - "manual": Direct force control
    - "lqr": LQR balancing controller
    - "pid": PID controller (placeholder)
    - "zero": No control (zero force)
    - "fvi": Fitted Value Iteration controller
    """
    
    def __init__(self, plant, controllers=[]):
        LeafSystem.__init__(self)

        self.debug_counter = 0
        
        self.controller_active = {mode: (mode in controllers) for mode in ["manual", "lqr", "pid", "fvi"]}

        self.plant = plant
        self.manual_force = 0.0

        self.lqr_controller = None
        if "lqr" in controllers:
            self.lqr_controller = self._design_lqr()

        self.fvi_controller: tuple[BarycentricMeshSystem, np.ndarray] | None = None
        if "fvi" in controllers:
            print("Designing FVI controller...")
            # Try to load precomputed FVI policy
            policy_file = os.path.join(os.path.dirname(__file__), "fvi_policy.npz")
            if os.path.exists(policy_file):
                try:
                    self.fvi_controller = self._load_fvi_policy(policy_file)
                    print(f"Loaded precomputed FVI policy from {policy_file}")
                except Exception as e:
                    print(f"Failed to load FVI policy: {e}. Computing new policy...")
                    self._design_fvi(save_policy=True)
            else:
                print(f"No precomputed FVI policy found at {policy_file}. Computing new policy...")
                self._design_fvi(save_policy=True)
        
        if "pid" in controllers:
            # PID gains
            self.kp = 1000.0
            self.ki = 0.0
            self.kd = 1.0
            self.integral_error = 0.0
    
        # Declare input port for state
        self.DeclareVectorInputPort("state", BasicVector(4))
        
        # Declare output port for control
        self.DeclareVectorOutputPort(
            "control",
            BasicVector(1),
            self.CalcControl
        )
        
        # Declare periodic update for integral term
        self.DeclarePeriodicDiscreteUpdateEvent(
            period_sec=0.01,
            offset_sec=0.0,
            update=self.UpdateIntegral
        )
        
        # Discrete state for integral term
        self.DeclareDiscreteState(1)

    def _design_lqr(self):
        """Design LQR controller for upright equilibrium."""
        context = self.plant.CreateDefaultContext()
        self.plant.get_actuation_input_port().FixValue(context, [0])
        
        # Upright state [x, theta, x_dot, theta_dot]
        upright = np.array([0, np.pi, 0, 0])
        self.plant.SetPositionsAndVelocities(context, upright)
        
        Q = np.diag((10.0, 10.0, 1.0, 1.0))
        R = np.array([1.0])
        print("LQR controller created.")
        return LinearQuadraticRegulator(
            self.plant, context, Q, R,
            input_port_index=self.plant.get_actuation_input_port().get_index() #type: ignore
        )
    
    def _load_fvi_policy(self, policy_file):
        """Load precomputed FVI policy from a file."""
        from pydrake.math import BarycentricMesh
        data = np.load(policy_file, allow_pickle=True)
        x_grid = [set(float(point) for point in grid) for grid in data['x_grid']]
        u_grid = [set(float(point) for point in grid) for grid in data['u_grid']]
        output_values = data['output_values']
        
        # Reconstruct BarycentricMesh
        mesh = BarycentricMesh(x_grid)
        fvi_system = BarycentricMeshSystem(mesh, output_values)
        return fvi_system, output_values
    
    def _design_fvi(self, save_policy=False):
        """Design FVI controller using a separate plant-only simulator."""
        temp_builder = DiagramBuilder()
        temp_plant, _ = AddMultibodyPlantSceneGraph(temp_builder, time_step=0.0)
        parser = Parser(temp_plant)
        self.urdf_path = os.path.join(os.path.dirname(__file__), "cartpole.urdf")
        parser.AddModelsFromUrl(f"file://{self.urdf_path}")
        temp_plant.Finalize()
        
        # Export the actuation input port to the diagram
        temp_builder.ExportInput(temp_plant.get_actuation_input_port(), "cart_actuation")
        temp_diagram = temp_builder.Build()
        temp_simulator = Simulator(temp_diagram)
        
        # FVI setup
        num_grid_points = 21
        x_grid = [
            set(np.linspace(-2.5, 2.5, num=num_grid_points)),      # x
            set(np.linspace(0, 2*np.pi, num=num_grid_points)),     # theta
            set(np.linspace(-3, 3, num=num_grid_points)),        # x_dot
            set(np.linspace(-10, 10, num=num_grid_points))         # theta_dot
        ]
        u_grid = [set(np.linspace(-100, 100, num=41))]
        time_step = 0.01

        options = DynamicProgrammingOptions()
        theta_pbc = DynamicProgrammingOptions.PeriodicBoundaryCondition(1, 0, 2*np.pi)
        options.periodic_boundary_conditions = [theta_pbc]
        options.input_port_index = temp_diagram.get_input_port(0).get_index()  # Use the exported port
        options.discount_factor = 0.99
        
        def fvi_cost_function(context) -> float:
            plant_context = temp_plant.GetMyContextFromRoot(context)
            x = temp_plant.GetPositionsAndVelocities(plant_context)
            u = temp_plant.get_actuation_input_port().Eval(plant_context)
            
            # Ensure u is a 1D array
            if np.isscalar(u) or u.ndim == 0:
                u = np.array([u])
            
            upright = np.array([0, np.pi, 0, 0])
            Q = np.diag((1000.0, 1000.0, 1.0, 1.0))
            R = np.array([[0.001]])  # Ensure R is a 2D 1x1 matrix
            
            x_bar = x - upright
            state_cost = x_bar.T @ Q @ x_bar
            action_cost = u.T @ R @ u

            if abs(x[0]) > 2.4 or abs(x[3]) > 20.0:
                state_cost += 1e6  # Large penalty for going out of bounds
            
            return float(state_cost + action_cost)

        self.fvi_controller = FittedValueIteration( #type: ignore
            temp_simulator,
            fvi_cost_function, 
            x_grid,
            u_grid,
            time_step,
            options
        )

        # Debug: Inspect FVI policy outputs
        if self.fvi_controller is not None:
            fvi_system, _ = self.fvi_controller
            fvi_context = fvi_system.CreateDefaultContext()
            
            # Save policy if requested
            if save_policy:
                policy_file = os.path.join(os.path.dirname(__file__), "fvi_policy.npz")
                np.savez(
                    policy_file,
                    x_grid=[np.array(list(grid)) for grid in x_grid],
                    u_grid=[np.array(list(grid)) for grid in u_grid],
                    output_values=fvi_system.get_output_values()
                )
                print(f"Saved FVI policy to {policy_file}")
        
        print("FVI controller created.")
    
    def CalcControl(self, context, output):
        """Calculate control output based on current mode."""
        state = np.array(self.get_input_port(0).Eval(context))
        
        if self.controller_active.get("lqr", False) and self.lqr_controller is not None:
            # Only use LQR when near equilibrium
            lqr_context = self.lqr_controller.CreateDefaultContext()
            theta_error = abs(wrap_to(state[1] - np.pi, -np.pi, np.pi))
            if theta_error < np.pi/2:
                self.lqr_controller.get_input_port(0).FixValue(lqr_context, state)
                control = self.lqr_controller.get_output_port(0).Eval(lqr_context)[0]
            else:
                control = 0.0
        
        elif self.fvi_controller is not None and self.controller_active.get("fvi", False):
            fvi_system, _ = self.fvi_controller
            fvi_context = fvi_system.CreateDefaultContext()
            
            # Wrap theta to [0, 2π]
            fvi_state = state.copy()
            fvi_state[1] = wrap_to(state[1], 0, 2*np.pi)
            
            # Set input state
            try:
                fvi_system.get_input_port(0).FixValue(fvi_context, fvi_state.tolist())
            except Exception as e:
                print(f"Error setting FVI input: {e}")
                control = 0.0
            else:
                # Evaluate output
                try:
                    control_array = np.array(fvi_system.get_output_port(0).Eval(fvi_context))
                    if control_array.size > 0:
                        control = float(control_array[0])
                    else:
                        print("Warning: FVI output is empty")
                        control = 0.0
                except Exception as e:
                    print(f"Error evaluating FVI output: {e}")
                    control = 0.0
            self.debug_counter += 1
            
        elif self.controller_active.get("pid", False):
            # Simple PID on theta (pole angle)
            theta = state[1]
            theta_dot = state[3]
            
            # Target is upright (theta = pi)
            error = np.pi - theta
            # Wrap error to [-π, π]
            error = wrap_to(error, -np.pi, np.pi)
            
            # Get integral from discrete state
            integral = context.get_discrete_state(0).GetAtIndex(0)
            
            # PID control
            control = self.kp * error + self.ki * integral + self.kd * (-theta_dot)
            
        else:
            control = 0.0

        if self.controller_active.get("manual", False):
            control += self.manual_force
            
        control = np.clip(control, -100.0, 100.0)
        output.SetFromVector([control])
    
    def UpdateIntegral(self, context, discrete_state):
        """Update integral term for PID controller."""
        if self.controller_active.get("pid", False):
            state = np.array(self.get_input_port(0).Eval(context))
            theta = state[1]
            error = np.pi - theta
            error = wrap_to(error, -np.pi, np.pi)
            
            dt = 0.01
            current_integral = context.get_discrete_state(0).GetAtIndex(0)
            new_integral = current_integral + error * dt
            
            # Anti-windup
            new_integral = np.clip(new_integral, -10.0, 10.0)
            
            discrete_state.get_mutable_vector(0).SetAtIndex(0, new_integral)
    
    def toggle_controller(self, controller):
        """
        Switch control mode.
        
        Args:
            mode: One of "manual", "lqr", "pid", "zero"
        """
        if controller in ["manual", "lqr", "pid", "fvi"] and controller in self.controller_active:
            self.controller_active[controller] = not self.controller_active[controller]
            # Reset integral when switching to PID
            if controller == "pid" and self.controller_active["pid"]:
                self.integral_error = 0.0
        elif controller == "zero":
            for key in self.controller_active:
                self.controller_active[key] = False
        else:
            print(f"Unknown mode: {controller}")
    
    def set_manual_force(self, force):
        """Set the manual control force."""
        if self.controller_active.get("manual", False):
            self.manual_force = float(force)
        print("manual force:", self.manual_force)
    
    def set_pid_gains(self, kp=None, ki=None, kd=None):
        """Update PID gains."""
        if kp is not None:
            self.kp = kp
        if ki is not None:
            self.ki = ki
        if kd is not None:
            self.kd = kd
