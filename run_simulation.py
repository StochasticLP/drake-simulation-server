from simulations.simulate_system import Simulation
import asyncio

async def main():
    # Create a simulation with default parameters (will use cartpole)
    sim = Simulation(sid="test_simulation", params={"sim_type": "cartpole", "controllers": ["fvi"] })
    # Print the meshcat URL where you can view the simulation
    print("Meshcat URL:", sim.url)
    # Start the simulation
    await sim.start()
    await sim.ready_event.wait()  # Wait for setup (Meshcat, diagram, init)

    try:
        # Run for 10 seconds
        print("Running simulation for 60 seconds...")
        await asyncio.sleep(60)
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    finally:
        # Clean up
        sim.stop()
        print("Simulation stopped")

if __name__ == "__main__":
    asyncio.run(main())
