from simulations.simulate_system import Simulation
import asyncio
import time
from fastapi import FastAPI
import socketio
import uvicorn

# FastAPI and Socket.IO setup
app = FastAPI()
sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")
sio_app = socketio.ASGIApp(sio, app)

# Active simulations: sid -> {"sim": Simulation, "timer": task, "last_activity": time}
active_simulations = {}

async def shutdown(sid):
    if sid in active_simulations:
        sim_data = active_simulations[sid]
        sim_data["sim"].stop()
        del active_simulations[sid]
        print(f"Shutdown simulation for {sid}")

async def schedule_shutdown(sid):
    await asyncio.sleep(300)  # 5 minutes
    await shutdown(sid)

def update_activity(sid):
    if sid in active_simulations:
        sim_data = active_simulations[sid]
        if "timer" in sim_data and sim_data["timer"]:
            sim_data["timer"].cancel()
        sim_data["timer"] = asyncio.create_task(schedule_shutdown(sid))
        sim_data["last_activity"] = time.time()

MAX_SIMS = 10  # Adjust based on server capacity
@sio.event
async def connect(sid, environ):
    if len(active_simulations) >= MAX_SIMS:
        await sio.emit("error", {"message": "Server at capacity"}, to=sid)
        await sio.disconnect(sid)
        return
    print(f"User {sid} connected")

@sio.event
async def init(sid, params=None):
    if sid in active_simulations:
        await sio.emit("error", {"message": "Simulation already initialized"}, to=sid)
        return
    sim = Simulation(sid, params)
    await sim.start()
    await sim.ready_event.wait()  # Wait for setup (Meshcat, diagram, init)
    active_simulations[sid] = {"sim": sim, "timer": None, "last_activity": time.time()}
    update_activity(sid)
    await sio.emit("simulation_started", {"url": sim.url}, to=sid)

@sio.event
async def get_output(sid):
    if sid in active_simulations:
        interval = 1.0 / 24.0  # 24Hz rate limiting
        data = active_simulations[sid]["sim"].output_data
        while sid in active_simulations:  # Continue while simulation is active
            new_data = active_simulations[sid]["sim"].output_data
            if new_data != data:  # Only send if data has changed
                data = new_data
                if not data.empty():
                    await sio.emit("sim_data", data, to=sid)
            await asyncio.sleep(interval)

@sio.event
async def disconnect(sid):
    print(f"User {sid} disconnected")
    await shutdown(sid)

@sio.event
async def input_event(sid, data):
    if sid in active_simulations:
        update_activity(sid)
        await active_simulations[sid]["sim"].input_queue.put(data)  # Await for async queue

# General event for keyboard, ui, params
@sio.event
async def keyboard_down(sid, data):
    await input_event(sid, {"type": "keyboard_down", "key": data["key"]})

@sio.event
async def keyboard_up(sid, data):
    await input_event(sid, {"type": "keyboard_up", "key": data["key"]})

@sio.event
async def ui_input(sid, data):
    await input_event(sid, {"type": "ui_input", **data})

@sio.event
async def update_param(sid, data):
    await input_event(sid, {"type": "update_param", "params": data})

if __name__ == "__main__":
    uvicorn.run(sio_app, host="0.0.0.0", port=8000)
