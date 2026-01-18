let socket = null;
let connected = false;
let sid = null;

const statusDiv = document.getElementById("status");
const connectBtn = document.getElementById("connectBtn");
const disconnectBtn = document.getElementById("disconnectBtn");
const controlsDiv = document.getElementById("controls");
const resetBtn = document.getElementById("resetBtn");
const forceSlider = document.getElementById("forceSlider");
const forceValue = document.getElementById("forceValue");
const controllerSelect = document.getElementById("controllerSelect");
const setControllerBtn = document.getElementById("setControllerBtn");
const meshcatLink = document.getElementById("meshcatLink");
const meshcatFrame = document.getElementById("meshcatFrame");
const playPauseBtn = document.getElementById("playPauseBtn");

function setStatus(msg, color = "green") {
  statusDiv.textContent = msg;
  statusDiv.style.color = color;
}

function enableControls(enable) {
  controlsDiv.style.display = enable ? "" : "none";
  disconnectBtn.disabled = !enable;
  connectBtn.disabled = enable;
}

connectBtn.onclick = () => {
  socket = io("ws://localhost:8000", {transports: ["websocket"],});
  setStatus("Connecting...", "orange");
  let connectTimeout = setTimeout(() => {
    if (!connected) {
      setStatus("Error: Connection timed out", "red");
      connectBtn.disabled = false;
      disconnectBtn.disabled = true;
      if (socket) {
        socket.disconnect();
      }
    }
  }, 5000); // 5 seconds

  socket.on("connect", () => {
    clearTimeout(connectTimeout);
    connected = true;
    sid = socket.id;
    socket.emit("init", { "sim_type": "cartpole", "controllers": ["manual", "fvi"] });
    setStatus("Connected! SID: " + sid);
    enableControls(true);
  });

  socket.on("simulation_started", (data) => {
    setStatus("Simulation started! Meshcat URL: " + data.url);
    if (data.url) {
      meshcatLink.href = data.url;
      meshcatLink.style.display = "block";
      meshcatLink.textContent = "Open Meshcat Visualization";
      meshcatFrame.src = data.url;
      meshcatFrame.style.display = "block";
    }
  });

  socket.on("disconnect", () => {
    clearTimeout(connectTimeout);
    connected = false;
    setStatus("Disconnected", "red");
    enableControls(false);
    meshcatLink.style.display = "none";
    meshcatFrame.style.display = "none";
    meshcatFrame.src = "";
  });

  socket.on("error", (data) => {
    clearTimeout(connectTimeout);
    setStatus("Error: " + data.message, "red");
  });
};

disconnectBtn.onclick = () => {
  if (socket && connected) {
    socket.disconnect();
  }
};

resetBtn.onclick = () => {
  if (socket && connected) {
    socket.emit("input_event", { type: "reset" });
    setStatus("Simulation reset");
  }
};

playPauseBtn.onclick = () => {
  if (socket && connected) {
    const isPaused = playPauseBtn.textContent === "Play";
    socket.emit("ui_input", {
      action: isPaused ? "play" : "pause"
    });
    playPauseBtn.textContent = isPaused ? "Pause" : "Play";
    setStatus(isPaused ? "Simulation resumed" : "Simulation paused");
  }
};

forceSlider.oninput = () => {
  forceValue.textContent = forceSlider.value;
  if (socket && connected) {
    socket.emit("ui_input", {
      action: "set_force",
      value: parseFloat(forceSlider.value),
    });
  }
};

toggleControllerBtn.onclick = () => {
  if (socket && connected) {
    const controller = controllerSelect.value;
    socket.emit("ui_input", {
      action: "toggle_controller",
      value: { controller: controller },
    });
    setStatus("Controller set to " + controller);
  }
};

// Arrow key controls
window.addEventListener("keydown", (e) => {
  if (!connected) return;
  if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
    socket.emit("keyboard_down", { key: e.key });
  }
});
window.addEventListener("keyup", (e) => {
  if (!connected) return;
  if (e.key === "ArrowLeft" || e.key === "ArrowRight") {
    socket.emit("keyboard_up", { key: e.key });
  }
});
