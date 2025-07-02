// === drone_ws.js ===
let socket;  // globală

function setupJoystick(containerId, axis) {
  const container = document.getElementById(containerId);
  const knob = container.querySelector('.knob');
  const rect = container.getBoundingClientRect();
  const centerX = rect.width / 2;
  const centerY = rect.height / 2;
  const maxDist = rect.width / 2;

  let currentDX = 0;
  let currentDY = 0;

  function moveKnob(x, y) {
    knob.style.transform = `translate(calc(-50% + ${x}px), calc(-50% + ${y}px))`;
  }

  function resetKnob() {
    currentDX = 0;
    currentDY = 0;
    moveKnob(0, 0);
  }

  function getRelativeCoords(clientX, clientY) {
    const bounds = container.getBoundingClientRect();
    const x = clientX - bounds.left - centerX;
    const y = clientY - bounds.top - centerY;
    const dist = Math.min(Math.sqrt(x * x + y * y), maxDist);
    const angle = Math.atan2(y, x);
    return {
      dx: Math.cos(angle) * dist / maxDist,
      dy: Math.sin(angle) * dist / maxDist
    };
  }

  let dragging = false;

  function update(e) {
    const clientX = e.touches ? e.touches[0].clientX : e.clientX;
    const clientY = e.touches ? e.touches[0].clientY : e.clientY;
    const { dx, dy } = getRelativeCoords(clientX, clientY);
    currentDX = dx;
    currentDY = dy;
    moveKnob(dx * maxDist, dy * maxDist);
  }

  container.addEventListener('mousedown', e => {
    dragging = true;
    update(e);
  });

  container.addEventListener('mousemove', e => {
    if (dragging) update(e);
  });

  document.addEventListener('mouseup', () => {
    if (dragging) {
      dragging = false;
      resetKnob();
    }
  });

  container.addEventListener('touchstart', e => {
    dragging = true;
    update(e);
  });

  container.addEventListener('touchmove', e => {
    if (dragging) update(e);
  });

  container.addEventListener('touchend', () => {
    dragging = false;
    resetKnob();
  });

  resetKnob();

  return () => ({ dx: currentDX, dy: currentDY });
}

let getHJoystick = null;
let getVJoystick = null;

// === Setup WebSocket ===
document.addEventListener("DOMContentLoaded", () => {
  socket = io();

  // === Comenzi drone ===
  window.TakeOff = function() {
    socket.emit('drone_command', { action: 'takeoff' });
    alert("Comandă decolare trimisă.");
  };

  window.Land = function() {
    socket.emit('drone_command', { action: 'land' });
    alert("Comandă aterizare trimisă.");
  };

  window.startGotoAndReturn = function() {
    socket.emit('drone_command', { action: 'goto_and_return' });
    alert("Comandă Go & Return trimisă.");
  };

  window.startOrbit = function() {
    socket.emit('drone_command', { action: 'orbit' });
    alert("Comandă Orbit trimisă.");
  };

  // === Status dronă ===
  socket.on('drone_status', (data) => {
    const droneIcon = document.getElementById("drone-connection");

    if (data.connected) {
      droneIcon.style.filter = "brightness(1) saturate(1000%) sepia(100%) hue-rotate(-50deg)";
      droneIcon.style.boxShadow = "0 0 8px 3px red";
      droneIcon.title = "Dronă conectată";
    } else {
      droneIcon.style.filter = "brightness(0)";
      droneIcon.style.boxShadow = "none";
      droneIcon.title = "Dronă deconectată";
    }

    document.getElementById("battery").innerText = data.battery.level + "%";
    document.getElementById("armed").innerText = data.armed ? "DA" : "NU";
    document.getElementById("mode").innerText = data.mode;

    if (data.location && data.location.lat && data.location.lon) {
      document.getElementById("current-coords").innerText =
        `Lat: ${data.location.lat.toFixed(5)}, Lon: ${data.location.lon.toFixed(5)}`;
    }

    if (data.event_location && data.event_location.lat && data.event_location.lon) {
      document.getElementById("event-coords").innerText =
        `Lat: ${data.event_location.lat.toFixed(5)}, Lon: ${data.event_location.lon.toFixed(5)}`;
    } else {
      document.getElementById("event-coords").innerText = "N/A";
    }
  });

  // === Inițializează joystick-uri ===
  getHJoystick = setupJoystick("joystick-horizontal", "horizontal");
  getVJoystick = setupJoystick("joystick-vertical", "vertical");

  // === Trimite comenzi joystick la fiecare 100ms ===
  setInterval(() => {
    if (!getHJoystick || !getVJoystick) return;
    const h = getHJoystick();
    const v = getVJoystick();

    socket.emit('joystick_move', {
      joystick: 'combined',
      x: h.dx,   // stânga-dreapta → roll
      y: v.dy,   // sus-jos → throttle
      z: v.dx,   // față-spate → pitch
      yaw: h.dy  // rotire → yaw
    });
  }, 100);
});
