let socket;  // global

function setupJoystick(containerId, axis) {
  const container = document.getElementById(containerId);
  const knob = container ? container.querySelector('.knob') : null;

  if (!container || !knob) {
    console.warn(`Joystick "${containerId}" nu a fost găsit în DOM.`);
    return () => ({ dx: 0, dy: 0 });  // fallback: joystick inactiv
  }

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

let getHJoystick = () => ({ dx: 0, dy: 0 });
let getVJoystick = () => ({ dx: 0, dy: 0 });

document.addEventListener("DOMContentLoaded", () => {
  socket = io();

  // === Comenzi drone ===
  window.TakeOff = function() {
    if (socket && socket.connected) {
      socket.emit('drone_command', { action: 'takeoff' });
    }
  };

  window.Land = function() {
    if (socket && socket.connected) {
      socket.emit('drone_command', { action: 'land' });
      alert("Comandă aterizare trimisă.");
    }
  };

  window.startGotoAndReturn = function() {
    if (socket && socket.connected) {
      socket.emit('drone_command', { action: 'goto_and_return' });
      alert("Comandă Go & Return trimisă.");
    }
  };

  window.startOrbit = function() {
    if (socket && socket.connected) {
      socket.emit('drone_command', { action: 'orbit' });
      alert("Comandă Orbit trimisă.");
    }
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
  if (!socket || !socket.connected) {
    console.warn("Socket not connected, skipping joystick update."); 
    return;
  }

  if (!getHJoystick || !getVJoystick) {
    console.warn("Joystick not initialized, skipping joystick update.");
    return;
  } // <-- ACEASTA lipsea

  const h = getHJoystick();
  const v = getVJoystick();
  console.log(`Joystick values: h.dx=${h.dx}, h.dy=${h.dy}, v.dx=${v.dx}, v.dy=${v.dy}`);

  // fallback în caz că joystick-ul nu returnează valori valide
  if (isNaN(h.dx) || isNaN(h.dy) || isNaN(v.dx) || isNaN(v.dy)) return;

  console.warn("emit joystick_move.");
  socket.emit('joystick_move', {
    joystick: 'combined',
    x: h.dx,
    y: v.dy,
    z: v.dx,
    yaw: h.dy
  });
}, 100);

});
