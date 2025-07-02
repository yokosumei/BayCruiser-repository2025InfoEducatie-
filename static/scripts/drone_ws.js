document.addEventListener("DOMContentLoaded", () => {
  const socket = io();

  // Emitere comenzi
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

  // Recepție status dronă
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
});
