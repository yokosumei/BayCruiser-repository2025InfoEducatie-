let streamActive = false;
let popupShown = false;
let detectedPreviously = false;

function startStream() {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      document.getElementById('video').style.display = 'block';
    });
}

function stopStream() {
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      document.getElementById('video').style.display = 'none';
    });
}

function confirmDetection(answer) {
  document.getElementById('popup-alert').style.display = 'none';
  if (answer) {
    fetch("/misca")
      .then(res => res.text())
      .then(() => alert("Initiere protocol de salvare."))
      .catch(() => alert("Eroare la mișcarea servomotorului."));
  } else {
    alert("Alarmă ignorată.");
  }
  // După ce popupul a fost tratat, așteptăm re-detecție după ieșire din cadru
  detectedPreviously = true;
}

function checkDetectionStatus() {
  if (!streamActive) return;
  fetch('/detection_status')
    .then(response => response.json())
    .then(data => {
      if (data.detected) {
        if (!popupShown && !detectedPreviously) {
          popupShown = true;
          showPopup();
        }
      } else {
        // resetăm starea dacă obiectul a ieșit din cadru
        popupShown = false;
        detectedPreviously = false;
        const existing = document.getElementById("popup-alert");
        if (existing) existing.remove();
      }
    })
    .catch(err => console.warn("Eroare verificare detecție:", err));
}

function showPopup() {
  const popup = document.getElementById("popup-alert");
  if (popup) popup.style.display = "flex";
}


setInterval(checkDetectionStatus, 2000);

function displayServoMessage() {
  fetch("/misca")
    .then(res => res.text())
    .then(() => {
      alert("Servomotorul a fost mișcat");
    })
    .catch(() => {
      alert("Eroare la mișcarea servomotorului.");
    });
}

function TakeOff() {
  fetch("/takeoff")
    .then(res => res.text())
    .then(() => {
      alert("Drona a decolat.");
    });
}

function Land() {
  fetch("/land")
    .then(res => res.text())
    .then(() => {
      alert("Drona a aterizat.");
    });
}
