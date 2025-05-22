let canShowMessage = true;
let streamActive = false;

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

// === VERIFICARE DETECȚIE LA INTERVAL ===
setInterval(() => {
  if (!streamActive) return;
  fetch('/detection_status')
    .then(res => res.json())
    .then(data => {
      if (data.detected) {
        displayMessage();
      }
    });
}, 2000); // La fiecare 2 secunde

// === POPUP DE DETECȚIE ===
function displayMessage() {
  if (!canShowMessage) return;
  canShowMessage = false;
  setTimeout(() => {
    canShowMessage = true;
  }, 10000);

  const panel = document.createElement("div");
  panel.className = "msgBox";

  const question = document.createElement("p");
  question.textContent = "Alarmă falsă?";
  panel.appendChild(question);

  const btnContainer = document.createElement("div");
  btnContainer.className = "btnContainer";

  const daBtn = document.createElement("button");
  daBtn.textContent = "DA";
  daBtn.onclick = () => {
    fetch("/misca")
      .then(() => {
        showResponse("Inițiere protocol de salvare.");
      });
    panel.remove();
    canShowMessage = false;
  };

  const nuBtn = document.createElement("button");
  nuBtn.textContent = "NU";
  nuBtn.onclick = () => {
    showResponse("Alarmă ignorată.");
    panel.remove();
    canShowMessage = false;
  };

  btnContainer.appendChild(daBtn);
  btnContainer.appendChild(nuBtn);
  panel.appendChild(btnContainer);
  document.body.appendChild(panel);
}

function showResponse(msg) {
  const responsePanel = document.createElement("div");
  responsePanel.className = "responseBox";
  responsePanel.textContent = msg;
  document.body.appendChild(responsePanel);
  setTimeout(() => responsePanel.remove(), 2000);
}
