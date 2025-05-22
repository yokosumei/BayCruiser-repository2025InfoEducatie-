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

function toggleTab(tabId) {
  document.querySelectorAll('.tab-content').forEach(tab => {
    tab.classList.remove('active');
  });
  document.getElementById(tabId).classList.add('active');
}

// === POPUP DE DETECȚIE ===
function displayMessage() {
  if (!canShowMessage) return;

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
      .then(res => res.text())
      .then(() => {
        showResponse("Initiere protocol de salvare.");
      })
      .catch(() => {
        showResponse("Eroare la mișcarea servomotorului.");
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

function displayServoMessage() {
  fetch("/misca")
    .then(res => res.text())
    .then(() => {
      showResponse("Servomotorul a fost mișcat");
    })
    .catch(() => {
      showResponse("Eroare la mișcarea servomotorului.");
    });
}

// === POLLING pentru detecție ===
setInterval(() => {
  if (!streamActive || !canShowMessage) return;

  fetch("/detection_status")
    .then(res => res.json())
    .then(data => {
      if (data.detected && canShowMessage) {
        displayMessage();
      }
    })
    .catch(err => console.warn("Eroare la polling:", err));
}, 1000);
