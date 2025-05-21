// === VARIABILE DE STARE ===
let canShowMessage = true;
let streamActive = false;

// === CONTROL STREAM VIDEO ===
  function startStream() {
                    fetch('/start_stream');
                }
 function stopStream() {
                     fetch('/stop_stream');
                }

// === TABURI ===
     function toggleTab(tabId) {
  const contents = document.querySelectorAll('.tab-content');
  const buttons = document.querySelectorAll('.tab-button');
  const container = document.getElementById('tabContent');

  let isAlreadyOpen = false;

  contents.forEach(content => {
    if (content.id === tabId && content.classList.contains('active')) {
      isAlreadyOpen = true;
    }
    content.classList.remove('active');
  });

  buttons.forEach(btn => btn.classList.remove('active'));

  if (isAlreadyOpen) {
    container.classList.remove('active');
    return;
  }

  document.getElementById(tabId).classList.add('active');
  container.classList.add('active');

  const activeButton = Array.from(buttons).find(
    btn => btn.textContent.trim().toLowerCase() === tabId.toLowerCase()
  );
  if (activeButton) activeButton.classList.add('active');
}

// === POPUP DETECȚIE „om_la_inec” ===
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

// === AFIȘARE MESAJ INFORMATIV TEMPORAR ===
function showResponse(msg) {
  const responsePanel = document.createElement("div");
  responsePanel.className = "responseBox";
  responsePanel.textContent = msg;
  document.body.appendChild(responsePanel);
  setTimeout(() => responsePanel.remove(), 2000);
}

// === COMANDĂ MANUALĂ SERVOMOTOR ===
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

// === POLLING DETECȚIE AUTOMATĂ ===
setInterval(() => {
  if (!streamActive || !canShowMessage) return;

  fetch("/detection_status")
    .then(res => res.json())
    .then(data => {
      if (data.detected && canShowMessage) {
        displayMessage();
      }
    })
    .catch(err => console.warn("Eroare verificare detecție:", err));
}, 1000);
