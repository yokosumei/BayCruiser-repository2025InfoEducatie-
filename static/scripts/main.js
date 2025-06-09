// === VARIABILE DE STARE ===
let canShowMessage = true;
let streamActive = false;

// === CONTROL STREAM VIDEO ===
 function startStream() {
    fetch('/start_stream')
        .then(() => {
            streamActive = true;
            document.getElementById('video').style.display = 'block';
        });

function startStream() {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      document.getElementById('video').style.display = 'block';
      document.querySelector('.status').textContent = '| Live';
      document.querySelector('.status').style.color = 'green';

      const circles = document.querySelectorAll('.circle .box');
      circles[0].textContent = '⚪'; // upload off 
      circles[1].textContent = '⚪'; // stop off
      circles[2].textContent = '🟢'; // start on
    });
}

function stopStream() {
    fetch('/stop_stream')
        .then(() => {
            streamActive = false;
            document.getElementById('video').style.display = 'none';
        });
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      document.getElementById('video').style.display = 'none';
      document.querySelector('.status').textContent = '| Non-live';
      document.querySelector('.status').style.color = 'red';

      const circles = document.querySelectorAll('.circle .box');
      circles[0].textContent = '🔴'; 
      circles[1].textContent = '⚪'; 
      circles[2].textContent = '⚪'; 
    });
}

function toggleView() {
  const live = document.getElementById("livestream-article");
  const upload = document.getElementById("upload-article");
  const circles = document.querySelectorAll('.circle .box');

  if (live.style.display !== "none") {
    live.style.display = "none";
    upload.style.display = "block";
    document.querySelector('.status').textContent = '| Upload';
    document.querySelector('.status').style.color = 'yellow';

    circles[0].textContent = '⚪'; // upload on
    circles[1].textContent = '🟡';
    circles[2].textContent = '⚪';
  } else {
    upload.style.display = "none";
    live.style.display = "block";

    if (streamActive) {
      document.querySelector('.status').textContent = '| Live';
      document.querySelector('.status').style.color = 'green';
      circles[0].textContent = '⚪';
      circles[1].textContent = '⚪';
      circles[2].textContent = '🟢';
    } else {
      document.querySelector('.status').textContent = '| Non-live';
      document.querySelector('.status').style.color = 'red';
      circles[0].textContent = '🔴';
      circles[1].textContent = '⚪';
      circles[2].textContent = '⚪';
    }
  }
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

function toggleView() {
  const live = document.getElementById("livestream-article");
  const upload = document.getElementById("upload-article");

  if (live.style.display !== "none") {
    live.style.display = "none";
    upload.style.display = "block";
  } else {
    upload.style.display = "none";
    live.style.display = "block";
  }
}
// === POPUP DETECȚIE „om_la_inec” ===
function displayMessage() {
  if (!canShowMessage) return;
  const panel = document.createElement("div");
  panel.className = "msgBox";
  const question = document.createElement("p");
  question.textContent = "Alarmă reală?";
  panel.appendChild(question);
  const btnContainer = document.createElement("div");
  btnContainer.className = "btnContainer";
  const daBtn = document.createElement("button");
  daBtn.textContent = "DA";
  daBtn.onclick = () => {
    fetch("/misca")
      .then(res => res.text())
      .then(() => {
        alert("Initiere protocol de salvare.");})
      .catch(() => {
        alert("Eroare la mișcarea servomotorului."); });
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
function TakeOff() {
  fetch("/takeoff")
    .then(res => res.text())
    .then(() => {
      alert("Drona a decolat.");
        })
}
function Ateriazare() {
  fetch("/land")
    .then(res => res.text())
    .then(() => {
      alert("Drona a aterizat.");
      
    })
}
