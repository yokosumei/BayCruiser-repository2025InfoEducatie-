let streaming = false;
let streamActive = false;
let popupShown = false;
let detectedPreviously = false;

function updateStatusIndicators() {
  const circles = document.querySelectorAll('.circle .box');
  const status = document.querySelector('.status');

  if (streamActive) {
    status.textContent = '| Live';
    status.style.color = 'green';
    circles[0].textContent = '⚪';
    circles[1].textContent = '⚪';
    circles[2].textContent = '🟢';
  } else {
    status.textContent = '| Non-live';
    status.style.color = 'red';
    circles[0].textContent = '🔴';
    circles[1].textContent = '⚪';
    circles[2].textContent = '⚪';
  }
}

function startStream() {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      streaming = true;
      document.getElementById('video').style.display = 'block';
      updateStatusIndicators();
    });
}

function stopStream() {
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      streaming = false;
      document.getElementById('video').style.display = 'none';
      updateStatusIndicators();
    });
}

function toggleView() {
  const live = document.getElementById("livestream-article");
  const upload = document.getElementById("upload-article");

  if (live.style.display !== "none") {
    live.style.display = "none";
    upload.style.display = "block";

    const status = document.querySelector('.status');
    status.textContent = '| Upload';
    status.style.color = 'yellow';

    const circles = document.querySelectorAll('.circle .box');
    circles[0].textContent = '⚪';
    circles[1].textContent = '🟡';
    circles[2].textContent = '⚪';
  } else {
    upload.style.display = "none";
    live.style.display = "block";
    updateStatusIndicators();
  }
}

function setStreamView(mode) {
  const raw = document.getElementById("rawStream");
  const yolo = document.getElementById("yoloStream");

  if (mode === "raw") {
    raw.style.display = "inline-block";
    yolo.style.display = "none";
  } else if (mode === "yolo") {
    raw.style.display = "none";
    yolo.style.display = "inline-block";
  } else if (mode === "split") {
    raw.style.display = "inline-block";
    yolo.style.display = "inline-block";
  }
}

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

function confirmDetection(answer) {
  const popup = document.getElementById('popup-alert');
  if (popup) popup.style.display = 'none';

  if (answer) {
    fetch("/misca")
      .then(res => res.text())
      .then(() => alert("Initiere protocol de salvare."))
      .catch(() => alert("Eroare la mișcarea servomotorului."));
  } else {
    alert("Alarmă ignorată.");
  }
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
        popupShown = false;
        detectedPreviously = false;

        const popup = document.getElementById("popup-alert");
        if (popup) popup.style.display = "none";

        const img = document.getElementById("popup-frame");
        if (img) img.src = "";
      }
    })
    .catch(err => console.warn("Eroare verificare detecție:", err));
}
setInterval(checkDetectionStatus, 1000);

function showPopup() {
  const popup = document.getElementById("popup-alert");
  if (popup) popup.style.display = "flex";

  const img = document.getElementById("popup-frame");
  if (img) {
    img.src = "/yolo_feed_snapshot?" + new Date().getTime(); // evităm cache
  }
}

function displayServoMessage() {
  fetch("/misca")
    .then(res => res.text())
    .then(() => alert("Servomotorul a fost mișcat"))
    .catch(() => alert("Eroare la mișcarea servomotorului."));
}

function TakeOff() {
  fetch("/takeoff")
    .then(res => res.text())
    .then(() => alert("Drona a decolat."));
}

function Ateriazare() {
  fetch("/land")
    .then(res => res.text())
    .then(() => alert("Drona a aterizat."));
}
