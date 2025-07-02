let streaming = false;
let streamActive = false;
let popupShown = false;
let detectedPreviously = false;

window.updateStatusIndicators = function () {
  const boxes = document.querySelectorAll('.box');
  const status = document.querySelector('.status');

  if (streamActive) {
    status.textContent = '| Live';
    status.style.color = 'green';
    if (boxes.length >= 3) {
      boxes[0].textContent = '⚪';
      boxes[1].textContent = '⚪';
      boxes[2].textContent = '🟢';
    }
  } else {
    status.textContent = '| Non-live';
    status.style.color = 'red';
    if (boxes.length >= 3) {
      boxes[0].textContent = '🔴';
      boxes[1].textContent = '⚪';
      boxes[2].textContent = '⚪';
    }
  }
};

window.startStream = function () {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      streaming = true;
      window.setStreamView('raw');
      window.updateStatusIndicators();
    });
};

window.stopStream = function () {
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      streaming = false;
      document.getElementById('rawStream').style.display = 'none';
      document.getElementById('yoloStream').style.display = 'none';
      window.updateStatusIndicators();
    });
};

window.toggleView = function () {
  const live = document.getElementById("livestream-article");
  const upload = document.getElementById("upload-article");
  const boxes = document.querySelectorAll('.circle .box');
  const status = document.querySelector('.status');

  if (live.style.display !== "none") {
    live.style.display = "none";
    upload.style.display = "block";
    status.textContent = '| Upload';
    status.style.color = 'yellow';

    if (boxes.length >= 3) {
      boxes[0].textContent = '⚪';
      boxes[1].textContent = '🟡';
      boxes[2].textContent = '⚪';
    }
  } else {
    upload.style.display = "none";
    live.style.display = "block";
    setTimeout(window.updateStatusIndicators, 10);
  }
};

window.setStreamView = function (mode) {
  const raw = document.getElementById("rawStream");
  const yolo = document.getElementById("yoloStream");

  if (!streaming) {
    raw.style.display = "none";
    yolo.style.display = "none";
    return;
  }

  raw.classList.remove("single", "split");
  yolo.classList.remove("single", "split");

  if (mode === "raw") {
    raw.style.display = "inline-block";
    raw.classList.add("single");
    yolo.style.display = "none";
  } else if (mode === "yolo") {
    yolo.style.display = "inline-block";
    yolo.classList.add("single");
    raw.style.display = "none";
  } else if (mode === "split") {
    raw.style.display = "inline-block";
    yolo.style.display = "inline-block";
    raw.classList.add("split");
    yolo.classList.add("split");
  }
};

window.toggleTab = function (tabId) {
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
};

window.confirmDetection = function (answer) {
  const popup = document.getElementById('popup-alert');
  if (popup) popup.style.display = 'none';

  if (answer) {
    fetch("/misca")
      .then(res => res.text())
      .then(() => {
        alert("Inițiere protocol de salvare.");
        window.setStreamView('split');
      })
      .catch(() => {
        alert("Eroare la mișcarea servomotorului.");
      });
  } else {
    alert("Alarmă ignorată.");
  }
  detectedPreviously = true;
};

window.checkDetectionStatus = function () {
  if (!streamActive) return;

  fetch('/detection_status')
    .then(response => response.json())
    .then(data => {
      if (data.detected) {
        if (!popupShown && !detectedPreviously) {
          popupShown = true;
          window.showPopup();
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
};

setInterval(window.checkDetectionStatus, 1000);

window.showPopup = function () {
  const popup = document.getElementById("popup-alert");
  if (popup) popup.style.display = "flex";

  const img = document.getElementById("popup-frame");
  if (img) {
    img.src = "/yolo_feed_snapshot?" + new Date().getTime();
  }
};

window.displayServoMessage = function () {
  fetch("/misca")
    .then(res => res.text())
    .then(() => alert("Servomotorul a fost mișcat"))
    .catch(() => alert("Eroare la mișcarea servomotorului."));
};
