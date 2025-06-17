let streaming = false;
let streamActive = false;
let popupShown = false;
let detectedPreviously = false;

document.addEventListener("DOMContentLoaded", () => {
  function setupJoystick(containerId) {
    const container = document.getElementById(containerId);
    const knob = container.querySelector('.knob');
    const maxDist = container.offsetWidth / 2;

    function moveKnob(dx, dy) {
      knob.style.transform = `translate(${dx}px, ${dy}px)`;
    }

    function resetKnob() {
      moveKnob(0, 0);
    }

    function getRelativeCoords(clientX, clientY) {
      const rect = container.getBoundingClientRect();
      const centerX = rect.left + rect.width / 2;
      const centerY = rect.top + rect.height / 2;
      const x = clientX - centerX;
      const y = clientY - centerY;
      const dist = Math.min(Math.hypot(x, y), maxDist);
      const angle = Math.atan2(y, x);
      return {
        dx: Math.cos(angle) * dist,
        dy: Math.sin(angle) * dist
      };
    }

    let dragging = false;

    function updatePosition(e) {
      const clientX = e.touches ? e.touches[0].clientX : e.clientX;
      const clientY = e.touches ? e.touches[0].clientY : e.clientY;
      const { dx, dy } = getRelativeCoords(clientX, clientY);
      moveKnob(dx, dy);
    }

    container.addEventListener('mousedown', e => {
      dragging = true;
      updatePosition(e);
    });

    container.addEventListener('mousemove', e => {
      if (dragging) updatePosition(e);
    });

    document.addEventListener('mouseup', () => {
      if (dragging) {
        dragging = false;
        resetKnob();
      }
    });

    container.addEventListener('touchstart', e => {
      dragging = true;
      updatePosition(e);
    });

    container.addEventListener('touchmove', e => {
      if (dragging) updatePosition(e);
    });

    container.addEventListener('touchend', () => {
      dragging = false;
      resetKnob();
    });

    resetKnob();
  }

  setupJoystick("joystick-horizontal");
  setupJoystick("joystick-vertical");
});

function updateStatusIndicators() {
  const boxes = document.querySelectorAll('.box');
  const status = document.querySelector('.status');

  if (streamActive) {
    status.textContent = '| Live';
    status.style.color = 'green';
    if (boxes.length >= 3) {
      boxes[0].textContent = '⚪'; // Upload
      boxes[1].textContent = '⚪'; // Stop
      boxes[2].textContent = '🟢'; // Start
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
}

function startStream() {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      streaming = true;
      setStreamView('raw'); 
      updateStatusIndicators();
    });
}
function stopStream() {
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      streaming = false;
      document.getElementById('rawStream').style.display = 'none';
      document.getElementById('yoloStream').style.display = 'none';
      updateStatusIndicators();
    });
}
function toggleView() {
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
      boxes[0].textContent = '⚪'; // upload ON
      boxes[1].textContent = '🟡';
      boxes[2].textContent = '⚪';
    }
  } else {
    upload.style.display = "none";
    live.style.display = "block";
    setTimeout(updateStatusIndicators, 10);
  }
}
function setStreamView(mode) {
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
      .then(() => {
        alert("Inițiere protocol de salvare.");
        setStreamView('split');
      })
      .catch(() => {
        alert("Eroare la mișcarea servomotorului.");
      });
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
