let streaming = false;
let streamActive = false;
let popupShown = false;
let detectedPreviously = false;

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

function updateDroneStatus() {
fetch("/drone_status")
  .then(res => res.json())
  .then(data => {
    console.log("Received drone status:", data); // 🔍 vezi ce primești
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
  })
  .catch(err => {
    console.error("Eroare la fetch /drone_status:", err);
  });

}




setInterval(updateDroneStatus, 1000);
