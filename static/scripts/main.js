changeRightStream///video

let streamActive = false;
let popupShown = false;
let detectedPreviously = false;
let poseStarted = false;
let currentViewMode = 'raw';
let currentStartStop = 'start'; // 'start' or 'stop'

const socket = io();

// actualizează indicatorul vizual live / non-live
function updateStatusIndicators() {
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
}

// START stream video
function startStream() {
  fetch('/start_stream')
    .then(() => {
      streamActive = true;
      toggleStreamView();
      updateStatusIndicators();
    });
}

// STOP stream video
function stopStream() {
  fetch('/stop_stream')
    .then(() => {
      streamActive = false;
      document.getElementById('rawStream').style.display = 'none';
      document.getElementById('rightStream').style.display = 'none';
      updateStatusIndicators();
    });
}



function toggleStartStop() {
  const btn = document.getElementById("toggleStartStopBtn");
  const viewBtn = document.getElementById("toggleViewBtn");
  currentViewMode = 'split';
  toggleStreamView();

  if (currentStartStop === 'start') {
    startStream();
    btn.textContent = 'STOP';
    currentStartStop = 'stop';
    viewBtn.disabled = false;
  } else {
    stopStream();
    btn.textContent = 'START';
    currentStartStop = 'start';
    viewBtn.disabled = true;
  }
}


function toggleStreamView() {
  const btn = document.getElementById("toggleViewBtn");

  if (currentViewMode === 'raw') {
    setStreamView('raw');
    btn.textContent = 'SPLIT';
    currentViewMode = 'split';
  } else {
    setStreamView('split');
    btn.textContent = 'RAW';
    currentViewMode = 'raw';
  }
}


// comută între view RAW / SPLIT
function setStreamView(mode) {
  const raw = document.getElementById("rawStream");
  const right = document.getElementById("rightStream");

  if (!streamActive) {
    raw.style.display = "none";
    right.style.display = "none";
    return;
  }

  raw.classList.remove("single", "split");
  right.classList.remove("single", "split");

  if (mode === "raw") {
    raw.style.display = "inline-block";
    right.style.display = "none";
    raw.classList.add("single");
  } else if (mode === "split") {
    raw.style.display = "inline-block";
    right.style.display = "inline-block";
    raw.classList.add("split");
    right.classList.add("split");
  }
}

// comută streamul dreapta (YOLO, SEG, MAR, XGB)
function changeRightStream(type) {
  fetch("/set_right_stream", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ type: type })
  }).then(resp => resp.json()).then(data => {
    if (data.status === "ok") {
      const rs = document.getElementById("rightStream");
      rs.src = "/right_feed?dummy=" + Date.now();

      // actualizează textul "Mod detectare"
      const label = document.getElementById("detection-mode-label");
      const nameMap = {
        yolo: "INEC (YOLO)",
        seg: "SEGMENTARE",
        mar: "LIVINGS (RECHINI, PERSOANE)",
        xgb: "POSE + XGBoost"
      };
      label.textContent = "Mod detectare: " + (nameMap[type] || type.toUpperCase());
    }
  });
}


// pornește SEG + LIVINGS, și dacă apare person -> comută pe XGB
function startOfficialMode() {
  fetch("/start_official")
    .then(() => {
      changeRightStream("mar");
      setStreamView("split");
      btn.textContent = 'RAW';
      currentViewMode = 'raw';
    });
}

// DETECȚIE și NIVEL realtime
socket.on("detection_update", data => {
  const detectie = document.getElementById("detectie-info");
  const nivel = document.getElementById("nivel-info");

  // Dacă e eveniment de înec
  if (data.eveniment && data.gps) {
    const msg = `${data.eveniment} la [${data.gps.lat}, ${data.gps.lon}] @ ${data.timestamp.toFixed?.(2) ?? data.timestamp}`;
    detectie.textContent = "ALERTĂ: " + msg;
    nivel.textContent = "NIVEL: " + (data.nivel || "-");
    return;
  }

  // Dacă sunt obiecte detectate
  let text = "-";
  if (data.obiecte && data.obiecte.length > 0) {
    const count = {};
    data.obiecte.forEach(obj => {
      count[obj] = (count[obj] || 0) + 1;
    });
    text = Object.entries(count)
      .map(([k, v]) => `${v} ${k}${v > 1 ? "i" : ""}`)
      .join(", ");

    detectie.textContent = "DETECȚIE: " + text;
    nivel.textContent = "NIVEL: " + (data.nivel || "-");

    if (data.obiecte.includes("person") && !poseStarted) {
      poseStarted = true;
      changeRightStream("xgb");
      console.log("[AUTO] Comut pe pose+xgb");
    }
  } else {
    detectie.textContent = "DETECȚIE: -";
    nivel.textContent = "NIVEL: -";
  }
});


// POPUP dacă detectează "om_la_inec"
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

// afișează popup
function showPopup() {
  const popup = document.getElementById("popup-alert");
  if (popup) popup.style.display = "flex";

  const img = document.getElementById("popup-frame");
  if (img) {
    img.src = "/yolo_feed_snapshot?" + new Date().getTime(); // evită cache
  }
}

// confirmare buton DA / NU
function confirmDetection(answer) {
  const popup = document.getElementById('popup-alert');
  if (popup) popup.style.display = 'none';

  if (answer) {
    fetch("/misca")
      .then(() => {
        // alert("Inițiere protocol de salvare.");
        setStreamView('split');
        btn.textContent = 'RAW';
        currentViewMode = 'raw';
      })
      .catch(() => {
        alert("Eroare la mișcarea servomotorului.");
      });
  } else {
    alert("Alarmă ignorată.");
  }
  detectedPreviously = true;
}

function displayServoMessage() {
  fetch("/misca")
    .then(() => alert("Servomotorul a fost mișcat"))
    .catch(() => alert("Eroare la mișcarea servomotorului."));
}

function activateServos() {
  const msgDiv = document.getElementById("servo-status-msg");

  fetch("/misca")
    .then(response => {
      if (response.ok) {
        showServoMessage("Servomotor activat cu succes.", true);
      } else {
        showServoMessage("Eroare la activarea servomotorului.", false);
      }
    })
    .catch(error => {
      console.error("Eroare fetch /misca:", error);
      showServoMessage("Conexiune eșuată cu serverul.", false);
    });
}

// === STREAM SWITCH LEFT + RIGHT (auto din backend) ===
socket.on("stream_config_update", (data) => {
  console.log("[SOCKET] stream_config_update:", data);
  const left = document.getElementById("rawStream");
  const right = document.getElementById("rightStream");

  if (left && data.left) {
    if (data.left === "raw") left.src = "/video_feed";
    else if (data.left === "mar") left.src = "/mar_feed";
    else if (data.left === "seg") left.src = "/seg_feed";
    else if (data.left === "xgb") left.src = "/xgb_feed";
  }

  if (right && data.right) {
    if (data.right === "xgb") right.src = "/xgb_feed";
    else if (data.right === "seg") right.src = "/seg_feed";
    else if (data.right === "mar") right.src = "/mar_feed";
    else if (data.right === "yolo") right.src = "/yolo_feed";
  }

  // actualizează textul "Mod detectare"
  const label = document.getElementById("detection-mode-label");
  const nameMap = {
    yolo: "INEC (YOLO)",
    seg: "SEGMENTARE",
    mar: "LIVINGS (RECHINI, PERSOANE)",
    xgb: "POSE + XGBoost"
  };
  if (label && data.right) {
    label.textContent = "Mod detectare: " + (nameMap[data.right] || data.right.toUpperCase());
  }
});
