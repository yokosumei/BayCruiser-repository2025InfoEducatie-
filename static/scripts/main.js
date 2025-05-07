let canShowMessage = true;
let streamActive = false;

const myImage = document.querySelector("img");
const btn = document.querySelector("#omgButton");

myImage.addEventListener("click", () => {
  const mySrc = myImage.getAttribute("src");
  if (mySrc === "images/firefox-icon.png") {
    myImage.setAttribute("src", "images/bleah-kitty.jpg");
  } else {
    myImage.setAttribute("src", "images/firefox-icon.png");
  }

  canShowMessage = false;
});

btn.addEventListener("click", () => {
  canShowMessage = true;
  document.querySelectorAll(".msgBox, .responseBox").forEach(el => el.remove());
});

function displayMessage() {
  if (!canShowMessage) return;

  const panel = document.createElement("div");
  panel.className = "msgBox";

  const question = document.createElement("p");
  question.textContent = "Este alarma reală?";
  panel.appendChild(question);

  const btnContainer = document.createElement("div");
  btnContainer.className = "btnContainer";

  const daBtn = document.createElement("button");
  daBtn.textContent = "DA";
  daBtn.onclick = () => {
    showResponse("oh nu!");
    panel.remove();
    canShowMessage = false;
    document.getElementById("demo").textContent = "mac";
  };

  const nuBtn = document.createElement("button");
  nuBtn.textContent = "NU";
  nuBtn.onclick = () => {
    showResponse("yay");
    panel.remove();
    canShowMessage = false;
    document.getElementById("demo").textContent = "mac mac";
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

  const activeButton = Array.from(buttons).find(btn => btn.textContent.toLowerCase() === tabId.toLowerCase());
  if (activeButton) activeButton.classList.add('active');
}

function startStream() {
  fetch('/start_stream')
    .then(() => {
      const stream = document.getElementById("yoloStream");
      stream.src = "/video_feed";
      stream.style.display = "block";
      streamActive = true;
    })
    .catch(err => console.error("Eroare pornire stream:", err));
}

function stopStream() {
  fetch('/stop_stream')
    .then(() => {
      const stream = document.getElementById("yoloStream");
      stream.src = "";
      stream.style.display = "none";
      streamActive = false;
    })
    .catch(err => console.error("Eroare oprire stream:", err));
}

// Poll server for detections
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
