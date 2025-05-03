let canShowMessage = true;

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

  const oldPanel = document.querySelector(".msgBox");
  if (oldPanel) oldPanel.remove();

  const oldResponses = document.querySelectorAll(".responseBox");
  oldResponses.forEach(resp => resp.remove());

  displayMessage();
});

function displayMessage() {
  if (!canShowMessage) return;

  const body = document.body;

  const panel = document.createElement("div");
  panel.setAttribute("class", "msgBox");

  const question = document.createElement("p");
  question.textContent = "Este alarma reală?";
  panel.appendChild(question);

  const btnContainer = document.createElement("div");
  btnContainer.setAttribute("class", "btnContainer");

  const daBtn = document.createElement("button");
  daBtn.textContent = "DA";
  btnContainer.appendChild(daBtn);

  const nuBtn = document.createElement("button");
  nuBtn.textContent = "NU";
  btnContainer.appendChild(nuBtn);

  panel.appendChild(btnContainer);
  body.appendChild(panel);

  daBtn.addEventListener("click", () => {
    showResponse("oh nu!");
    panel.remove();
  });

  nuBtn.addEventListener("click", () => {
    showResponse("yay");
    panel.remove();
  });
}

function showResponse(text) {
  const body = document.body;

  const responseBox = document.createElement("div");
  responseBox.setAttribute("class", "responseBox");

  const p = document.createElement("p");
  p.textContent = text;
  responseBox.appendChild(p);

  body.appendChild(responseBox);
}

function handleSampleDetection() {
  const result = confirm("Obiectul sample a fost detectat. Continuați?");
  const demo = document.getElementById("demo");
  if (result) {
    demo.textContent = "mac";
  } else {
    demo.textContent = "mac mac";
  }
}

// Tabs
function openContent(id) {
  var tabs = document.getElementsByClassName("tabcontent");
  for (var i = 0; i < tabs.length; i++) {
    tabs[i].style.display = "none";
  }

  var content = document.getElementById(id);
  content.style.display = content.style.display === "block" ? "none" : "block";
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

// Stream control
let streamActive = false;

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
// Simulare detecție obiect "sample"
setInterval(() => {
  if (!streamActive) return;

  fetch("/detection_status")
    .then(res => res.json())
    .then(data => {
      if (data.detected) {
        displayMessage(); // apelează funcția deja existentă
      }
    })
    .catch(err => console.warn("Eroare verificare detecție:", err));
}, 1000);

