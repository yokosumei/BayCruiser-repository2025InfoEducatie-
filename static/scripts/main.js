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

  canShowMessage = false; // dezactivează întrebarea
});

btn.addEventListener("click", () => {
  // Resetează tot
  canShowMessage = true;

  // Șterge întrebarea veche (dacă cumva a rămas)
  const oldPanel = document.querySelector(".msgBox");
  if (oldPanel) oldPanel.remove();

  // Șterge răspunsurile vechi
  const oldResponses = document.querySelectorAll(".responseBox");
  oldResponses.forEach(resp => resp.remove());

  // Afișează din nou mesajul
  displayMessage();
});

function displayMessage() {
  if (!canShowMessage) return;

  const body = document.body;

  const panel = document.createElement("div");
  panel.setAttribute("class", "msgBox");

  const question = document.createElement("p");
  question.textContent = "Este alarma reala?";
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
///pentru info
function openContent(id) {
  var tabs = document.getElementsByClassName("tabcontent");
  for (var i = 0; i < tabs.length; i++) {
    tabs[i].style.display = "none";
  }

  var content = document.getElementById(id);
  if (content.style.display === "block") {
    content.style.display = "none"; // Dacă e deja afișat, îl ascunde
  } else {
    content.style.display = "block"; // Altfel, îl arată
  }
}
///cod pt pi 
///cd ~/yolo-stream-app
///pip3 install -r requirements.txt
///python3 stream_app.py  exemplu de output:Running on http://0.0.0.0:5000
///taburi :))
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
    container.classList.remove('active'); //papa containerul
    return;
  }

  document.getElementById(tabId).classList.add('active');
  container.classList.add('active');

  //butonul este activat
  const activeButton = Array.from(buttons).find(btn => btn.textContent.toLowerCase() === tabId.toLowerCase());
  if (activeButton) activeButton.classList.add('active');
}
function toggleStream() {
  fetch('http://192.168.50.119:5000/toggle_stream')
    .then(response => response.text())
    .then(status => alert(status))
    .catch(error => console.error('Eroare:', error));
}

// Verificare automată dacă s-a detectat 'sample'
setInterval(() => {
  fetch('http://192.168.50.119:5000/detection_status')
    .then(res => res.json())
    .then(data => {
      if (data.detected) {
        // Simulează click pe butonul de popup
        document.getElementById('popup-button').click();
      }
    });
}, 1000);

function startStream() {
  fetch("http://192.168.50.119:5000/start_stream")
    .then(res => res.text())
    .then(msg => alert("Stream pornit: " + msg))
    .catch(err => console.error("Eroare la pornire stream:", err));
}

function stopStream() {
  fetch("http://192.168.50.119:5000/stop_stream")
    .then(res => res.text())
    .then(msg => alert("Stream oprit: " + msg))
    .catch(err => console.error("Eroare la oprire stream:", err));
}

setInterval(() => {
  fetch("http://<IP-ul-PI-ului>:5000/check_detection")
    .then(response => response.json())
    .then(data => {
      if (data.sample_detected) {
        showPopup(); 
      }
    });
}, 1000); // check that nigg
function startStream() {
  fetch('/start_stream');
  const stream = document.getElementById("yoloStream");
  stream.src = "/video_feed";
  stream.style.display = "block";
}

function stopStream() {
  fetch('/stop_stream');
  const stream = document.getElementById("yoloStream");
  stream.src = "";
  stream.style.display = "none";
}
