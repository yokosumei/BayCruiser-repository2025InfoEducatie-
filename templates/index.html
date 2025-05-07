<!DOCTYPE html>
<html lang="en">
<html lang="en-US">
<head>
    <meta charset="UTF-8">
    <title>YOLO Stream</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            background-color: #1e1e1e;
            color: #fff;
            text-align: center;
            margin: 0;
            padding: 0;
        }
        h1 {
            margin-top: 20px;
        }
        #stream {
            margin-top: 20px;
            width: 80%;
            max-width: 800px;
            border: 4px solid #555;
            border-radius: 10px;
        }
        .buttons {
            margin-top: 15px;
        }
        button {
            padding: 10px 20px;
            margin: 0 10px;
            font-size: 16px;
            border: none;
            border-radius: 5px;
            background-color: #444;
            color: #fff;
            cursor: pointer;
        }
        button:hover {
            background-color: #666;
        }
        #popup {
            display: none;
            position: fixed;
            top: 30%;
            left: 50%;
            transform: translate(-50%, -50%);
            background: #333;
            padding: 20px;
            border-radius: 10px;
            font-size: 24px;
            color: #fff;
            border: 2px solid #fff;
        }
    </style>
  <meta charset="UTF-8" />
  <meta name="viewport" content="width=device-width" />
  <title>kITTY</title>
  <link href="styles/style.css" rel="stylesheet" />
  <link rel="stylesheet" href="{{ url_for('static', filename='styles/style.css') }}">
  <style>
    .stream-wrapper {
      display: flex;
      justify-content: center;
      padding: 10px;
    }

    #yoloStream {
      max-width: 100%;
      border: 2px solid black;
    }

    .stream-controls {
      display: flex;
      gap: 10px;
      align-items: center;
      justify-content: center;
      padding: 15px;
    }
  </style>
</head>
<body>
    <h1>YOLO Stream Test</h1>
    <img id="stream" src="{{ url_for('video_feed') }}" alt="Live Stream">

    <div class="buttons">
        <button onclick="startStream()">Start Stream</button>
        <button onclick="stopStream()">Stop Stream</button>
    </div>
  <header>
    <h1>Pagina de control</h1>
  </header>

  <main class="main-layout">
    <!-- Coloana stângă: taburi -->
    <aside id="sidebar">
      <div class="tab-container">
        <button class="tab-button" onclick="toggleTab('info')">Info</button>
        <button class="tab-button" onclick="toggleTab('scop')">Scop</button>
        <button class="tab-button" onclick="toggleTab('alte butoane')">Butoane+</button>
      </div>
      <link rel="stylesheet" href="{{ url_for('static', filename='styles/style.css') }}">
      <script src="{{ url_for('static', filename='scripts/main.js') }}"></script>
      <div class="tab-content-wrapper">
        <div id="info" class="tab-content">
          <p>Aceasta pagina este conceputa ca o punte dintre drona si salvamar</p>
          <p>more info idkFEATURES ig</p>
        </div>
        <div id="scop" class="tab-content">
          <p>De multe ori salvamarii au nevoie de ajutor pe mare si de o interventie rapida
            insa de multe ori nu pot ajunge la victima in timp util. Aici intervine drona, care 
            este capabila sa ajunga la victima in cateva secunde si sa ii ofere un colac de salvare.
          </p>
        </div>
        <div id="alte butoane" class="tab-content">
          <p>probabil aici sunt alte butoane idk</p>
          <button class="idk buton" onclick="alert('nu stiu ce face')">buton</button>
        </div>
      </div>
    </aside>

    <!-- Dreapta: doar stream -->
    <!-- Dreapta: doar stream -->
<article class="custom-article">
  <div class="stream-wrapper">
    <img id="yoloStream" src="{{ url_for('video_feed') }}" alt="YOLO Stream">
  </div>
  <div id="popup" style="display:none;
        position: fixed;
        top: 30%;
        left: 50%;
        transform: translate(-50%, -50%);
        background: #333;
        padding: 20px;
        border-radius: 10px;
        font-size: 24px;
        color: #fff;
        border: 2px solid #fff;">
    Obiectul „sample” a fost detectat!
  </div>
</article>

  </main>

    <div id="popup">Obiectul „sample” a fost detectat!</div>
  <!-- Butoanele sub articol, deasupra footerului -->
  <div class="stream-controls">
    <button id="omgButton">omg merge?</button>
    <button onclick="startStream()">Start Stream</button>
    <button onclick="stopStream()">Stop Stream</button>
  </div>

  <footer>
    <p>Pagina creata in scopul concursului IndoEducatie</p>
  </footer>

  <script src="scripts/main.js"></script>
  <script src="{{ url_for('static', filename='scripts/main.js') }}"></script>
    <script>
        let popupShown = false;
  let popupShown = false;

        function startStream() {
            fetch('/start_stream');
        }
  function startStream() {
    fetch('/start_stream');
  }

        function stopStream() {
            fetch('/stop_stream');
        }
  function stopStream() {
    fetch('/stop_stream');
  }

        function checkDetection() {
            fetch('/detection_status')
                .then(res => res.json())
                .then(data => {
                    if (data.detected && !popupShown) {
                        const popup = document.getElementById("popup");
                        popup.style.display = "block";
                        popupShown = true;
                        setTimeout(() => {
                            popup.style.display = "none";
                            popupShown = false;
                        }, 2000);
                    }
                });
  function checkDetection() {
    fetch('/detection_status')
      .then(response => response.json())
      .then(data => {
        if (data.detected && !popupShown) {
          const popup = document.getElementById("popup");
          popup.style.display = "block";
          popupShown = true;
          setTimeout(() => {
            popup.style.display = "none";
            popupShown = false;
          }, 2000);
        }
      });
  }

  setInterval(checkDetection, 1000);
</script>

        setInterval(checkDetection, 1000);
    </script>
</body>
</html>
