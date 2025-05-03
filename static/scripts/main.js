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
