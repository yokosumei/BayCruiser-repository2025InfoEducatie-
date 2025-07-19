# BayCruiser 
[![Status](https://img.shields.io/badge/Info-Educatie-brightgreen?style=flat-square)]()
[![Platform](https://img.shields.io/badge/platform-RaspberryPi4-blue?style=flat-square)]()
[![Python](https://img.shields.io/badge/python-3.11+-blue?style=flat-square)]()
[![Drone](https://img.shields.io/badge/Drone-Robot+-blue?style=flat-square)]()
[![AI Model](https://img.shields.io/badge/YOLO11-XGBoost-orange?style=flat-square)]()

> Dronă autonomă de salvare echipată cu inteligență artificială, rulată în timp real pe Raspberry Pi 4B. Detectează persoane în pericol de înec și reacționează automat.

![](media/overview.jpg)  
🎥 *Video demonstrativ — va fi adăugat în `media/`*

---

## Descriere generală

**BayCruiser** este un sistem AI + dronă + interfață web pentru prevenirea înecurilor. Detectează persoane și analizează comportamentul (înot vs. înec) folosind YOLOv11 Pose + XGBoost. În caz de pericol, activează drona autonom, transmite video live și salvează poziția GPS.

Sistemul este modular, rulează local și este adaptat pentru salvare acvatică inteligentă.

---

## Foldere proiect

- [Setup complet](#setup)
- [Demo - poze & filmări](#demo)
- [Cum funcționează clasificarea](#xgb)

---

### <a name="setup"></a>Setup complet

<details>
  <summary><strong>Show instructions</strong></summary>

Sistemul este organizat în mai multe componente funcționale, fiecare având rol clar:

- interfață web (Flask + SocketIO)
- stream video (raw + AI)
- procesare AI în thread-uri paralele
- control dronă cu DroneKit
- salvare coordonate, reacție automată și activare servo

Arhitectura este scalabilă și permite extinderea cu noi module.

</details>

---

### <a name="demo"></a>Demo - poze & filmări

<details>
  <summary><strong>Show demo</strong></summary>

#### 📸 Detecție în timp real

![](media/yolo_detect.jpg)  
> YOLOv11 detectează persoane, adâncime, meduze, rechini și curenți periculoși.

---

#### 🧠 Estimare poziție + clasificare

![](media/pose_frame.jpg)  
> YOLO Pose extrage 34 keypoints umane, analizate apoi de modelul XGBoost pentru a decide dacă persoana înoată normal sau este în pericol.

![](media/pose_xgb.gif)

---

#### 🚁 Reacție automată + control dronă

![](media/drone_flight.jpg)  
> Drona răspunde la detecții confirmate:
- execută decolare automată
- orbitează în jurul punctului detectat
- eliberează flotorul cu servomotor

---

#### 🌐 Interfață intuitivă

![](media/web_interface.jpg)  
> Interfața web include stream video live, comenzi dronei și popup-uri automate în caz de detecție `inec`.

</details>

---

### <a name="xgb"></a>Modele AI folosite

<details>
  <summary><strong>Explică-mi clasificarea cu XGBoost</strong></summary>

Modelul XGBoost primește vectori de 1020 dimensiuni (30 frameuri × 34 coordonate x/y) extrași cu YOLOv11 Pose.

Pe baza mișcării corpului într-un interval de 3–4 secunde, modelul clasifică dacă persoana:
- 🟢 înoată normal
- 🔴 este în pericol (`inec`)

Dacă este detectat `inec`, sistemul:
- afișează popup de confirmare
- salvează timestamp + coordonate GPS
- trimite drona în zona periculoasă
- activează servomotorul pentru lansarea unui dispozitiv de salvare

📄 Modelele sunt descrise în [`AI_models/models_info.txt`](./AI_models/models_info.txt)

</details>

---
