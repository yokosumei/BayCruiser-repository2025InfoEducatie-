# BayCruiser 🚁🌊

[![Status](https://img.shields.io/badge/status-active-brightgreen?style=flat-square)]()
[![Platform](https://img.shields.io/badge/platform-RaspberryPi4-blue?style=flat-square)]()
[![Python](https://img.shields.io/badge/python-3.11+-blue?style=flat-square)]()
[![AI Model](https://img.shields.io/badge/YOLOv11+Pose+XGBoost-functional-orange?style=flat-square)]()
[![License](https://img.shields.io/badge/license-MIT-yellow?style=flat-square)]()

> Dronă autonomă de salvare echipată cu inteligență artificială, rulată în timp real pe Raspberry Pi 4B. Detectează persoane în pericol de înec și reacționează automat.

![](media/overview.jpg)

---

## 📜 Descriere generală

**BayCruiser** este un sistem AI + dronă + interfață web pentru prevenirea înecurilor. Detectează persoane și analizează comportamentul (înot vs. înec) folosind YOLOv11 Pose + XGBoost. În caz de pericol, activează drona autonom, transmite video live și salvează poziția GPS.

Sistemul este modular, rulează local și este adaptat pentru salvare acvatică inteligentă.

---

## 🚀 Getting Started

Instrucțiuni rapide pentru a rula proiectul pe Raspberry Pi sau PC.

### ✅ Prerequisites

- Python 3.11+
- Raspberry Pi 4B
- Cameră (ex: PiCamera2)
- Dronă compatibilă cu MAVLink
- Servo conectat la GPIO
- YOLOv11 Pose model (NCNN) + XGBoost

---

### 🔧 Installing

```bash
git clone https://github.com/username/BayCruiser.git
cd BayCruiser
pip install -r requirements.txt
python main.py
