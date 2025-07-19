# BayCruiser 
[![Status](https://img.shields.io/badge/Info-Educatie-brightgreen?style=flat-square)]()
[![Platform](https://img.shields.io/badge/platform-RaspberryPi4-blue?style=flat-square)]()
[![Python](https://img.shields.io/badge/python-3.11+-blue?style=flat-square)]()
[![Drone](https://img.shields.io/badge/Drone-Robot+-blue?style=flat-square)]()
[![AI Model](https://img.shields.io/badge/YOLO11-XGBoost-orange?style=flat-square)]()

> Dronă autonomă de salvare echipată cu inteligență artificială, rulată în timp real pe Raspberry Pi 4B. Detectează persoane în pericol de înec și reacționează automat.

![](media/overview.jpg)  
 *Video demonstrativ — va fi adăugat în `media/`*

---

## Descriere generală

**BayCruiser** este un sistem AI + dronă + interfață web pentru prevenirea înecurilor.
Dronă autonomă de salvare echipată cu inteligență artificială, rulată în timp real pe Raspberry Pi 4B.
Sistemul utilizează camere video și algoritmi de viziune computațională pentru a monitoriza suprafața apei în mod continuu. Modelele AI integrate – inclusiv YOLO pentru detecția de persoane și animale marine, un segmentator pentru analiza adâncimii și curenților de rupere, precum și un clasificator XGBoost antrenat pe mișcările corpului – permit identificarea rapidă a situațiilor de risc.

Atunci când o persoană aflată în pericol de înec este detectată, drona declanșează automat o reacție predefinită: transmite un semnal de alertă, activează un mecanism de salvare (cum ar fi lansarea unei veste de salvare) și se îndreaptă autonom spre locul incidentului, ghidată de coordonatele GPS înregistrate la momentul detecției.

Sistemul rulează în întregime local, fără dependențe de cloud, fiind optimizat pentru performanță și latență minimă. Datorită arhitecturii sale modulare și multitasking (cu threaduri dedicate pentru captură video, inferență AI, streaming și controlul dronei), aplicația poate funcționa în timp real chiar și pe hardware cu resurse limitate. Interfața web permite monitorizare, intervenție manuală și control complet asupra comportamentului dronei.

---

## Foldere proiect

- [Setup complet](#setup)
- [Sistemul software BayCruiser](#soft)
- [Controlul dronei](#drona)
- [Interfața web](#web)
- [Demo - poze & filmări](#demo)
- [Detectie](#xgb)

---

###<a name="setup"></a>Setup complet
<details> <summary><strong>Overview tehnic – logică internă și organizare</strong></summary>
BayCruiser este un sistem embedded construit în jurul unui Raspberry Pi 4B care rulează în timp real algoritmi de analiză video, control de dronă și interfață web. Sistemul funcționează complet local și include componente pentru captură video, procesare AI, control fizic și comunicare cu utilizatorul.

Arhitectura generală este modulară și multi-threaded. Fiecare sarcină critică (precum detecția video, transmiterea fluxului sau controlul dronei) rulează într-un fir de execuție separat. Astfel, sistemul poate capta în mod continuu imaginea de la cameră, o poate analiza prin mai multe modele AI, poate transmite video live către utilizatori și poate reacționa prin drone controlate autonom. Fiecare modul comunică prin structuri sincronizate, cu acces thread-safe, fără interferență între procese. Platforma este scalabilă și permite adăugarea ușoară de noi componente.

</details>

---
###<a name="soft"></a>Sistemul software BayCruiser
<details> <summary><strong>Software</strong></summary>
Sistemul software este împărțit în trei zone funcționale: captură și flux video, analiză AI și control fizic. Camera video este accesată printr-un thread care salvează fiecare frame cu timestamp și coordonate GPS. Aceste date sunt distribuite către mai multe fire de detecție, fiecare specializat pe un anumit model. Sistemul de detecție include fire pentru detecția de obiecte, segmentare semantică și analiză comportamentală prin XGBoost. Fiecare model AI este activat doar când este necesar, în funcție de context. De exemplu, clasificarea comportamentală pornește doar când se detectează o persoană.

Fluxurile video sunt transmise în paralel prin HTTP/MJPEG, iar statusul sistemului (baterie, poziție, conexiune, stare) este transmis prin WebSocket. Comenzile de zbor sunt gestionate printr-o coadă FIFO procesată secvențial de o mașină de stare internă, care garantează coerența acțiunilor dronei. Toate componentele funcționează asincron și sunt sincronizate prin Lock-uri, Event-uri și buffers dedicate. Sistemul este complet autonom dar permite și control semi-autonom sau manual prin interfață.

</details>

---
###<a name="drona"></a>Controlul dronei
<details> <summary><strong>Drona</strong></summary>
Controlul dronei este realizat printr-o clasă dedicată care comunică cu Pixhawk folosind protocolul MAVLink și biblioteca DroneKit. Toate comenzile sunt procesate de o mașină de stare internă care gestionează tranzițiile între stări precum IDLE, TAKEOFF, IN_AIR sau LANDING. Comenzile pot fi declanșate automat, în urma unui eveniment AI, sau manual prin interfață. Fiecare comandă este trimisă într-o coadă și procesată secvențial, fără suprapunere sau conflict.

Drona poate executa decolare, aterizare, zbor către un punct detectat, orbitare în jurul acestuia și patrulare autonomă. De asemenea, poate activa sisteme fizice precum servomotoare (pentru lansarea colacului), LED-uri adresabile și boxă activă pentru semnalizare sonoră. Comanda land are prioritate absolută și întrerupe orice altă acțiune în desfășurare. Sistemul garantează siguranța și predictibilitatea reacției în orice stare.

</details>
---
###<a name="web"></a>Website
<details> <summary><strong>Website</strong></summary>
Interfața web este găzduită local pe Raspberry Pi și oferă utilizatorului control complet asupra sistemului. Aceasta este implementată cu Flask și Flask-SocketIO și este accesibilă prin browser, fără instalare. Fluxurile video sunt afișate în timp real, cu posibilitatea de a comuta între streamuri raw și AI. Utilizatorul poate trimite comenzi de zbor, activa modele AI, confirma alerte sau vizualiza statusul sistemului (poziție, baterie, conexiune).

Comenzile sunt transmise prin WebSocket, iar fluxurile video prin HTTP/MJPEG. Interfața este proiectată să funcționeze pe desktop, tabletă sau telefon și include protecții împotriva comenzilor invalide, sincronizând totul cu mașina de stare internă. În regim semi-autonom, alertele generate de AI pot fi confirmate sau anulate de utilizator. În modul manual, operatorul are control total asupra dronei. Interfața este ușor extensibilă și se integrează complet cu restul sistemului BayCruiser.

</details>
---
### <a name="demo"></a>Demo - poze & filmări

<details>
  <summary><strong>Show demo</strong></summary>

#### Detecție în timp real

![](media/yolo_detect.jpg)  
> YOLOv11 detectează persoane, adâncime, meduze, rechini și curenți periculoși.

---

#### Estimare poziție + clasificare

![](media/pose_frame.jpg)  
> YOLO Pose extrage 34 keypoints umane, analizate apoi de modelul XGBoost pentru a decide dacă persoana înoată normal sau este în pericol.

![](media/pose_xgb.gif)

---

#### Reacție automată + control dronă

![](media/drone_flight.jpg)  
> Drona răspunde la detecții confirmate:
- execută decolare automată
- orbitează în jurul punctului detectat
- eliberează flotorul cu servomotor

---

####Interfață intuitivă

![](media/web_interface.jpg)  
> Interfața web include stream video live, comenzi dronei și popup-uri automate în caz de detecție `inec`.

</details>

---

### <a name="xgb"></a>Modele AI folosite

<details>
  <summary><strong>Modele AI</strong></summary>

Sistemul BayCruiser folosește o suită de modele AI specializate, optimizate pentru rulare locală pe Raspberry Pi. Detecția generală este realizată prin YOLOv11n, antrenat să recunoască persoane, meduze și rechini. Modelul de segmentare semantică YOLOv11n-Segmentation este folosit pentru identificarea curenților de rupere și zonelor de adâncime crescută. Estimarea poziției este realizată cu YOLOv11n-Pose, care extrage 17 keypoints per persoană. Pe baza acestor date, modelul XGBoost analizează 30 de cadre consecutive pentru a clasifica mișcarea ca fiind înot normal sau potențial înec.

Fiecare model este activat doar în contextul potrivit. Detecția de obiecte rulează constant, segmentarea și analiza comportamentală sunt declanșate pe baza detecțiilor. Datele sunt partajate între fire prin structuri sincronizate, iar rezultatele pot declanșa acțiuni automate, cum ar fi revenirea dronei la locul detectat sau activarea unui semnal de urgență. Modelele sunt complet integrate cu sistemul de reacție fizică și cu interfața web.

📄 Modelele sunt descrise în [`AI_models/models_info.txt`](./AI_models/models_info.txt)

</details>

---
