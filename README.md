## 🔽 Ghiduri rapide

- [🧰 Setup complet](#setup)
- [🎥 Demo detectare](#demo)
- [🤖 Cum funcționează clasificarea](#xgb)

---

### <a name="setup"></a>🧰 Setup complet

<details>
  <summary><strong>Show instructions</strong></summary>

(… aici conținutul complet pentru setup …)

</details>

---

### <a name="demo"></a>🎥 Demo detectare

<details>
  <summary><strong>Show demo</strong></summary>

(… demo cu poză + descriere …)

</details>

---

### <a name="xgb"></a>🤖 Cum funcționează clasificarea

<details>
  <summary><strong>Explică-mi clasificarea cu XGBoost</strong></summary>

Modelul XGBoost primește vectori 1020 (30 frameuri × 34 coordonate) extrași cu YOLO Pose.  
Apoi decide între două clase: `inec` și `inot`.  
Dacă se detectează `inec`, sistemul:
- Activează popupul de confirmare
- Salvează timestamp + coordonate
- Pornește drona

</details>
