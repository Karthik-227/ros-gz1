# Missile Guidance Computer Assignments: Parameter Reference

This document outlines the specific parameters, initial conditions, and guidance laws for each computer assignment based on the provided source file.

## 1. Deviated Pursuit 
**Source:** Assignment 2, Question 2[cite: 1]

### Part A: Ideal Deviated Pursuit
* **Deviation Angle ($\delta$):** $25^\circ$[cite: 1]
* **Speed Ratios ($\nu = V_M/V_T$):** $0.9, 1.5, 3.0$[cite: 1]
* **Target Maneuver:** Non-maneuvering (assumed, as no $a_T$ is given)[cite: 1]

### Part B: Implementation of Deviated Pursuit
* **Guidance Law:** $a_M = V_M \dot{\theta} - k(\alpha_M - \theta - \delta)$[cite: 1]
* **Deviation Angle ($\delta$):** $8^\circ$[cite: 1]
* **Initial Geometry:** The initial missile velocity vector lags the Line of Sight (LOS) by $10^\circ$[cite: 1]
* **Speed Ratios ($\nu$):** $0.9, 1.5, 3.0$[cite: 1]
* **Implementation Gain ($k$):** $1.5, 10$[cite: 1]

---

## 2. Pure Pursuit
**Source:** Pure Pursuit section[cite: 7] and Assignment Question 3[cite: 6, 8]

### Part A: Ideal Pure Pursuit
* **Deviation Angle ($\delta$):** $0^\circ$[cite: 7]
* **Target Angle ($\alpha_T$):** $\in [0^\circ, 180^\circ]$[cite: 7]
* **Speed Ratios ($\nu = V_M/V_T$):** $0.9, 1.5, 3.0$[cite: 7]

### Part B: Implementation of Pure Pursuit (C1)
* **Guidance Law:** $a_M = V_M \dot{\theta} - k(\alpha_M - \theta)$[cite: 7]
* **Initial Geometry:** The initial missile velocity vector lags the LOS by $10^\circ$[cite: 7]
* **Speed Ratios ($\nu$):** $0.9, 1.5, 3.0$[cite: 7]

### Advanced Pure Pursuit (Against Maneuvering Target)
* **Speed Ratios ($\nu = V_P/V_T$):** $0.8, 1.0, 1.5$[cite: 6]
* **Target Turn Rate ($\dot{\alpha}_T$):** $\pm \pi/6$ rad/sec[cite: 6] or $c_T \dot{\theta}$ where $c_T = 0.8, 1.0, 1.5$[cite: 8]
* **Initial Range ($R_0$):** $5000$ m[cite: 6]
* **Initial Angles:** $\psi_0 = \pi/3$, $a_{P_0} = \theta_0$[cite: 6]

---

## 3. True Proportional Navigation (TPN)
**Source:** TPN Assignment[cite: 4]

* **Initial Geometry:** $R = 7$ km, Target angle $= 60^\circ$, Reference LOS angle $= 30^\circ$[cite: 4]
* **Missile Velocity ($V_M$):** $400$ m/s[cite: 4]
* **Target Velocity ($V_T$):** Between $0.5$ and $0.7$ of $V_M$[cite: 4]
* **Guidance Law:** $a_M = c\dot{\theta}$ where $c = -3V_M$[cite: 4]
* **Target Maneuver (Part C):** $a_T = 30$ m/s$^2$ in the clockwise direction[cite: 4]

---

## 4. Pure Proportional Navigation (PPN)
**Source:** PPN Assignment[cite: 5]

* **Initial Geometry:** $R = 7$ km, Target angle $= 60^\circ$, Reference LOS angle $= 30^\circ$[cite: 5]
* **Missile Velocity ($V_M$):** $400$ m/s[cite: 5]
* **Target Velocity ($V_T$):** Between $0.5$ and $0.7$ of $V_M$[cite: 5]
* **Guidance Law:** $a_M = NV_M\dot{\theta}$[cite: 5]
* **Navigation Constant ($N$):** Select a value between $3$ and $6$[cite: 5]
* **Target Maneuver (Part 2):** $a_T = 30$ m/s$^2$ clockwise[cite: 5]

---

## 5. Beam Rider (BR) & Command to Line of Sight (CLOS)
**Source:** Assignment 2-3rd question[cite: 2]

### Beam Rider (BR) Missile
* **Speed Ratio ($V_T/V_M$):** $0.6$[cite: 2]
* **Guidance Law (Without Autopilot):** $a_M = KR_M(\theta_T - \theta_M)$[cite: 2]
* **Gain ($K$):** $1, 5, 10$[cite: 2]
* **Target Maneuver (Part B):** $3g$ pitch-up constant target maneuver (for $K=10$)[cite: 2]

### Command to Line of Sight (CLOS) Missile
* **Speed Ratio ($V_T/V_M$):** $0.6$[cite: 2]
* **Gain ($K$):** $1, 5, 10$[cite: 2]
* **Guidance Law (Without Autopilot):** $a_M = KR_M(\theta_T - \theta_M) + R_M \ddot{\theta}_T + 2\dot{R}_M\dot{\theta}_T$[cite: 2]
* **Target Maneuver:** Evaluated for both a non-maneuvering target and a $3g$ pitch-up maneuver[cite: 2]