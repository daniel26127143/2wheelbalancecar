# Design of an Omnidirectional-Wheel Self-Balancing Robot



An underactuated, two-wheel inverted pendulum robot driven by collinear Mecanum wheels. Built upon an ATmega328P architecture, featuring hardware-timed deterministic multi-rate cascade PID balance control and dynamic decoupling for future omnidirectional expansion.

[![Demo Video](https://img.shields.io/badge/YouTube-Video%20Demo-red?logo=youtube)](https://www.youtube.com/shorts/cfTprMj1Mzs)
[![Platform](https://img.shields.io/badge/Platform-Arduino%20Uno-blue.svg)](https://www.arduino.cc/)


[English](README.md) | [繁體中文](README_zh.md)

---

## 📌 Project Overview

This project implements a self-balancing inverted pendulum utilizing collinear Mecanum wheels. The primary engineering objective is to solve the non-linear dynamics of an underactuated 2-wheel balance core before mechanically expanding to a 3-wheel platform that unlocks pure lateral planar crabbing without body orientation adjustments.

---

## 🚀 Key Validated Benchmarks

- **Continuous Stationary Balance**: Achieved **> 90 seconds** uninterrupted upright balance without positional drift[cite: 6].
- **Precision Upright Neutral**: Steady-state pitch variance tightly bounded within **±0.4°**[cite: 6].
- **Deterministic Multi-Rate Scheduling**: High-frequency balance loop executed strictly at **10.13 ms (98.7 Hz)** via ATmega328P Timer2 CTC interrupts[cite: 6].
- **Low-Speed Stiction Elimination**: Feedforward deadband compensation offset suppresses non-linear motor hunting at zero-crossing[cite: 6].

---

## ⚙️ Control Firmware & Architecture

The control system adopts a decoupled, dual-rate cascade architecture:

### 1. Balance Inner Loop (10.13 ms / 98.7 Hz)[cite: 6]
- **Sensor Input**: InvenSense MPU-6050 on-chip Digital Motion Processor (DMP) providing 6-axis attitude fusion.
- **Control Law**: Proportional-Derivative (PD) control stabilizing pitch angle and angular velocity[cite: 6].
- **Verified Gains**:
  - $K_p = 1275$[cite: 6]
  - $K_d = 70$[cite: 6]

### 2. Velocity Outer Loop (40.50 ms / 24.7 Hz)[cite: 6]
- **Sensor Input**: Dual quadrature magnetic encoders measuring cumulative wheel rotation.
- **Control Law**: Proportional-Integral (PI) control dynamically tuning equilibrium neutral pitch angle offset.
- **Verified Gains**:
  - $K_p = 2.8$[cite: 6]
  - $K_i = 0.005$[cite: 6]

---

## 🛠️ Hardware Specifications & Pinout

| Subsystem | Component / Specification | Pin Assignment / Interface |
| :--- | :--- | :--- |
| **Main MCU** | Microchip ATmega328P (16 MHz) | Onboard Hardware Timer2 CTC |
| **IMU Sensor** | InvenSense MPU-6050 (DMP) | I2C (A4/SDA, A5/SCL) + Ext INT0 (D2) |
| **Motor Driver** | TB6612FNG Dual H-Bridge | PWM: D5, D6 \| Dir: D4, D7, D8, D9 |
| **Actuators** | 2× Micro DC Metal Gear Motors | Hall Magnetic Encoders (Phase A/B) |
| **Wheel Base** | 2× 65mm Collinear Mecanum Wheels | High-traction rubber rollers |

---
