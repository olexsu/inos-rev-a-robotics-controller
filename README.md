# inos-rev-a-robotics-controller
INOS Rev A — CAN-based robotic arm controller designed as a real-time motion-control node for NVIDIA Jetson robotics systems.

# INOS Rev A – Intelligent Networked Open Robotics

INOS Rev A is a custom robotics controller designed to operate as a dedicated real-time motion-control node for NVIDIA Jetson-based robotic systems.

The platform follows a distributed architecture where the Jetson performs high-level tasks such as computer vision, motion planning, and AI processing, while the INOS controller executes deterministic real-time functions including motion control, homing, safety monitoring, and hardware management.

## Key Features

* Teensy 4.1 real-time controller
* CAN bus communication with NVIDIA Jetson
* 24V industrial power architecture
* Six-axis motion control
* External stepper driver support
* Inductive sensor integration
* Custom 4-layer PCB designed in KiCad
* Modular architecture for future robotics platforms

## System Architecture

PS4 Controller / Vision System

↓

NVIDIA Jetson (Master Computer)

↓ CAN Bus

INOS Controller (Real-Time Motion Controller)

↓ STEP/DIR

External Driver Board

↓

Motors & Sensors

## Design Philosophy

INOS is built around the principle of separating high-level intelligence from deterministic real-time control.

Jetson Responsibilities:

* Computer vision
* AI processing
* Motion planning
* User interface

INOS Controller Responsibilities:

* Motion control
* Homing routines
* Safety monitoring
* Hardware management
* Real-time CAN communication

## Current Status

INOS Rev A is an experimental prototype and active development platform.

Current hardware includes:

* Custom 4-layer controller PCB
* Teensy 4.1
* CAN communication
* Six-axis motion support
* 24V power architecture
* NEMA stepper motor integration

## Project Goals

The long-term goal of INOS is to develop a scalable robotics platform capable of supporting robotic arms, agricultural automation, machine vision systems, and future autonomous robotic applications.

