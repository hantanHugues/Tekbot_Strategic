# TEKBOT Control Center

## Overview

TEKBOT Control Center is a desktop application built for the TEKBOT Robotics Challenge 2025. It serves as the central command hub for managing and supervising an automated waste sorting system consisting of a mobile robot (Robomaster X3), a sorting station with conveyor belt, and a robotic arm (Dofbot Jetson Nano). The application provides real-time monitoring, strategic planning, calibration tools, and score calculation during the 5-minute competition matches.

## Recent Changes

**September 08, 2025**: Completed initial application development
- Implemented modern Vue.js 3 interface with professional dark theme design
- Created complete Dashboard de Match with real-time monitoring, timer, and score tracking
- Built comprehensive Calibration Module for LDR sensors and color detection
- Developed Mission Planning interface with QR code scanning and route optimization
- Added System Health monitoring with ROS2 node status and diagnostics
- Integrated ROSLib.js service for ROS2 communication (ready for connection)
- Configured Electron.js desktop application with proper security and performance settings

## User Preferences

Preferred communication style: Simple, everyday language.
Language preference: French

## System Architecture

### Frontend Architecture
- **Framework**: Vue.js 3 with Composition API for reactive user interfaces
- **Desktop Platform**: Electron.js for cross-platform desktop application deployment
- **Build System**: Vite for fast development and optimized production builds
- **Styling**: Custom CSS with CSS variables for consistent theming and dark mode design
- **Charts**: Chart.js with Vue-ChartJS wrapper for real-time data visualization

### Backend/Communication Architecture
- **ROS Integration**: ROSLib.js for WebSocket-based communication with ROS2 nodes
- **Real-time Communication**: WebSocket connection to ROS bridge server (default: ws://localhost:9090)
- **Process Architecture**: Electron main/renderer process separation with secure IPC communication
- **Security**: Context isolation enabled with preload scripts for secure API exposure

### Development Architecture
- **Development Server**: Concurrent development with Vite dev server and Electron
- **Hot Reload**: Live reloading for both frontend and Electron components
- **Build Pipeline**: Separate build processes for web assets and Electron packaging
- **Asset Management**: Path resolution with aliases for organized asset structure

### Application Structure
- **Modular Services**: Centralized ROSService for managing robot communication
- **Component-Based UI**: Vue.js single-file components for modular interface elements
- **Event-Driven**: Reactive data flow using Vue's reactivity system
- **Configuration Management**: Environment-based configuration for development/production

## External Dependencies

### Core Technologies
- **Electron**: Desktop application framework for cross-platform deployment
- **Vue.js 3**: Progressive JavaScript framework for building user interfaces
- **Vite**: Next-generation frontend build tool for fast development
- **ROS2**: Robot Operating System for robot communication and control

### Visualization and UI
- **Chart.js**: JavaScript charting library for real-time data visualization
- **Vue-ChartJS**: Vue.js wrapper for Chart.js integration

### ROS Communication
- **ROSLib.js**: JavaScript library for ROS WebSocket communication
- **ROS Bridge**: WebSocket server bridge for ROS2 topic/service communication

### Development Tools
- **Concurrently**: Tool for running multiple npm scripts simultaneously
- **Wait-on**: Utility for waiting on resources (TCP ports, files, etc.)
- **Electron Builder**: Complete solution for packaging Electron applications

### Robot Ecosystem Integration
- **Robomaster X3**: Mobile robot platform for navigation and QR code scanning
- **Dofbot Jetson Nano**: Robotic arm for waste sorting operations
- **Arduino/ESP32**: Microcontroller for conveyor belt control
- **Various Sensors**: LDR sensors, color sensors, and laser sensors for object detection