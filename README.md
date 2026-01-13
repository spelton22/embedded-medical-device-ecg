# Embedded Medical Device: ECG & Temperature Sensor with BLE

This repository contains the firmware and analysis for an embedded medical device developed as the final project for Duke University's graduate level *Embedded Medical Devices (BME 544L)* course.

The system implements a wireless ECG and temperature sensing device using Zephyr RTOS, with Bluetooth Low Energy (BLE) communication and a state-machine-based firmware architecture.

## Features
- Zephyr RTOS–based firmware architecture
- Event-driven state machine design
- ECG signal acquisition and heart rate estimation
- I2C temperature sensing
- BLE notifications for heart rate, temperature, battery level, and errors
- Robust error handling and logging
- Data-driven verification using oscilloscope measurements and BLE outputs

## Technologies
- C / Zephyr RTOS
- Nordic nRF52 platform
- BLE (GATT services and characteristics)
- ADC, GPIO, PWM, I2C
- Python (NumPy, Pandas, Matplotlib)
- Jupyter Notebook for analysis and verification

## Repository Structure
- `application/`: Embedded firmware source code
- `state_diagram/`: State diagram chart showing states and transitions of device
- `final_testing/ECG_temp_analysis/raw_data/`: Raw oscilloscope captures
- `final_testing/ECG_temp_analysis/report/`: Jupyter notebooks for signal analysis and validation
- `final_testing/ECG_temp_analysis/images/`: Generated plots and figures

## Analysis
The `final_testing/ECG_temp_analysis/report/final_technical_report.ipynb` notebook demonstrates:
- Visualization of raw ECG signals captured from an oscilloscope
- Peak detection for heart rate estimation
- Comparison between firmware output, oscilloscope measurements, and BLE-reported data

## Notes
This project was developed for academic purposes. Hardware-specific configuration may need adjustment for different platforms.
