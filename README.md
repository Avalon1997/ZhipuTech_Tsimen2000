## Tsimen2000 Introduction
Tsimen2000 is a spectral water quality online monitoring system, which uses spectral technology to analyze the spectral characteristics of its penetrating substance (water), so as to monitor the content of certain specific substances in the water and its sensitive changes over time. 

The sensor monitor water quality changes in real time and in-situ, and current and historical water quality data can be viewed on the central control system, which will also upload the data to the cloud server. Customers can view real-time water quality changes on the water quality large-screen website, and monitor the current equipment status on the backend management website.

At present, it is mainly used in the fields of wastewater treatment in refineries, wastewater treatment in water plants, and surface water environment monitoring.

**Note:**  This repository is only the tsimen sensor code. The central control system is developed by other colleagues.

## Tsimen2000 Code Module Introduction

Tsimen's sensor code is divided into five parts: spectrometer communication module, host computer communication module, temperature and humidity sensor module, built-in flash storage module, digital servo motor control module.

### Spectrometer Communication Module
This module is used to communicate with the micro-spectrometer, issue instructions to set the spectrometer acquisition parameters, receive raw spectral data, set the acquisition mode, etc.

- **Implementation method:** Use the USART hardware of STM32 and follow the communication instructions of the spectrometer to communicate with the spectrometer via RS232 protocol. And there is modebus-CRC16 check at the end of the command.

### Host Computer Communication Module
This module is used for data communication with the host computer, receive commands from the host computer and generate corresponding actions. Such as collecting temperature and humidity, configuring the spectrometer intergration constant and average times and writing the optical path or algorithm cofficients.

- **Implementation method:** Also use the USART hardware of STM32 and follow the communication instructions that I designed to communicate with the host computer via RS485 protocol. And there is modebus-CRC16 check at the end of the command.

### Tempereture And Humidity Sensor Module
This module is designed for read the T&H inside the sensor by IIC protocal, to monitor the sensor status and determine whether the sensor has abnormal conditions such as water ingress.

- **Implementation method:** Use the GPIO to simulate IIC protocal to read the T&H.

### Built-in Flash Storage Module
Due to the SRAM is mightly not enough to store a large number of parameters, I extracted six pages from the built-in flash memory to store these parameters such as algorithm coefficients which typically do not change frequently.

### Digital Servo Motor Control Module
The sensor is a dual-optical path design, and light can only be transmitted through one optical path at a time, so a digital servo is needed to switch the optical path.

- **Implementation method:** Use the timer which can generate PWM of STM32 to control the angle of the digital servo.







