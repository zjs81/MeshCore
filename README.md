## About MeshCore

MeshCore is a lightweight, portable C++ library that enables multi-hop packet routing for embedded projects using LoRa and other packet radios. It is designed for developers who want to create resilient, decentralized communication networks that work without the internet.

## 🔍 What is MeshCore?

MeshCore now supports a range of LoRa devices, allowing for easy flashing without the need to compile firmware manually. Users can flash a pre-built binary using tools like Adafruit ESPTool and interact with the network through a serial console.
MeshCore provides the ability to create wireless mesh networks, similar to Meshtastic and Reticulum but with a focus on lightweight multi-hop packet routing for embedded projects. Unlike Meshtastic, which is tailored for casual LoRa communication, or Reticulum, which offers advanced networking, MeshCore balances simplicity with scalability, making it ideal for custom embedded solutions., where devices (nodes) can communicate over long distances by relaying messages through intermediate nodes. This is especially useful in off-grid, emergency, or tactical situations where traditional communication infrastructure is unavailable.

## ⚡ Key Features

* Multi-Hop Packet Routing – Devices can forward messages across multiple nodes, extending range beyond a single radio's reach. MeshCore supports up to a configurable number of hops to balance network efficiency and prevent excessive traffic.
* Supports LoRa Radios – Works with Heltec, RAK Wireless, and other LoRa-based hardware.
* Decentralized & Resilient – No central server or internet required; the network is self-healing.
* Low Power Consumption – Ideal for battery-powered or solar-powered devices.
* Simple to Deploy – Pre-built example applications make it easy to get started.

## 🎯 What Can You Use MeshCore For?

* Off-Grid Communication: Stay connected even in remote areas.
* Emergency Response & Disaster Recovery: Set up instant networks where infrastructure is down.
* Outdoor Activities: Hiking, camping, and adventure racing communication.
* Tactical & Security Applications: Military, law enforcement, and private security use cases.
* IoT & Sensor Networks: Collect data from remote sensors and relay it back to a central location.

## 🚀 How to Get Started

Flash the Firmware: Download the pre-built firmware binary for Heltec V3 and flash it using Adafruit ESPTool.
Install [PlatformIO](https://docs.platformio.org) in Visual Studio Code (optional for developers who want to modify the firmware).
Download & Open the MeshCore repository.
Select a Sample Application: Choose from chat, ping, repeater, or admin test tools.
Monitor & Communicate using the Serial Monitor (e.g., Serial USB Terminal on Android).

📁 Included Example Applications
* 📡 Terminal Chat: Secure text communication between devices.
* 📡 Simple Repeater: Extends network coverage by relaying messages.
* 📡 Companion Radio: For use with an external chat app, over BLE or USB.
* 📡 Room Server: A simple BBS server for shared Posts.
* 📡 Ping Client, Ping Server & Client: Basic client/server example.
* 📡 Test Admin: Monitors and manages repeaters remotely.

## 🛠 Hardware Compatibility

MeshCore is designed for use with:
* Heltec V3 LoRa Boards
* RAK4631
* XiaoS3 WIO (sx1262 combo)
* XiaoC3 (plus external sx126x module)
* LilyGo T3S3
* Heltec T114
* Station G2

## 📜 License
MeshCore is open-source software released under the MIT License. You are free to use, modify, and distribute it for personal and commercial projects.

## 📞 Get Support

Check out the GitHub Issues page to report bugs or request features.

You will be able to find additional guides and components at [my site](https://buymeacoffee.com/ripplebiz), or [join Andy Kirby's Discord](https://discord.gg/GBxVx2JMAy) for discussions.

## RAK Wireless Board Support in PlatformIO

Before building/flashing the RAK4631 targets in this project, there is, unfortunately, some patching you have to do to your platformIO packages to make it work. There is a guide here on the process:
   https://learn.rakwireless.com/hc/en-us/articles/26687276346775-How-To-Perform-Installation-of-Board-Support-Package-in-PlatformIO
   

