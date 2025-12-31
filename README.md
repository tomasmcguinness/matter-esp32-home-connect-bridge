# Matter Home Connect Bridge for ESP32

Built on the ESP32-S3 Elecrow 4.2" ePaper display, this project will act as a Matter Bridge, exposing your Home Connect Dishwasher to your Matter network.

This project is in its infancy and many parts are missing.

<img width="500" alt="image" src="https://github.com/user-attachments/assets/330fc1f3-bdf4-4e56-ae0d-cdb371db7c73" />

## Setup

There are two steps involved in setting up the device. After flashing the firmware and powering it up..

### Scan the Matter QR code 
This will allow you to commission the device into your Matter network. It should appear as a bridge.

### Scan the Home Connect QR code.
This will then allow the device to access your Home Connect devices. This happens using OAuth and your username and password is never exposed.

### The available programs should appear.
Once connected to Home Connect, the device will pull down and store the list of available programs.

## Matter Support
So far, the only Matter controller I've seen that can handle the dishwasher is the one in Home Assistant

<img width="500" alt="image" src="https://github.com/user-attachments/assets/a9776395-61f3-4760-b9d4-5cf029fbc37b" />

iOS Home has no clue about dishwashers and the Aqara app will only expose the Start/Stop and current state. 

> [!WARNING]
> This code assumes you have *one* device and that it's a dishwasher. 

> [!NOTE]
> I know the QR codes are small, but they are perfectly formed :)

## Building and Flashing

Built against the `esp-matter` `main` branch.

At present, you will need to compile and flash the firmware manually using IDF. I hope to provide a precompiled binary.

## Tasks

[ ] Correctly handle the available devices
[ ] Display current status of the device (started, stopped, door open etc)
[ ] Real time events
[ ] Fully local connection
[ ] Refactor the code to clean it up.

## Dev Notes

This was an interesting technical challenge as it involved figuring out Matter Bridging, the Home Connect API and the Elecrow display. Here are some blog posts I wrote.

https://tomasmcguinness.com/2025/12/24/creating-a-matter-bridge-for-my-neff-dishwasher/
https://tomasmcguinness.com/2025/12/30/creating-a-matter-bridge-for-my-neff-dishwasher-pt2/

## The display

If you want to purchase the Elecrow display, you can find it here. This is an affiliate link and I'll earn a little commissioning if you purchase with this.

https://www.awin1.com/cread.php?awinmid=82721&awinaffid=2699766&ued=https%3A%2F%2Fwww.elecrow.com%2Fcrowpanel-esp32-4-2-e-paper-hmi-display-with-400-300-resolution-black-white-color-driven-by-spi-interface.html
