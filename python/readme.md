# Modified madmom Files 

I added a callback function so that when the beat detector finds a beat, it calls my routine to process it. I did some processing to handle missing beats and keep to a cadence, labeled "a, b, c, d" in the data.

The WebSocket interface is used to support a web application. Commands are relayed to the ESP32s via the serial interface, which sends them wirelessly via ESP-NOW.
