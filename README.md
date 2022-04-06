# MONA_ESP_WeBot
WeBot simulation of the ESP MONA robotic platform. A big thank you to [Dorian Lévêque](https://github.com/dorianleveque) for developing the inital simulation as part of an internship!!! 

The `protos` folder contains the Mona prototype, for the 2021 version of WeBot. Unfortunately in the 2022 version coordinate system changed and the prototype needs chaging to take this into account.

Also `controllers/algo1` contains a `ino` file that can be reused as-is with the real Mona robots.

The arduino environment is mimicked thanks to the `Makefile` in the controller folder, the `Makefile.include` it the root of the project folder and the `libraries/arduino` folder.
