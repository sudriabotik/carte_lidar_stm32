#!/bin/bash

openocd -f interface/stlink.cfg -f target/stm32g4x.cfg -c "program build/Release/carte_lidar.elf verify reset exit"
