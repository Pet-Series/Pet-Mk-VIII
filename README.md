
# Pet-Mk-VIII
<h1 align="center">Welcome to the Pet-Mk-VIII repository</h1>
<h1 align="center">The Robot Dashboard</h1>

ROS repository in the https://github.com/Pet-Series Git-Organizations.</br>
Containing multiply ROS-packages.

# Software/Setup Pet series micro robots #
The main objective/scope for this repository is to control the part of the software (and parameter settings) that is unique for this Pet Mark VIII (eight) robot. A.k.a "The Dashboard".

ROS2 (Robot Operating System v2) is used as middleware.
Ubuntu is used as operating system.

# The journey is the destination and goal
<table>
    <tr>Pet-Mk.VIII early itterations
      <td><img src="./doc/Pet-Mk.VIII_build_phase_04(design_iterations).png" width="400px"></td>
      <td><img src="./doc/Pet-Mk.VIII_build_phase_8(pet_joystick_prototype).png" width="400px"></td>
    </tr>
</table>
<table>
    <tr>The dashboard just starting to take shape.
      <td><img src="./doc/Pet-Mk.VIII_build_phase_14(aluminium_panel).png" width="450px"></td>
      <td><img src="./doc/Pet-Mk.VIII_build_phase_16(oak_side_panels).png" height="260px"></td>
    </tr>
</table>
<table>
    <tr>Electrical installation is beginning to take place
      <td><img src="./doc/Pet-Mk.VIII_build_phase_17(RPi-bracket).png" width="400px"></td>
      <td><img src="./doc/Pet-Mk.VIII_build_phase_20(ADS1115).png" width="400px"></td>
    </tr>
</table>

# Modules
## Module: Joystick
Using Analog/Digital Converter ADS1115 via I2C (3 of 4 chanels in use)
For more information see https://github.com/Pet-Series/pet_ros2_joystick_pkg
<table>
    <tr>Wiring diagram
      <td><img src="./doc/pet_ros2_joystick_wiring.png" height="400px"></td>
      <td></td>
    </tr>
</table>

## Module: Trim Potentiometers
Using Analog/Digital Converter ADS1115 via I2C (3 of 4 chanels in use)
<table>
    <tr>Wiring diagram
      <td><img src="./doc/pet_ros2_potentiometer_wiring.png" height="400px"></td>
      <td></td>
    </tr>
</table>

## Module: Buttons
Using direct GPIO-pins
<table>
    <tr>Wiring diagram
      <td><img src="./doc/pet_ros2_buttons_wiring.png" height="400px"></td>
      <td></td>
    </tr>
</table>

## Module: LED-strip
Using direct GPIO-pins
<table>
    <tr>Wiring diagram
      <td><img src="./doc/pet_ros2_LED0..LED4_wiring.png"height="400px"></td>
      <td></td>
    </tr>
</table>

## Module: Current & Voltage monitoring
Using INA219 sensor via I2C-bus.
<table>
    <tr>Wiring diagram
      <td><img src="./doc/pet_ros2_currentsensor(INA219)_wiring.png" height="400px"></td>
      <td></td>
    </tr>
</table>

# External references
- http://wiki.ros.org/

For my own convenience - Some GitHub cheat sheets
</br></br>
<a href="https://gitmoji.dev">
  <img src="https://img.shields.io/badge/gitmoji-%20😜%20😍-FFDD67.svg?style=flat-square" alt="Gitmoji">
</a></br>
- <a>https://guides.github.com/features/mastering-markdown/</a>
- https://help.github.com/en/github/writing-on-github/basic-writing-and-formatting-syntax
- https://www.webfx.com/tools/emoji-cheat-sheet/
