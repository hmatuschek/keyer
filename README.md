# Electronic touch CW keyer
This tiny circuit and its firmware implements a simple electronic CW keyer using capacitive touch-paddles. Hence it has absolutely no moving parts which eases the build of the mechanical part of the keyer a lot. 

The central logic of the keyer is implemented in software using a ATTiny 45 MCU. It implements a simple (mode A) electronic CW key. The capacitive touch interface is easy too. It basically measures the capacity of the metal paddles which changes as they get touched with the hand. In fact, the hardware measures the time needed to charge the paddles though a large resistor. This time increases significantly if the paddle is touched. 

The circuit provides a open-collector output to switch even simple transmitters which expect a straight key. Additionally an LED and a side-tone generator are present for practicing. The speed of the keyer can be set using a potentiometer in a range from 10WPM to 42WPM.


## License
Both, the hard and software are licensed under the terms of the GNU Public License (GPL).

Copyright (C) 2015  Hannes Matuschek <hmatuschek at gmail dot com>

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with this program; if not, write to the Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.