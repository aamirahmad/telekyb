#
# Copyright 2011  Carlo Masone    
#
# This file is part of TeleKyb.
#
# TeleKyb is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# TeleKyb is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with TeleKyb. If not, see <http://www.gnu.org/licenses/>.

# Contains the set point for the low-lever controller of the gripper

Header header

# Points
geometry_msgs/Point[] controlPoints

# Velocity
geometry_msgs/Point[] controlPointsVelocity

# Knots
float64[] knots

# Indices of the portion of spline to draw
float64[] indices

# Order
uint8 order

# Color
geometry_msgs/Point[] color

# Number of lines to be drawn to approximate the spline
uint8 nLines

# Type
uint8 bsType
