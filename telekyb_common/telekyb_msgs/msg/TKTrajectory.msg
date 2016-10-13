#
# Copyright 2011  Antonio Franchi and Martin Riedel    
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

Header header

# Input Pos
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
geometry_msgs/Vector3 snap

geometry_msgs/Vector3 rotangle
geometry_msgs/Vector3 angvelocity
geometry_msgs/Vector3 angvelocitydot

#TELEKYB_ENUM_VALUES(PosControlType, const char*,
#		(Position)("Position Control Mode")
#		(Velocity)("Velocity Control Mode")
#		(Acceleration)("Acceleration Control Mode")
#               (Geometric)("Geometric Tracking / Output Regulation Mode, position & attitude")
#)

uint8 POS_MODE=0
uint8 VEL_MODE=1
uint8 ACC_MODE=2

uint8 xAxisCtrlType
uint8 yAxisCtrlType
uint8 zAxisCtrlType

uint8 anglesCtrlType

float64 yawAngle
float64 yawRate
float64 yawAcceleration

#TELEKYB_ENUM_VALUES(YawControlType, const char*,
#		(AngleMode)("Angle Mode")
#		(RateMode)("Rate Mode")
#		(AccelerationMode)("Acceleration Mode")
#)

uint8 YAWANGLE_MODE=0
uint8 YAWRATE_MODE=1
uint8 YAWACC_MODE=2

uint8 yawCtrlType

