# Copyright (c) 2016
# Author: Victor Arribas <v.arribas.urjc@gmail.com>
# License: GPLv3 <http://www.gnu.org/licenses/gpl-3.0.html>

import math

__author__ = 'varribas'


# drone dimensions
real_width  = 0.5  # m (drone width)
real_height = 0.15  # m

# drone kinematics
max_vel_XY = 2.5  # m/s
max_vel_Z  = 3.0  # m/s
max_rot_Zn = 0.5  # rad/s

# latency: ms=80 (=12.5fps)
secs = 0.08
fps = 1/secs


# death zone length
safety_distance = 0.25


# collision (corners/diagonal):
static_collision_XY = math.sqrt((real_width/2)**2 *2) *2

# collision (latency)
dynamic_collision_XY = max_vel_XY * secs + static_collision_XY

print
print 'static_collision:', static_collision_XY
print 'dynamic_collision_XY:', dynamic_collision_XY


static_collision_Z = real_height/2 * 2

up_reaction = max_vel_Z * secs
rot_reaction = math.tan(max_rot_Zn * secs) * real_width/2
dynamic_collision_Z = up_reaction + rot_reaction + static_collision_Z

print
print 'static_collision_Z:', static_collision_Z
print 'dynamic_collision_Z:', dynamic_collision_Z

safety_static_distance_XZ = static_collision_XY + safety_distance
safety_static_distance_Z = static_collision_Z + safety_distance

safety_dynamic_distance_XZ = dynamic_collision_XY + safety_distance
safety_dynamic_distance_Z = dynamic_collision_Z + safety_distance

print
print
print 'safety_static_distance_XZ:', safety_static_distance_XZ
print 'safety_dynamic_distance_XZ:', safety_dynamic_distance_XZ
print
print 'safety_static_distance_Z:', safety_static_distance_Z
print 'safety_dynamic_distance_Z:', safety_dynamic_distance_Z