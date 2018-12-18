-- for documentation, see the top of soldier_motor_control.lua
local U = dofile("Files/Scripts/motor_control_util.lua")

-- put all the joints into named variables
local neck = U.findJoint( "carapace", "head" )
local tail = U.findJoint( "carapace", "tail" )

local lja1, rja1 = U.findSymmetricJoints( "carapace", "leg a 1" )
local lja2, rja2 = U.findSymmetricJoints( "leg a 1",  "leg a 2" )
local lja3, rja3 = U.findSymmetricJoints( "leg a 2",  "leg a 3" )

local ljb1, rjb1 = U.findSymmetricJoints( "carapace", "leg b 1" )
local ljb2, rjb2 = U.findSymmetricJoints( "leg b 1",  "leg b 2" )
local ljb3, rjb3 = U.findSymmetricJoints( "leg b 2",  "leg b 3" )

local ljc1, rjc1 = U.findSymmetricJoints( "carapace", "leg c 1" )
local ljc2, rjc2 = U.findSymmetricJoints( "leg c 1",  "leg c 2" )
local ljc3, rjc3 = U.findSymmetricJoints( "leg c 2",  "leg c 3" )


local inv_timestep = 60.0				-- in the c++ code inv_timestep is actually derived from timestep, but w/e
local timestep = 1.0 / inv_timestep


-- [flail wildly]
lja1.setOrientedTorque(ba.createVector(500, 0, 0))
rja1.setOrientedTorque(ba.createVector(500, 0, 0))
ljb1.setOrientedTorque(ba.createVector(500, 0, 0))
rjb1.setOrientedTorque(ba.createVector(500, 0, 0))
ljc1.setOrientedTorque(ba.createVector(500, 0, 0))
rjc1.setOrientedTorque(ba.createVector(500, 0, 0))
