# New API
The hardawre and general bringup described in PRD.md is fine, but some things need to be changed.

## Bringup
There are a lot of unnecesary logs in bringup. These require extra API methods and extra code in the API. Clean it up.

## README.md
README.md has a lot of unnecesary info for users. Like the hardware map, motor parameters, FOC tuning, etc. are useless for the user since they will always use this PCB and motor. Make sure it's only what's strictly needed in the API.

## Motor API
The motor API doesn't make sense anymore, since as you can see in phase 3 and 4 the actual profile computation has to occur in the FOC tight loop. Because of this, I want the API to be like the core0 just sends profiles over to core1 which executes them. 

All the weird motor commands go away. There is just `motorEnable` and `motorSetProfile`. Because of this you don't need to send over a position, velocity, accel number and then have code to pull the commanded position towards the target position - by computing the curve onbaord the loop it simplifies it a lot. The code in phase 3 and 4 should be vastly simplified for this.

(For phase 3 it's just a more relaxed profile with a really large distance value. As long as it will run for over 2 minutes at 1700rpm it's plenty long). Maybe phase 5 has to use some undocumented methods in the API.

Because the user can't calculate the profile directly, there should be lots of ways to construct the S curves. Here are some ways you should be able to construct the curve:
- Give distance, max velocity, and max acceleration
- Give distance, time, and max acceleration
- Give another profile and just scale the distance by some amount (like a vertical stretch)
- Give another profile and scale the time by some amount (like a horizontal stretch)

Each curve should also of course have a way of getting the time, the max velocity, the max accel, the time spent accelerating, decelerating, etc.

Add a print method that prints these in a nice easy to read manner in the serial terminal.

Everything will be in radians and seconds.

The user should be able to read the currently active profile (the actual profile object) and also read the progress of the profile (like current distance, time, velocity, acceleration, etc.). Make the current 0.8A always don't expose current to the user. 

## Messy code
In general there have been lots of changes bodged onto the original code in this repository leading to unclean, overly verbose code with lots of vestigial remnants, unused sections, unneeded features, and extra API methods. Simplify to the absolute max. The code in bringup phases 1-4 should be very very easy to read with the new API and super simple. (Phase 5 is of course complicated, that one is fine). Every file should getn inspected and refactored (and of course README has to be updated with the new motor API)
