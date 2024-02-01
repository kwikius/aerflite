#!/bin/sh
/usr/games/fgfs \
    --fg-aircraft=/home/colin/apps/qgroundcontrol/release/files/flightgear/Aircraft \
    --fdm=jsb \
    --units-meters \
    --aircraft=Rascal110-JSBSim \
    --prop:/controls/gear/brake-left=0.25 --prop:/controls/gear/brake-right=0.25 \  #having a bit of brake on helps us stop when we land, but its not enough to prevent take off but make sure you pull up as hard as possible during takeoff
    --generic=socket,out,50,127.0.0.1,5501,udp,MAVLink --generic=socket,in,50,127.0.0.1,5500,udp,MAVLink \
    --roll=0 --pitch=0 --vc=0 --heading=0 --lat=52.5152 --lon=-4.0559 \
    --on-ground --wind=0@0 --turbulence=0.0 --timeofday=noon \
    --disable-hud-3d --disable-fullscreen --geometry=400x300 --disable-anti-alias-hud  \
    --prop:/sim/frame-rate-throttle-hz=30 --prop:/engines/engine/running=true \
    --disable-intro-music --disable-sound --disable-random-objects --disable-ai-models --shading-flat --fog-disable \
    --disable-specular-highlight --disable-panel --disable-clouds --prop:/sim/rendering/shader-effects=false \
    --prop:/sim/tower/latitude-deg=52.5152 --prop:/sim/tower/longitude-deg=-4.0559 --prop:/sim/tower/altitude-ft=7 #this should place the control tower at our launch site but doesn't seem to work properly
