#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --enable-mouse-pointer \
    --prop:/input/joysticks/js[0]=0 \
    --fdm=external \
    --aircraft=easystar \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --units-meters \
    --on-ground \
    --fg-scenery=/home/andy/resources/flightgear/scenery \
    --lat=51.573067 \
    --lon=-4.260000 \
    --wind=270@15 \
    --heading=270 \
    --disable-anti-alias-hud \
    --disable-hud-3d \
    --disable-horizon-effect \
    --timeofday=noon \
    --disable-sound \
    --disable-fullscreen \
    --disable-random-objects \
    --disable-ai-models \
    --fog-disable \
    --disable-specular-highlight \
    --wind=0@0 \
    $*
