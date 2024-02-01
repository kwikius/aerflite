#!/bin/sh

AUTOTESTDIR=$(dirname $0)

nice fgfs \
    --native-fdm=socket,in,10,,5503,udp \
    --enable-mouse-pointer \
    --prop:/input/joysticks/js[0]=0 \
    --fdm=external \
    --aircraft=Rascal110-JSBSim \
    --fg-aircraft="$AUTOTESTDIR/aircraft" \
    --airport=YKRY \
    --geometry=650x550 \
    --bpp=32 \
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
    --glideslope=0 \
    --vc=0 \
    $*
