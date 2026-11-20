#!/bin/sh
# Simulacao no PC (sem hardware) dos estados de seguranca e do mixer.
# Uso: sh tools/sim/run.sh
set -e
D=$(dirname "$0"); Q="$D/../../QuadController"
# Headers do sketch que nao dependem do hardware; os demais vem de stubs/.
mkdir -p "$D/build/inc"
cp "$Q/DCM.h" "$Q/PID_v1.h" "$Q/GY80.h" "$Q/Safety.h" "$D/build/inc/"
python3 "$D/gen_sketch.py" "$D/build"
g++ -std=gnu++17 -DARDUINO=100 -I"$D/stubs" -I"$D/build/inc" -I"$D" \
    "$D/test_safety.cpp" "$Q/PID_v1.cpp" "$Q/GY80.cpp" -o "$D/build/test_safety"
"$D/build/test_safety"
