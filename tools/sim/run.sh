#!/bin/sh
# Simulacao no PC (sem hardware): seguranca/mixer, laco sincrono e filtros.
# Uso: sh tools/sim/run.sh
set -e
D=$(dirname "$0"); Q="$D/../../QuadController"
CXX="g++ -std=gnu++17"
SKETCH_FLAGS="-DARDUINO=100 -I$D/stubs -I$D/build/inc -I$D"
SKETCH_SRC="$Q/PID_v1.cpp $Q/GY80.cpp $Q/GyroFilter.cpp"

# Headers do sketch que nao dependem do hardware; os demais vem de stubs/.
mkdir -p "$D/build/inc"
cp "$Q/DCM.h" "$Q/PID_v1.h" "$Q/GY80.h" "$Q/Safety.h" "$Q/GyroFilter.h" "$D/build/inc/"
python3 "$D/gen_sketch.py" "$D/build"

$CXX $SKETCH_FLAGS "$D/test_safety.cpp" $SKETCH_SRC -o "$D/build/test_safety"
$CXX $SKETCH_FLAGS "$D/test_loop.cpp"   $SKETCH_SRC -o "$D/build/test_loop"
$CXX -I"$Q" "$D/test_filters.cpp" "$Q/GyroFilter.cpp" -o "$D/build/test_filters"

echo "== Seguranca e mixer =="; "$D/build/test_safety"
echo; echo "== Laco sincrono =="; "$D/build/test_loop"
echo; echo "== Filtros do giroscopio (ganho em cada frequencia) =="; "$D/build/test_filters"
