"""Concatena os .ino como o Arduino IDE faz (principal + demais em ordem
alfabetica) e gera prototipos, para compilar o sketch no PC com stubs."""
import glob, os, re, sys
Q = os.path.join(os.path.dirname(__file__), '..', '..', 'QuadController')
main = open(os.path.join(Q, 'QuadController.ino')).read()
others = sorted(f for f in glob.glob(os.path.join(Q, '*.ino')) if not f.endswith('QuadController.ino'))
bodies = [main] + [open(f).read() for f in others]
protos = []
for b in bodies:
    for m in re.finditer(r'^\s*(?:void|bool|double|float|int|long)\s+(\w+)\s*\(([^)]*)\)\s*\{?', b, re.M):
        if m.group(1) in ('setup', 'loop'):
            continue
        protos.append(m.group(0).strip().rstrip('{').strip() + ';')
cut = main.index('\n', main.index('#include "Safety.h"')) + 1
out = main[:cut] + '\n'.join(protos) + '\n' + main[cut:] + '\n' + '\n'.join(open(f).read() for f in others)
os.makedirs(sys.argv[1], exist_ok=True)
open(os.path.join(sys.argv[1], 'sketch.cpp'), 'w').write(out)
