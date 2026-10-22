"""pack.py: the players' archive of the mod.

    python tools/pack.py

From the build's output (build/x86/out/mml/mods/ragdoll/: ragdoll.dll, mod.json, config.json; no PDB, no linker
files) into build/dist/mafia-ragdoll-<version>-<date>.zip, laid out as the game's folder wants it
(mml/mods/ragdoll/...), with the README beside. The version is mod.json's. Build the Release preset first.
"""
import datetime
import json
import os
import sys
import zipfile

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, 'build', 'x86', 'out', 'mml', 'mods', 'ragdoll')
FILES = ('ragdoll.dll', 'mod.json', 'config.json')


def main():
    missing = [f for f in FILES if not os.path.exists(os.path.join(OUT, f))]
    if missing:
        print('not built: %s (cmake --build --preset x86-release)' % ', '.join(missing))
        return 1
    version = json.load(open(os.path.join(OUT, 'mod.json'), encoding='utf-8'))['version']
    dist = os.path.join(ROOT, 'build', 'dist')
    os.makedirs(dist, exist_ok=True)
    name = os.path.join(dist, 'mafia-ragdoll-%s-%s.zip' % (version, datetime.date.today().isoformat()))
    with zipfile.ZipFile(name, 'w', zipfile.ZIP_DEFLATED) as z:
        for f in FILES:
            z.write(os.path.join(OUT, f), 'mml/mods/ragdoll/' + f)
        z.write(os.path.join(ROOT, 'README.md'), 'mml/mods/ragdoll/README.md')
    print(name, '%.1f MB' % (os.path.getsize(name) / 1e6))
    return 0


if __name__ == '__main__':
    sys.exit(main())
