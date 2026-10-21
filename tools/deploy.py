"""deploy.py: the mod's build into the labs, and reloaded in a running game.

    python tools/deploy.py              # every lab of MafiaModLoader's mml.config.json
    python tools/deploy.py --game 1.0   # one lab

The build's output (build/x86/out/mml/mods/ragdoll/: ragdoll.dll, its PDB, mod.json, config.json) is copied into
<lab>/mml/mods/ragdoll/; the lab's own mod.json "enabled" and config.json are kept once they exist (settings changed
in the game stay). The loader loads a mod from a copy of its own (mml\\live), so the lab's DLL is never locked; a
running game of that lab then reloads it (MafiaModLoader's MCP server through its tools/mcp_call.py).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MML = os.environ.get('MML_ROOT') or os.path.join(os.path.dirname(ROOT), 'MafiaModLoader')
OUT = os.path.join(ROOT, 'build', 'x86', 'out', 'mml', 'mods', 'ragdoll')


def labs():
    cfg = json.load(open(os.path.join(MML, 'mml.config.json'), encoding='utf-8'))
    return cfg.get('labs') or {'default': cfg['game_dir']}


def enabled_of(text):
    m = re.search(r'"enabled"\s*:\s*(true|false)', text)
    return m.group(1) if m else None


def deploy(build, folder):
    dest = os.path.join(folder, 'mml', 'mods', 'ragdoll')
    os.makedirs(dest, exist_ok=True)
    copied = []
    for name in os.listdir(OUT):
        src, dst = os.path.join(OUT, name), os.path.join(dest, name)
        if os.path.isdir(src) or name.lower().endswith(('.ilk', '.exp', '.lib')):
            continue
        if name == 'config.json' and os.path.exists(dst):
            continue   # (the lab's settings stay)
        data = open(src, 'rb').read()
        if name == 'mod.json' and os.path.exists(dst):
            keep = enabled_of(open(dst, encoding='utf-8').read())
            if keep:
                data = re.sub(rb'"enabled"\s*:\s*(true|false)', ('"enabled": %s' % keep).encode(), data, count=1)
        if os.path.exists(dst) and open(dst, 'rb').read() == data:
            continue
        tmp = dst + '.new'
        open(tmp, 'wb').write(data)
        if os.path.exists(dst):
            os.remove(dst)   # (Windows refuses a rename over a file another process opened)
        os.rename(tmp, dst)
        copied.append(name)
    r = subprocess.run([sys.executable, os.path.join(MML, 'tools', 'mcp_call.py'), 'mod_reload', json.dumps({'id': 'ragdoll', 'game': build})],
                       capture_output=True, text=True, errors='replace')
    reloaded = 'not running' if 'not running' in (r.stdout + r.stderr) else ('reloaded' if r.returncode == 0 else (r.stdout + r.stderr).strip()[-300:])
    print('%s (%s): %s; %s' % (build, folder, ', '.join(copied) or 'nothing changed', reloaded))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--game')
    a = ap.parse_args()
    if not os.path.isdir(OUT):
        sys.exit('no build: cmake --build --preset x86-release')
    for build, folder in labs().items():
        if a.game and a.game != build:
            continue
        deploy(build, folder)


if __name__ == '__main__':
    main()
