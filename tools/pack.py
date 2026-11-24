"""Build both ragdoll player downloads. --mod-only does not require built MML binaries."""
import argparse
import importlib.util
import json
import os
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'build/x86/out/mml/mods/ragdoll'
# Pin the baseline at packaging time. The installed mod negotiates services through the SDK.
TESTED_BASE = 'fc9549b830f17399511ffd0e7039b0008fd6bd3b929e01e824250b270426046d'
# MML 0.1's public packaging tool lists the same tested host at the game root.
TESTED_ROOT_LAYOUT = 'd70fff2e2392d9551e1a6e555d937155aec3668c83a262e9374c16e4167b56d5'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    choices = parser.add_mutually_exclusive_group()
    choices.add_argument('--mod-only', action='store_true')
    choices.add_argument('--with-loader', action='store_true', help='build only the clean-install download')
    choices.add_argument('--existing-asi', action='store_true', help='bundle MML as a plugin of an existing ASI loader')
    parser.add_argument('--loader-root', type=Path, default=os.environ.get('MML_ROOT', ROOT.parent / 'MafiaModLoader'))
    args = parser.parse_args()
    loader = args.loader_root.resolve()
    spec = importlib.util.spec_from_file_location('player_release', loader / 'tools/player_release.py')
    release = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(release)

    version = json.loads((OUT / 'mod.json').read_text(encoding='utf-8'))['version']
    name = f'mafia-ragdoll-{version}'
    dist = ROOT / 'build/dist' / f'{name}-player'
    prefix = 'mml/mods/ragdoll/'
    files = {prefix + n: OUT / n for n in ('ragdoll.dll', 'mod.json', 'config.json')}
    files[prefix + 'README.md'] = ROOT / 'README.md'
    files[prefix + 'licenses/Jolt.txt'] = loader / 'third_party/_deps/jolt/LICENSE'
    if not args.mod_only:
        root_layout = 'MafiaModLoader.asi' in release.base_files(loader)
        _, base = release.base_archive(loader, expected_content=TESTED_ROOT_LAYOUT if root_layout else TESTED_BASE)
        if root_layout:
            base['scripts/MafiaModLoader.asi'] = base.pop('MafiaModLoader.asi')
        archive = f'1-{name}-with-MML-{release.RELEASE}.zip'
        if args.existing_asi:
            del base['dinput8.dll']
            del base['licenses/UltimateASILoader.txt']
            archive = f'{name}-with-MML-{release.RELEASE}-existing-ASI.zip'
        bundle = {**base, **files, 'INSTALL-RAGDOLL.txt': ROOT / 'docs/INSTALL.txt'}
        print(release.write_zip(dist / archive, bundle))
    if not args.with_loader and not args.existing_asi:
        print(release.write_zip(dist / f'2-{name}-mod-only-if-MML-is-installed.zip', files))


if __name__ == '__main__':
    main()
