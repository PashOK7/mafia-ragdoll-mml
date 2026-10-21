"""The mod's settings page (mod/mod.json "settings", mod/config.json's defaults) laid out by the loader's own code in a
running lab game (the bridge's panel.layout) and judged by the loader's offline check (tests/run_tests.py's
ui_layout_problems): no control or text over another, nothing off its window or the screen, no text wider than its
button, no label cut. Prints the problems; exits 1 on any.

    python tools/check_settings.py [--game 1.2]
"""
import argparse
import importlib.util
import json
import os
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
MML = os.environ.get('MML_ROOT', os.path.join(os.path.dirname(ROOT), 'MafiaModLoader'))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--game', default='1.2')
    a = ap.parse_args()
    spec = importlib.util.spec_from_file_location('mml_run_tests', os.path.join(MML, 'tests', 'run_tests.py'))
    tests = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(tests)
    manifest = json.load(open(os.path.join(ROOT, 'mod', 'mod.json'), encoding='utf-8'))
    defaults = json.load(open(os.path.join(ROOT, 'mod', 'config.json'), encoding='utf-8'))
    args = {'game': a.game, 'method': 'panel.layout',
            'params': {'settings': manifest['settings'], 'defaults': defaults, 'name': manifest['id']}}
    r = subprocess.run([sys.executable, os.path.join(MML, 'tools', 'mcp_call.py'), 'call', json.dumps(args)],
                       capture_output=True, text=True, encoding='utf-8')
    try:
        layout = json.loads(r.stdout)
    except ValueError:
        print('no layout:', r.stdout[:500], r.stderr[:500])
        return 1
    problems = {}
    for page in layout['mods'][0]['pages']:
        p = tests.ui_layout_problems(page)
        if p:
            problems[page.get('part', '?')] = p
        print('page %s: %d controls' % (page.get('part', '?'), len(page['items']) - 1))
    p = tests.ui_layout_problems(layout['list'])
    if p:
        problems['the Mods list'] = p
    for k, v in problems.items():
        print('%s:' % k)
        for x in v:
            print('  ' + x)
    print('ALL PASSED' if not problems else 'FAILED')
    return 1 if problems else 0


if __name__ == '__main__':
    sys.exit(main())
