"""Sample the character skin without the detached visual, at every stock LOD."""
import io
import struct
from collections import Counter
from pathlib import Path


def skin_lods(data):
    stream = io.BytesIO(data)

    def read(fmt):
        fmt = '<' + fmt
        return struct.unpack(fmt, stream.read(struct.calcsize(fmt)))

    def string():
        return stream.read(read('B')[0])

    assert stream.read(4) == b'4DS\0' and read('H') == (29,)
    stream.read(8)
    for _ in range(read('H')[0]):
        flags, = read('I')
        stream.read(40)
        if flags & (1 << 19):
            stream.read(4)
            string()
        diffuse = flags & (1 << 18)
        opacity = flags & (1 << 30) and not flags & (1 << 24)
        if diffuse:
            string()
        if opacity:
            string()
        if not diffuse and not opacity:
            stream.read(1)
        if flags & (3 << 25):
            stream.read(18)
    for _ in range(read('H')[0]):
        kind, = read('B')
        if kind == 1:
            visual, = read('B')
            stream.read(2)
        stream.read(43)
        string()
        string()
        if kind == 6:
            stream.read(24)
        elif kind == 7:
            stream.read(2)
            stream.read(2 * read('B')[0])
        elif kind == 10:
            stream.read(68)
        else:
            assert kind == 1 and visual in (2, 3) and read('H') == (0,)
            for _ in range(read('B')[0]):
                stream.read(4)
                vertices = [read('8f')[:3] for _ in range(read('H')[0])]
                faces = []
                for _ in range(read('B')[0]):
                    faces.extend(read('3H') for _ in range(read('H')[0]))
                    stream.read(2)
                yield vertices, faces
            return
    raise AssertionError('no character skin in model')


def crown_height(vertices, faces, x, z):
    top = float('-inf')
    for i, j, k in faces:
        a, b, c = vertices[i], vertices[j], vertices[k]
        ux, uy, uz = (b[d] - a[d] for d in range(3))
        vx, vy, vz = (c[d] - a[d] for d in range(3))
        det = ux * vz - uz * vx
        if det >= -1e-9:  # downward or edge-on, invisible from above
            continue
        s = ((x-a[0])*vz - (z-a[2])*vx) / det
        t = (ux*(z-a[2]) - uz*(x-a[0])) / det
        if min(s, t, 1-s-t) >= -1e-6:
            top = max(top, a[1] + s*uy + t*vy)
    return top


def check_hands(game, path, prepared):
    # These stock models also use the headwear material on their distant hands.
    name = 'models\\' + path.name
    data = bytearray()
    while True:
        part = game.call('files.read', name=name, game=True, offset=len(data), size=8192)
        chunk = bytes.fromhex(part['hex'])
        assert chunk and part['source'] == 'game'
        data.extend(chunk)
        if len(data) == part['size']:
            break

    def hands(level):
        vertices, faces = level
        return Counter(tuple(vertices[i] for i in face) for face in faces
                       if all(abs(vertices[i][0]) > .6 for i in face))

    stock = list(skin_lods(data))
    assert len(stock) == len(prepared) > 1
    for source, output in zip(stock[1:], prepared[1:]):
        expected = hands(source)
        assert expected and hands(output) == expected, path.name


def check(game):
    files = game.call('files.list', mod='ragdoll')['files']
    wearers = [Path(f['path']) for f in files if f['mod'] == 'ragdoll' and
               f['name'].startswith('models\\') and not f['name'].startswith('models\\rdhat_')]
    assert wearers
    levels = 0
    for path in wearers:
        prepared = list(skin_lods(path.read_bytes()))
        if path.stem.lower() in ('bookmaker01', 'bookmaker02', 'enemy10+', 'enemy10k',
                                 'freegang02', 'friend1', 'homeless01', 'salman05k'):
            check_hands(game, path, prepared)
        for level, (vertices, faces) in enumerate(prepared):
            # Stock human bind space, five rays inside the opening above the brow.
            for x, z in [(0,.01), (.025,.01), (-.025,.01), (0,.04), (0,-.02)]:
                height = crown_height(vertices, faces, x, z)
                assert height > .69, (path.name, level, x, z, height)
            levels += 1
    print(game.build, 'closed crowns:', len(wearers), 'wearers,', levels, 'LODs')
