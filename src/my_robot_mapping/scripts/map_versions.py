"""Transactional, immutable map versions. No ROS or robot side effects."""
import hashlib
import json
import math
import os
from pathlib import Path
import uuid
import time
import yaml


def write_grid(prefix, *, width, height, resolution, origin, data):
    """Write the received occupancy snapshot without spawning a DDS map saver.

    PGM rows are reversed relative to OccupancyGrid. Use Nav2 trinary thresholds.
    Files live in an unpublished version directory until commit succeeds.
    """
    if width <= 0 or height <= 0 or len(data) != width * height:
        raise ValueError('invalid occupancy grid dimensions')
    if not math.isfinite(resolution) or resolution <= 0:
        raise ValueError('invalid resolution')
    origin = validate_pose(origin)
    if any(v < -1 or v > 100 for v in data):
        raise ValueError('invalid occupancy value')
    prefix = Path(prefix)
    pixels = bytearray()
    for row in range(height - 1, -1, -1):
        pixels.extend(205 if v < 0 or 25 < v < 65 else (254 if v <= 25 else 0)
                      for v in data[row*width:(row+1)*width])
    with prefix.with_suffix('.pgm').open('xb') as f:
        f.write(('P5\n%d %d\n255\n' % (width, height)).encode() + pixels)
        f.flush()
        os.fsync(f.fileno())
    with prefix.with_suffix('.yaml').open('x') as f:
        yaml.safe_dump(dict(image=prefix.name+'.pgm', mode='trinary',
                            resolution=resolution, origin=origin, negate=0,
                            occupied_thresh=.65, free_thresh=.25), f)
        f.flush()
        os.fsync(f.fileno())


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    with tmp.open('x', encoding='utf8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2, allow_nan=False)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    if os.name == 'posix':
        fd = os.open(str(path.parent), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)


def validate_pose(pose):
    if len(pose) != 3 or not all(math.isfinite(float(v)) for v in pose):
        raise ValueError('initial pose must contain finite x, y, yaw')
    return [float(v) for v in pose]


class MapVersions:
    def __init__(self, root):
        self.root = Path(root).expanduser().resolve()
        (self.root / 'versions').mkdir(parents=True, exist_ok=True)

    def begin(self):
        path = self.root / 'versions' / uuid.uuid4().hex
        path.mkdir()
        return path / 'map'

    def validate(self, prefix):
        prefix = Path(prefix).resolve()
        if prefix.parent.parent != self.root / 'versions':
            raise ValueError('map prefix outside version store')
        config = yaml.safe_load(prefix.with_suffix('.yaml').read_text())
        resolution = float(config['resolution'])
        if not math.isfinite(resolution) or resolution <= 0:
            raise ValueError('invalid map resolution')
        validate_pose(config['origin'])
        image = (prefix.parent / config['image']).resolve()
        if image.parent != prefix.parent:
            raise ValueError('image must be in the same immutable version')
        files = [prefix.with_suffix(ext) for ext in ('.yaml', '.posegraph', '.data')]
        files.append(image)
        hashes = {}
        for p in files:
            if not p.is_file() or p.stat().st_size == 0:
                raise ValueError('missing or empty map artifact: ' + str(p))
            with p.open('rb') as artifact:
                hashes[p.name] = hashlib.sha256(artifact.read()).hexdigest()
                os.fsync(artifact.fileno())
        return hashes

    def commit(self, prefix, *, complete, session, pose, reason):
        prefix = Path(prefix).resolve()
        record = dict(schema=1, map_id=prefix.parent.name, prefix=str(prefix),
                      complete=bool(complete), session_id=session, saved_unix=time.time(),
                      pose_at_save=validate_pose(pose), reason=reason,
                      hashes=self.validate(prefix))
        atomic_json(prefix.parent / 'manifest.json', record)
        # Partial snapshots can never replace a validated complete map.
        if complete:
            atomic_json(self.root / 'current.json', record)
        else:
            atomic_json(self.root / 'checkpoint.json', record)
        return record

    def current(self):
        path = self.root / 'current.json'
        if not path.exists():
            return None
        record = json.loads(path.read_text())
        if record.get('complete') is not True:
            raise ValueError('current map is not complete')
        if self.validate(record['prefix']) != record['hashes']:
            raise ValueError('map artifact checksum mismatch')
        return record
