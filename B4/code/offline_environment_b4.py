"""Synthetic B4 protocol business-rule double. Only this transport holds truth.

This is not an HTTP server: payloads are already decoded dictionaries. Raw
headers, duplicate JSON keys, and concurrent socket policing are out of scope.
"""
import copy
import hashlib
import json
import math
import random
import unicodedata

PATTERNS = ('random_mixed', 'all_directional', 'mostly_dir', 'mostly_omni',
            'boundary', 'backface_origin', 'coverage_edge', 'cluster', 'origin',
            'adversarial_fan')
ERROR_MODES = ('fixed_field', 'endpoints', 'zero')


def wrap180(angle):
    return (angle + 180.) % 360. - 180.


def emission_contains(source, position):
    if source['kind'] == 'omni':
        return True
    dx, dy = position[0]-source['x'], position[1]-source['y']
    if dx == 0 and dy == 0:
        return True  # The vertex belongs to every closed halfplane.
    angle = math.degrees(math.atan2(dy, dx))
    return abs(wrap180(angle-source['direction_deg'])) <= math.nextafter(90., math.inf)


def make_case(seed, count=None, pattern='random_mixed'):
    if pattern not in PATTERNS:
        raise ValueError('unknown B4 pattern')
    count = 10+seed % 7 if count is None else count
    if not isinstance(count, int) or isinstance(count, bool) or not 10 <= count <= 16:
        raise ValueError('generated case count must be 10..16')
    rng = random.Random(seed)
    sources = []
    for i, channel in enumerate(rng.sample(range(1, 21), count)):
        a = rng.uniform(0, 2*math.pi)
        distance = 1800*math.sqrt(rng.random())
        radius = rng.uniform(1000, 1500)
        if pattern == 'boundary':
            distance, radius = 1800, 1000
        elif pattern == 'cluster':
            distance, a = rng.uniform(900, 1000), rng.uniform(.10, .14)
        elif pattern == 'origin' and i == 0:
            distance = 0.
        probability = .75 if pattern == 'mostly_dir' else .25 if pattern == 'mostly_omni' else .5
        kind = 'directional' if rng.random() < probability else 'omni'
        direction = rng.uniform(0, 360)
        if pattern in ('all_directional', 'backface_origin', 'coverage_edge', 'adversarial_fan'):
            kind = 'directional'
        toward_origin = (math.degrees(a)+180) % 360
        if pattern == 'backface_origin':
            direction = math.degrees(a) % 360
        elif pattern == 'coverage_edge':
            direction = (toward_origin+90+(-1e-8, 0., 1e-8)[i % 3]) % 360
        elif pattern == 'adversarial_fan':
            direction = (toward_origin+90-1e-8) % 360
        sources.append({'channel': channel, 'x': distance*math.cos(a),
                        'y': distance*math.sin(a), 'radius': radius, 'kind': kind,
                        'direction_deg': direction if kind == 'directional' else None})
    return {'kind': 'synthetic_offline_B4', 'seed': seed, 'pattern': pattern, 'sources': sources}


def _finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _identifier(value, limit):
    return (isinstance(value, str) and 1 <= len(value.encode('utf-8')) <= limit
            and not any(unicodedata.category(c) in ('Cc', 'Cf') for c in value))


class OfflineTransportB4:
    def __init__(self, case, robot_id='offline-test', error_mode='fixed_field'):
        if error_mode not in ERROR_MODES:
            raise ValueError('unknown error mode')
        if not _identifier(robot_id, 64):
            raise ValueError('invalid robot id')
        self._sources = {}
        for source in case['sources']:
            s = copy.deepcopy(source)
            if (not isinstance(s.get('channel'), int) or isinstance(s['channel'], bool)
                or not 1 <= s['channel'] <= 20 or s['channel'] in self._sources
                or not all(_finite(s.get(k)) for k in ('x', 'y', 'radius'))
                or math.hypot(s['x'], s['y']) > 1800+1e-8
                or not 1000 <= s['radius'] <= 1500
                or s.get('kind') not in ('omni', 'directional')):
                raise ValueError('invalid synthetic source')
            if s['kind'] == 'directional' and (not _finite(s.get('direction_deg')) or not 0 <= s['direction_deg'] < 360):
                raise ValueError('invalid emission direction')
            if s['kind'] == 'omni' and s.get('direction_deg') is not None:
                raise ValueError('omni source must have no direction')
            self._sources[s['channel']] = s
        self.seed = case.get('seed', 0)
        self.robot_id, self.error_mode = robot_id, error_mode
        self.cleared = set()
        self.position, self.channel = (0., 0.), 1
        self.active, self.ended = False, False
        self.virtual_us = 0
        self.cache = {}

    @property
    def vt(self):
        return self.virtual_us/1_000_000

    def _body(self, accepted):
        return {'accepted': accepted, 'real_timestamp_ms': 0,
                'virtual_time_s': self.vt if accepted else 0}

    def _validate(self, path, payload):
        if path not in ('/enter', '/measure', '/clear', '/exit'):
            return 404
        if not isinstance(payload, dict):
            return 400
        required = {'arena_id', 'robot_id', 'request_id'}
        if path in ('/measure', '/clear'):
            required |= {'position', 'channel'}
        if not required <= payload.keys():
            return 400
        if not _identifier(payload['robot_id'], 64) or not _identifier(payload['request_id'], 128):
            return 400
        if not isinstance(payload['arena_id'], str):
            return 400
        if path in ('/measure', '/clear'):
            p, ch = payload['position'], payload['channel']
            if (not isinstance(p, dict) or not {'x', 'y'} <= p.keys()
                or not all(_finite(p[k]) and abs(p[k]) <= 2_000_000 for k in ('x', 'y'))
                or not _finite(ch) or not 1 <= ch <= 20 or int(ch) != ch):
                return 400
            if set(p) != {'x', 'y'}:
                return 200
        if set(payload) != required or payload['arena_id'] != 'default' or payload['robot_id'] != self.robot_id:
            return 200
        return None

    def __call__(self, path, payload):
        status = self._validate(path, payload)
        if status is not None:
            return status, self._body(False)
        identity = json.dumps([path, payload], sort_keys=True, separators=(',', ':'), allow_nan=False)
        key = payload['request_id']
        if key in self.cache:
            old, response = self.cache[key]
            return copy.deepcopy(response) if old == identity else (409, self._body(False))
        if self.ended or (path == '/enter' and self.active) or (path != '/enter' and not self.active):
            return 200, self._body(False)
        if path == '/enter':
            self.active = True
            body = self._body(True)
            body.update(max_virtual_duration_s=360000, max_real_duration_s=1200, remaining_real_duration_s=1200)
        elif path == '/exit':
            self.active, self.ended = False, True
            body = self._body(True); body['exit_reason'] = 'user_exit'
        else:
            p = payload['position']; position = (p['x'], p['y']); channel = int(payload['channel'])
            self.virtual_us += round(math.dist(self.position, position)/5*1_000_000)
            self.position = position
            source = self._sources.get(channel)
            exists = source is not None and channel not in self.cleared
            distance = math.dist(position, (source['x'], source['y'])) if exists else math.inf
            if path == '/clear':
                success = exists and distance <= 20
                self.virtual_us += (5 if success else 3)*1_000_000
                if success:
                    self.cleared.add(channel)
                body = self._body(True)
                body['clear_result'] = 'success' if success else 'no_target_in_range'
            else:
                self.virtual_us += (5+int(channel != self.channel))*1_000_000
                self.channel = channel
                body = self._body(True)
                if not exists or distance > source['radius'] or not emission_contains(source, position):
                    body['measure_result'] = 'no_signal'
                elif distance <= 5:
                    body['measure_result'] = 'near'
                else:
                    true = math.degrees(math.atan2(source['y']-position[1], source['x']-position[0]))
                    lo, hi = math.ceil((true-1)*100), math.floor((true+1)*100)
                    field = f'{self.seed}:{channel}:{position[0]:.9f}:{position[1]:.9f}'
                    h = int(hashlib.sha256(field.encode()).hexdigest()[:16], 16)
                    value = (lo if h % 2 else hi) if self.error_mode == 'endpoints' else round(true*100) if self.error_mode == 'zero' else lo+h % (hi-lo+1)
                    body.update(measure_result='direction', svd_deg=(value/100) % 360)
        response = (200, body)
        self.cache[key] = (identity, copy.deepcopy(response))
        return response
