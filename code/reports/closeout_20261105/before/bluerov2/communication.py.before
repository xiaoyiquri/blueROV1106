"""Load the existing communication/logger only, without DVL or planning imports."""
import importlib
from pathlib import Path
import sys


def load_communication_backend(directory):
    directory = Path(directory).resolve()
    source = directory / 'rov_control.py'
    if not source.is_file():
        raise ValueError(f'missing legacy communication backend: {source}')
    cached = sys.modules.get('rov_control')
    if cached is not None and Path(cached.__file__).resolve() != source:
        raise ValueError('rov_control was already loaded from a different directory')
    sys.path.insert(0, str(directory))
    try:
        return importlib.import_module('rov_control')
    finally:
        sys.path.pop(0)


def load_connection_config(path):
    """Read only transport fields; unrelated host-PD/DVL keys are not required."""
    import json
    raw = json.loads(Path(path).read_text())
    names = ('connection', 'baud', 'source_system', 'target_system', 'target_component',
             'connect_timeout', 'confirm_timeout', 'heartbeat_timeout')
    cfg = {name: raw[name] for name in names if name in raw}
    if not isinstance(cfg.get('connection'), str) or not cfg['connection'].strip():
        raise ValueError('connection must be a nonempty MAVLink endpoint')
    for name, default, lo, hi in [('baud',115200,1,10000000),('source_system',255,1,255),
                                   ('target_system',None,1,255),('target_component',None,1,255)]:
        value = cfg.get(name, default)
        if value is not None and (isinstance(value,bool) or not isinstance(value,int) or not lo <= value <= hi):
            raise ValueError('invalid connection field: '+name)
        cfg[name] = value
    return cfg
