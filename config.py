import json
from copy import deepcopy
from pathlib import Path


CONFIG_PATH = Path(__file__).parent / "config.json"

DEFAULT_CONFIG = {
    "workspace_rotation": {
        "x_deg": 0.0,
        "y_deg": 0.0,
        "z_deg": 0.0,
    },
    "aruco": {
        "marker_size_m": 0.04,
        "dictionary": "DICT_4X4_50",
        "markers": {},
    },
    "homography": {},
    "robot": {
        "zero_pose": None,
    },
}


def _merge_defaults(config: dict, defaults: dict) -> dict:
    merged = deepcopy(defaults)
    for key, value in config.items():
        if isinstance(value, dict) and isinstance(merged.get(key), dict):
            merged[key] = _merge_defaults(value, merged[key])
        else:
            merged[key] = value
    return merged


def load_config() -> dict:
    if not CONFIG_PATH.exists():
        return deepcopy(DEFAULT_CONFIG)
    try:
        return _merge_defaults(json.loads(CONFIG_PATH.read_text()), DEFAULT_CONFIG)
    except Exception:
        return deepcopy(DEFAULT_CONFIG)


def save_config(config: dict) -> None:
    CONFIG_PATH.write_text(json.dumps(config, indent=2))


def get_section(name: str) -> dict:
    return deepcopy(load_config().get(name, {}))


def update_section(name: str, data: dict) -> None:
    config = load_config()
    config[name] = deepcopy(data)
    save_config(config)
