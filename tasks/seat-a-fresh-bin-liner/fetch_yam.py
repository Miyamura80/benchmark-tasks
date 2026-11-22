"""Download and verify the pinned MuJoCo Menagerie assets for this task."""
import hashlib
import json
from pathlib import Path
from urllib.request import urlopen


ROOT = Path(__file__).resolve().parent
YAM = ROOT / "vendor" / "i2rt_yam"
REVISION = "370c43965323c0944b433b202aac5058b5f907a9"
BASE_URL = f"https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/{REVISION}/i2rt_yam/"


def main():
    files = json.loads((ROOT / "yam-assets.json").read_text())
    for name, digest in files.items():
        path = YAM / name
        if path.is_file() and hashlib.sha256(path.read_bytes()).hexdigest() == digest:
            continue
        with urlopen(BASE_URL + name, timeout=45) as response:
            payload = response.read()
        if hashlib.sha256(payload).hexdigest() != digest:
            raise RuntimeError(f"Published model asset hash differs for {name}")
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(payload)
    print(f"Verified {len(files)} stock YAM files.")


if __name__ == "__main__":
    main()
