#!/usr/bin/env bash
set -euo pipefail

VERSION="2.4.0"
OUT="siminput-firmware-${VERSION}.zip"

# Generate manifest. Lists every file the OTA updater should push.
# config.json is deliberately excluded everywhere (manifest and ZIP) so an
# update can never overwrite the user's own config.
python3 -c "
import json, hashlib, os

def hash_file(path):
    data = open(path, 'rb').read()
    return {'path': path, 'sha256': hashlib.sha256(data).hexdigest(), 'size': len(data)}

files = []
for f in ['code.py', 'boot.py']:
    files.append(hash_file(f))
for root, dirs, fnames in sorted(os.walk('lib')):
    dirs.sort()
    for f in sorted(fnames):
        files.append(hash_file(os.path.join(root, f)))

manifest = {
    'firmware_version': '$VERSION',
    'files': files,
}
json.dump(manifest, open('manifest.json', 'w'), indent=2)
"

rm -f "$OUT"
zip -r "$OUT" manifest.json code.py boot.py lib/
rm -f manifest.json

echo "Built $OUT"
