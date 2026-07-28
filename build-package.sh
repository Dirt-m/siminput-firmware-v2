#!/usr/bin/env bash
set -euo pipefail

VERSION="2.6.0"
OUT="siminput-firmware-${VERSION}.zip"

# The version the firmware reports must match the version this package
# declares, or the desktop app's post-update check reports a mismatch.
FW_VERSION=$(sed -n 's/^FW_VERSION = "\(.*\)"$/\1/p' lib/serial_handler.py)
if [ "$FW_VERSION" != "$VERSION" ]; then
  echo "ERROR: build-package.sh VERSION=$VERSION but serial_handler.py FW_VERSION=$FW_VERSION" >&2
  exit 1
fi

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
# Note: dirs must be pruned in-place on the live walk iterator — wrapping
# os.walk in sorted() materializes the walk first and disables the pruning.
for root, dirs, fnames in os.walk('lib'):
    dirs[:] = sorted(d for d in dirs if d != '__pycache__')
    for f in sorted(fnames):
        if f.endswith(('.pyc', '.pyo')):
            continue
        files.append(hash_file(os.path.join(root, f)))

manifest = {
    'firmware_version': '$VERSION',
    # Forward-compat gates, added while the installed base is small:
    # protocol lets clients know what this package speaks; min_updater_version
    # lets a package refuse configurators too old to install it safely.
    'protocol': 2,
    'min_updater_version': '1.1.0',
    'files': files,
}
json.dump(manifest, open('manifest.json', 'w'), indent=2)
"

rm -f "$OUT"
zip -r "$OUT" manifest.json code.py boot.py lib/ -x "lib/__pycache__/*" "*.pyc" "*.pyo"
rm -f manifest.json

echo "Built $OUT"
