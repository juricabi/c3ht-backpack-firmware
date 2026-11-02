Import("env")
# C3HT EU build post-script (only in C3HTEU_* envs, after build_env_setup.py).
#  - _via_WIFI envs upload the SIGNED image to http://10.0.0.1 with login
#    (python/c3ht_eu_upload.py) instead of the unauthenticated stock uploader;
#  - writes firmware.eu.json next to firmware.bin (what the signer needs).
# Signing itself is an offline step (release key never on the build machine):
#   python -I <C3HT>/firmware/eu-build/tools/sign_fw.py sign --key <priv.pem> firmware.bin
import hashlib
import json
import os
import re

import c3ht_eu_upload

target_name = env["PIOENV"].upper()
if "_WIFI" in target_name:
    env.Replace(UPLOAD_PROTOCOL="custom")
    env.Replace(UPLOADCMD=c3ht_eu_upload.on_upload)
    if env.get("UPLOAD_PORT") in (None, "", "elrs_vrx.local"):
        env.Replace(UPLOAD_PORT="10.0.0.1")


def write_manifest(source, target, env):
    path = str(target[0])
    with open(path, "rb") as f:
        data = f.read()
    sv = None
    for fl in env.get("BUILD_FLAGS", []):
        m = re.search(r"-D\s*C3HT_EU_SECURITY_VERSION=(\d+)", str(fl))
        if m:
            sv = int(m.group(1))
    keys = ""
    hdr = os.path.join("include", "c3ht_eu_pubkeys.h")
    if os.path.exists(hdr):
        m = re.search(r'C3HT_EU_KEY_IDS_STR "([0-9a-f,]*)"', open(hdr).read())
        keys = m.group(1) if m else ""
    man = {"env": env["PIOENV"], "target": re.sub("_VIA_.*", "", target_name), "security_version": sv,
           "accepted_key_ids": [k for k in keys.split(",") if k], "file": os.path.basename(path),
           "size": len(data), "sha256": hashlib.sha256(data).hexdigest(),
           "sign": "python -I firmware/eu-build/tools/sign_fw.py sign --key <private.pem> %s" % path}
    with open(os.path.join(os.path.dirname(path), "firmware.eu.json"), "w") as f:
        json.dump(man, f, indent=2)


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", write_manifest)
