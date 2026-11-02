#!/usr/bin/env python3
"""Upload a SIGNED C3HT EU firmware image over the device's Wi-Fi (http://10.0.0.1).

  python c3ht_eu_upload.py firmware.signed.bin [--host 10.0.0.1] [--other-target]

Also used by PlatformIO for the C3HTEU_*_via_WIFI envs (pio run -t upload),
which uploads .pio/build/<env>/firmware.signed.bin. Join the C3HT-xxxxxx
Wi-Fi network first. The password is asked for interactively (getpass). The
C3HT_DEVICE_PASSWORD environment variable also works but can leak through
shell history / process listings; prefer the prompt.
Stdlib only.
"""
import getpass
import http.client
import json
import os
import sys
import time


def _req(host, method, path, body=None, headers=None, timeout=30):
    c = http.client.HTTPConnection(host, 80, timeout=timeout)
    h = {"X-C3HT": "1"}
    h.update(headers or {})
    c.request(method, path, body=body, headers=h)
    r = c.getresponse()
    data = r.read()
    cookies = r.getheader("Set-Cookie", "")
    c.close()
    try:
        j = json.loads(data.decode() or "{}")
    except ValueError:
        j = {}
    return r.status, j, cookies


def upload(path, host="10.0.0.1", other_target=False, password=None):
    with open(path, "rb") as f:
        blob = f.read()
    if blob[-128:-120] != b"C3HTSIG1":
        raise SystemExit("%s is not signed (no C3HTSIG1 trailer). Sign it with sign_fw.py first." % path)
    pw = password or os.environ.get("C3HT_DEVICE_PASSWORD") or getpass.getpass("C3HT device password: ")
    st, j, cookie = _req(host, "POST", "/api/login", json.dumps({"password": pw}),
                         {"Content-Type": "application/json"})
    pw = None
    if st != 200:
        raise SystemExit("login failed (%d): %s" % (st, j.get("msg", "")))
    sid = cookie.split(";", 1)[0]
    auth = {"Cookie": sid, "X-C3HT-CSRF": j["csrf"]}
    print("uploading %s (%d bytes) ..." % (os.path.basename(path), len(blob)))
    hdr = dict(auth)
    hdr["Content-Type"] = "application/octet-stream"
    st, j, _ = _req(host, "POST", "/api/update" + ("?xtarget=1" if other_target else ""), blob, hdr, timeout=300)
    if st != 202:
        raise SystemExit("upload rejected (%d): %s" % (st, j.get("msg", "")))
    for _ in range(60):
        time.sleep(0.5)
        st, j, _ = _req(host, "GET", "/api/update/status", None, {"Cookie": sid})
        if st != 200:
            print("device stopped answering (probably rebooting)")
            return 0
        if j.get("state") in ("done", "failed"):
            print("%s: %s" % (j["state"], j.get("msg", "")))
            return 0 if j["state"] == "done" else 1
    print("timeout waiting for verification")
    return 1


def on_upload(source, target, env):
    fw = str(source[0])
    signed = os.path.join(os.path.dirname(fw), "firmware.signed.bin")
    if not os.path.exists(signed):
        raise SystemExit("No %s. Sign firmware.bin offline first:\n  python -I sign_fw.py sign --key <private.pem> %s"
                         % (signed, fw))
    if os.path.getmtime(signed) < os.path.getmtime(fw):
        raise SystemExit("%s is older than firmware.bin; sign the new build first" % signed)
    return upload(signed, env.get("UPLOAD_PORT") or "10.0.0.1")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    host = "10.0.0.1"
    if "--host" in sys.argv:
        host = sys.argv[sys.argv.index("--host") + 1]
        args = [a for a in args if a != host]
    sys.exit(upload(args[0], host, "--other-target" in sys.argv))
