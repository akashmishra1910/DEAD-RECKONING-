"""
scripts/download_lfs_dataset.py
===============================
Fetches the full authentic Git LFS dataset (Synchronised V and S datasets.zip)
from https://github.com/onyekpeu/IO-VNBD.git using the GitHub LFS Batch API.
"""

import os
import sys
import json
import zipfile
import urllib.request

LFS_BATCH_URL = "https://github.com/onyekpeu/IO-VNBD.git/info/lfs/objects/batch"
ZIP_OID = "624003b0bfb3d221114eb262dd02f21f7dba74fb25d8045b4b8ac684956d2855"
ZIP_SIZE = 203606286
TARGET_ZIP = "Synchronised_V_and_S_datasets.zip"
EXTRACT_DIR = "synchronised_iovnbd_data"


def download_lfs_zip():
    if os.path.exists(TARGET_ZIP) and os.path.getsize(TARGET_ZIP) == ZIP_SIZE:
        print(f"[*] {TARGET_ZIP} already exists with full size ({ZIP_SIZE} bytes).")
        return TARGET_ZIP

    print(f"[*] Requesting download URL from Git LFS Batch API: {LFS_BATCH_URL}...")
    req_body = {
        "operation": "download",
        "transfers": ["basic"],
        "objects": [{"oid": ZIP_OID, "size": ZIP_SIZE}]
    }

    req = urllib.request.Request(
        LFS_BATCH_URL,
        data=json.dumps(req_body).encode("utf-8"),
        headers={
            "Accept": "application/vnd.git-lfs+json",
            "Content-Type": "application/vnd.git-lfs+json",
            "User-Agent": "git-lfs"
        }
    )
    import ssl
    try:
        import certifi
        ssl_ctx = ssl.create_default_context(cafile=certifi.where())
    except Exception:
        ssl_ctx = ssl._create_unverified_context()

    with urllib.request.urlopen(req, context=ssl_ctx) as resp:
        res = json.loads(resp.read().decode("utf-8"))
        obj = res["objects"][0]
        download_url = obj["actions"]["download"]["href"]
        print(f"[+] Download URL acquired from GitHub LFS storage!")

    print(f"[*] Downloading {ZIP_SIZE / (1024*1024):.1f} MB archive to {TARGET_ZIP}...")
    with urllib.request.urlopen(download_url, context=ssl_ctx) as u, open(TARGET_ZIP, 'wb') as f:
        downloaded = 0
        while True:
            chunk = u.read(1024 * 1024) # 1MB chunks
            if not chunk:
                break
            f.write(chunk)
            downloaded += len(chunk)
            percent = min(100.0, downloaded / ZIP_SIZE * 100)
            sys.stdout.write(f"\r    Progress: {percent:5.1f}% ({downloaded / (1024*1024):.1f} / {ZIP_SIZE / (1024*1024):.1f} MB)")
            sys.stdout.flush()
    print("\n[+] Download complete!")
    return TARGET_ZIP


def extract_zip(zip_path):
    print(f"[*] Extracting {zip_path} into {EXTRACT_DIR}...")
    os.makedirs(EXTRACT_DIR, exist_ok=True)
    with zipfile.ZipFile(zip_path, 'r') as zf:
        zf.extractall(EXTRACT_DIR)
    print(f"[+] Extraction complete into {EXTRACT_DIR}/")


if __name__ == "__main__":
    zip_path = download_lfs_zip()
    extract_zip(zip_path)
