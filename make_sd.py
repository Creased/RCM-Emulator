#!/usr/bin/env python3
"""Build a FAT32 SD image from a source folder.

The output is an `sd.img` containing a single bootable FAT32 partition
populated with the contents of `--src`. Usable directly with rcm_emu via
`--sd sd.img`.

Requires Docker (uses an alpine container for dosfstools, mtools and
util-linux).

Usage:
    python3 make_sd.py --src ./sd_root --out sd.img [--size-mb 512]
"""
import argparse
import os
import shlex
import subprocess
import sys


def build_sd_image(src_dir: str, out_img: str, size_mb: int) -> int:
    if not os.path.isdir(src_dir):
        print(f"[error] source folder not found: {src_dir}")
        return 1
    if size_mb < 64:
        print("[error] --size-mb must be at least 64: FAT32 needs 65525 clusters")
        return 1

    src_dir_abs = os.path.abspath(src_dir)
    out_img_abs = os.path.abspath(out_img)
    work_dir = os.path.dirname(out_img_abs) or os.getcwd()
    img_name = os.path.basename(out_img_abs)
    fat_name = img_name + ".fat"

    print(f"[*] source : {src_dir_abs}")
    print(f"[*] output : {out_img_abs}")
    print(f"[*] size   : {size_mb} MB")

    # Create a blank disk image and a slightly smaller blank FAT image.
    print(f"[*] allocating blank image files")
    disk_path = os.path.join(work_dir, img_name)
    fat_path = os.path.join(work_dir, fat_name)
    with open(disk_path, "wb") as f:
        f.seek(size_mb * 1024 * 1024 - 1)
        f.write(b"\0")
    with open(fat_path, "wb") as f:
        f.seek((size_mb - 1) * 1024 * 1024 - 1)
        f.write(b"\0")

    print(f"[*] formatting + populating via Docker (alpine)")
    fdisk_script = f"o\\nn\\np\\n1\\n2048\\n\\nt\\nc\\nw\\n"
    # FatFs (and every FAT driver) decides FAT12/16/32 by cluster count, and
    # FAT32 needs at least 65525 clusters: 4 KiB clusters only get there from
    # ~260 MB up. Below that, format with 512-byte clusters, or the payload's
    # f_mount rejects the volume (FR_NO_FILESYSTEM).
    spc = 8 if size_mb >= 512 else 1
    # File names go into a shell command line: quote them.
    q_fat, q_img = shlex.quote(fat_name), shlex.quote(img_name)
    docker_cmd = [
        "docker", "run", "--rm",
        "-v", f"{work_dir}:/work",
        "-v", f"{src_dir_abs}:/src:ro",
        "-w", "/work",
        "alpine",
        "sh", "-c",
        "apk add --no-cache dosfstools mtools util-linux >/dev/null && "
        f"mkfs.fat -F 32 -s {spc} {q_fat} && "
        # Every top-level entry, dotfiles included; an empty source is fine.
        f"(cd /src && find . -mindepth 1 -maxdepth 1 "
        f"-exec mcopy -i /work/{q_fat} -s {{}} :: \\;) && "
        f"printf '{fdisk_script}' | fdisk {q_img} && "
        f"dd if={q_fat} of={q_img} bs=1M seek=1 conv=notrunc status=none"
    ]
    try:
        subprocess.run(docker_cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[error] docker step failed: {e}")
        try:
            os.remove(fat_path)
        except OSError:
            pass
        return 1
    except FileNotFoundError:
        print("[error] docker not found. Install Docker Desktop or docker.io and retry.")
        try:
            os.remove(fat_path)
        except OSError:
            pass
        return 1

    print("[*] cleanup")
    try:
        os.remove(fat_path)
    except OSError:
        pass

    print(f"[+] wrote {out_img_abs} ({size_mb} MB)")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="Build a FAT32 SD image for rcm_emu.")
    ap.add_argument("--src", required=True, help="folder whose contents go into the FAT32 partition")
    ap.add_argument("--out", default="sd.img", help="output image path (default: sd.img)")
    ap.add_argument("--size-mb", type=int, default=512, help="image size in MB (default: 512)")
    args = ap.parse_args()
    return build_sd_image(args.src, args.out, args.size_mb)


if __name__ == "__main__":
    sys.exit(main())
