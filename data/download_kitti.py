"""Download the GPS/IMU (OXTS) part of KITTI raw sequences, without camera and lidar data.

Each archive is 6 to 25 GB, of which the oxts folder is a few MB. Only that part
is read from the remote archive, using HTTP range requests.

KITTI is licensed CC BY-NC-SA 3.0: non-commercial use, with attribution.
Geiger, Lenz, Stiller, Urtasun: Vision meets Robotics: The KITTI Dataset, IJRR 2013.
"""
import argparse
import io
import urllib.request
import zipfile
from pathlib import Path

BASE = "https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data"
DRIVES = ["2011_09_30_drive_0027", "2011_09_30_drive_0033", "2011_09_30_drive_0034"]


class RemoteFile(io.RawIOBase):
    """A read-only remote file, fetched in chunks with HTTP range requests."""

    def __init__(self, url, chunk=8 << 20):
        self.url = url
        self.chunk = chunk
        head = urllib.request.urlopen(urllib.request.Request(url, method="HEAD"))
        self.size = int(head.headers["Content-Length"])
        self.pos = 0
        self.cache = b""
        self.cache_start = 0
        self.transferred = 0

    def readable(self):
        return True

    def seekable(self):
        return True

    def tell(self):
        return self.pos

    def seek(self, offset, whence=io.SEEK_SET):
        self.pos = offset + {io.SEEK_SET: 0, io.SEEK_CUR: self.pos, io.SEEK_END: self.size}[whence]
        return self.pos

    def readinto(self, buffer):
        n = min(len(buffer), self.size - self.pos)
        if n <= 0:
            return 0
        cached = self.cache_start <= self.pos and self.pos + n <= self.cache_start + len(self.cache)
        if not cached:
            last = min(self.pos + max(n, self.chunk), self.size) - 1
            request = urllib.request.Request(self.url, headers={"Range": f"bytes={self.pos}-{last}"})
            self.cache = urllib.request.urlopen(request).read()
            self.cache_start = self.pos
            self.transferred += len(self.cache)
        offset = self.pos - self.cache_start
        buffer[:n] = self.cache[offset:offset + n]
        self.pos += n
        return n


def download(drive, out):
    """Extract <date>/<drive>_extract/oxts/ of the 100 Hz archive to out/<drive>/oxts/."""
    target = out / drive / "oxts"
    if (target / "timestamps.txt").exists():
        print(f"{drive}: already there")
        return
    remote = RemoteFile(f"{BASE}/{drive}/{drive}_extract.zip")
    with zipfile.ZipFile(remote) as archive:
        members = [m for m in archive.infolist() if "/oxts/" in m.filename and not m.is_dir()]
        for member in members:
            path = target / member.filename.split("/oxts/")[1]
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(archive.read(member))
    print(f"{drive}: {len(members)} files, {remote.transferred / 1e6:.1f} MB transferred "
          f"of a {remote.size / 1e9:.1f} GB archive")


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("drives", nargs="*", default=DRIVES, help="for example 2011_09_30_drive_0033")
    parser.add_argument("--out", type=Path, default=Path("data/kitti"))
    args = parser.parse_args()
    for drive in args.drives:
        download(drive, args.out)


if __name__ == "__main__":
    main()
