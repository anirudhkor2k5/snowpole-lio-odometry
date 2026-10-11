"""Read-only ROS1 bag index inspection.

Chunk payloads are not decompressed and the bag is never opened for writing.
Message counts come from CHUNK_INFO records in the index.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from pathlib import Path

MAGIC = b"#ROSBAG V2.0\n"
OP_CHUNK_INFO = 0x06
OP_CONNECTION = 0x07
# Chunk timestamps outside this window are recorded as index anomalies.
# The 2024-02-28 recording is near 1.709e9 unix seconds.
SANE_UNIX_NS = (1.6e18, 1.9e18)


@dataclass
class BagTopic:
    connection_id: int
    name: str
    msg_type: str
    message_count: int
    pure_chunk_count: int


@dataclass
class BagIndex:
    path: str
    size_bytes: int
    connection_count_header: int
    chunk_count_header: int
    topics: list[BagTopic]
    sane_start_unix_s: float | None
    sane_end_unix_s: float | None
    n_chunk_infos: int
    n_chunks_with_nonsane_time: int

    def topic(self, name: str) -> BagTopic | None:
        for item in self.topics:
            if item.name == name:
                return item
        return None


def _fields(buf: bytes) -> dict[str, bytes]:
    out: dict[str, bytes] = {}
    index = 0
    while index + 4 <= len(buf):
        field_len = struct.unpack_from("<I", buf, index)[0]
        index += 4
        if field_len <= 0 or index + field_len > len(buf):
            break
        raw = buf[index : index + field_len]
        index += field_len
        if b"=" in raw:
            key, value = raw.split(b"=", 1)
            out[key.decode()] = value
    return out


def _read_record(handle) -> tuple[dict[str, bytes], int] | None:
    header_len_raw = handle.read(4)
    if len(header_len_raw) < 4:
        return None
    header_len = struct.unpack("<I", header_len_raw)[0]
    header = _fields(handle.read(header_len))
    data_len = struct.unpack("<I", handle.read(4))[0]
    return header, data_len


def read_bag_index(path: Path) -> BagIndex:
    path = Path(path)
    size = path.stat().st_size
    with path.open("rb") as handle:
        magic = handle.read(len(MAGIC))
        if magic != MAGIC:
            raise ValueError(f"{path} is not a ROS1 bag v2 file")
        header_record = _read_record(handle)
        if header_record is None:
            raise ValueError(f"{path} has no file header")
        header, data_len = header_record
        handle.seek(data_len, 1)
        index_pos = struct.unpack("<Q", header["index_pos"])[0]
        conn_count = struct.unpack("<I", header["conn_count"])[0]
        chunk_count = struct.unpack("<I", header["chunk_count"])[0]
        handle.seek(index_pos)
        connections: dict[int, tuple[str, str]] = {}
        counts: dict[int, int] = {}
        pure: dict[int, int] = {}
        sane_start = None
        sane_end = None
        n_chunk = 0
        n_bad_time = 0
        while True:
            record = _read_record(handle)
            if record is None:
                break
            rec_header, rec_len = record
            op = rec_header.get("op", b"\x00")[0]
            if op == OP_CONNECTION:
                payload = _fields(handle.read(rec_len))
                cid = struct.unpack("<I", rec_header["conn"])[0]
                connections[cid] = (
                    rec_header.get("topic", b"").decode(),
                    payload.get("type", b"").decode(),
                )
            elif op == OP_CHUNK_INFO:
                start_ns = struct.unpack("<Q", rec_header["start_time"])[0]
                end_ns = struct.unpack("<Q", rec_header["end_time"])[0]
                data = handle.read(rec_len)
                ids: list[int] = []
                for offset in range(0, len(data), 8):
                    cid, count = struct.unpack_from("<II", data, offset)
                    counts[cid] = counts.get(cid, 0) + count
                    ids.append(cid)
                if len(set(ids)) == 1 and ids:
                    pure[ids[0]] = pure.get(ids[0], 0) + 1
                n_chunk += 1
                if SANE_UNIX_NS[0] <= start_ns <= SANE_UNIX_NS[1] and SANE_UNIX_NS[0] <= end_ns <= SANE_UNIX_NS[1]:
                    sane_start = start_ns if sane_start is None else min(sane_start, start_ns)
                    sane_end = end_ns if sane_end is None else max(sane_end, end_ns)
                else:
                    n_bad_time += 1
            else:
                handle.seek(rec_len, 1)
    topics = [
        BagTopic(
            connection_id=cid,
            name=name,
            msg_type=msg_type,
            message_count=counts.get(cid, 0),
            pure_chunk_count=pure.get(cid, 0),
        )
        for cid, (name, msg_type) in sorted(connections.items())
    ]
    return BagIndex(
        path=str(path.resolve()),
        size_bytes=size,
        connection_count_header=conn_count,
        chunk_count_header=chunk_count,
        topics=topics,
        sane_start_unix_s=None if sane_start is None else sane_start / 1e9,
        sane_end_unix_s=None if sane_end is None else sane_end / 1e9,
        n_chunk_infos=n_chunk,
        n_chunks_with_nonsane_time=n_bad_time,
    )
