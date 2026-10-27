"""读 PCL 的 .pcd（ascii / binary / binary_compressed），只取 x y z。"""
import struct

import numpy as np


def _lzf_decompress(src, out_len):
    out = bytearray(out_len)
    i = o = 0
    n = len(src)
    while i < n:
        c = src[i]
        i += 1
        if c < 32:
            c += 1
            out[o:o + c] = src[i:i + c]
            i += c
            o += c
        else:
            ln = c >> 5
            if ln == 7:
                ln += src[i]
                i += 1
            ref = o - ((c & 0x1f) << 8) - src[i] - 1
            i += 1
            ln += 2
            for k in range(ln):
                out[o] = out[ref + k]
                o += 1
    return bytes(out)


_NP_TYPE = {('F', 4): np.float32, ('F', 8): np.float64,
            ('U', 1): np.uint8, ('U', 2): np.uint16, ('U', 4): np.uint32,
            ('I', 1): np.int8, ('I', 2): np.int16, ('I', 4): np.int32}


def load_pcd_xyz(path):
    """返回 (N, 3) float64"""
    with open(path, 'rb') as f:
        hdr = {}
        while True:
            line = f.readline()
            if not line:
                raise ValueError('pcd header truncated: ' + path)
            parts = line.decode('ascii', 'replace').strip().split()
            if not parts or parts[0].startswith('#'):
                continue
            hdr[parts[0]] = parts[1:]
            if parts[0] == 'DATA':
                break
        data = f.read()
    fields = hdr['FIELDS']
    sizes = [int(s) for s in hdr['SIZE']]
    types = hdr['TYPE']
    counts = [int(c) for c in hdr.get('COUNT', ['1'] * len(fields))]
    n = int(hdr['POINTS'][0])
    mode = hdr['DATA'][0]
    dtypes = [_NP_TYPE[(t, s)] for t, s in zip(types, sizes)]
    if mode == 'ascii':
        arr = np.loadtxt(data.decode().splitlines(), ndmin=2)
        col = {}
        k = 0
        for f_, c in zip(fields, counts):
            col[f_] = arr[:, k]
            k += c
        return np.stack([col['x'], col['y'], col['z']], 1).astype(np.float64)
    if mode == 'binary':
        rec = np.dtype({'names': [f'{f_}_{i}' for i, f_ in enumerate(fields)],
                        'formats': [(d, c) if c > 1 else d for d, c in zip(dtypes, counts)]})
        a = np.frombuffer(data, rec, n)
        get = {f_: a[f'{f_}_{i}'] for i, f_ in enumerate(fields)}
        return np.stack([get['x'], get['y'], get['z']], 1).astype(np.float64)
    if mode == 'binary_compressed':
        comp, raw = struct.unpack('<II', data[:8])
        buf = _lzf_decompress(data[8:8 + comp], raw)
        cols = {}
        off = 0
        for f_, s, d, c in zip(fields, sizes, dtypes, counts):
            cols[f_] = np.frombuffer(buf, d, n * c, off)[::c] if c > 1 else np.frombuffer(buf, d, n, off)
            off += s * c * n
        return np.stack([cols['x'], cols['y'], cols['z']], 1).astype(np.float64)
    raise ValueError('unsupported pcd DATA mode: ' + mode)
