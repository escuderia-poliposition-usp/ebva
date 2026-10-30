"""Exporta uma vista XY em PDF de uma nuvem PCD ASCII/binária não comprimida.

filled agrupa pontos em células e scatter desenha os pontos individualmente.
As coordenadas e unidades são as do arquivo de entrada. Nenhum recurso ROS,
sensor ou janela gráfica é iniciado na importação.
"""

import argparse
import os
import sys

import numpy as np


# PCD TYPE letter -> numpy kind
_NP_KIND = {'F': 'f', 'I': 'i', 'U': 'u'}


def read_pcd_xyz(path):
    """Parse a PCD file (ascii or binary, uncompressed) into an (N, 3) XYZ array."""
    with open(path, 'rb') as f:
        fields, sizes, types, counts = [], [], [], []
        n_points = None
        data_fmt = None

        while True:
            raw = f.readline()
            if not raw:
                raise ValueError('Unexpected EOF while reading PCD header')
            line = raw.decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            if key == 'FIELDS':
                fields = vals
            elif key == 'SIZE':
                sizes = [int(v) for v in vals]
            elif key == 'TYPE':
                types = vals
            elif key == 'COUNT':
                counts = [int(v) for v in vals]
            elif key == 'POINTS':
                n_points = int(vals[0])
            elif key == 'DATA':
                data_fmt = vals[0]
                break

        if not fields or n_points is None or data_fmt is None:
            raise ValueError('Malformed PCD header (missing FIELDS/POINTS/DATA)')
        for f_name in ('x', 'y', 'z'):
            if f_name not in fields:
                raise ValueError("PCD is missing the '%s' field" % f_name)
        if not counts:
            counts = [1] * len(fields)
        if len(counts) != len(fields) or any(count < 1 for count in counts):
            raise ValueError('PCD COUNT must have one positive value per field')
        if any(counts[fields.index(name)] != 1 for name in ('x', 'y', 'z')):
            raise ValueError('PCD coordinates x/y/z must each have COUNT 1')

        if data_fmt == 'ascii':
            cols = {}
            offset = 0
            for name, count in zip(fields, counts):
                cols[name] = offset
                offset += count
            xs, ys, zs = [], [], []
            for raw in f:
                line = raw.decode('ascii', 'replace').strip()
                if not line:
                    continue
                parts = line.split()
                if len(parts) != offset:
                    raise ValueError('PCD ASCII row does not match FIELDS/COUNT')
                xs.append(float(parts[cols['x']]))
                ys.append(float(parts[cols['y']]))
                zs.append(float(parts[cols['z']]))
            return np.array([xs, ys, zs], dtype=np.float64).T

        if data_fmt != 'binary':
            raise ValueError("Unsupported PCD DATA format '%s' (only ascii/binary)"
                             % data_fmt)

        # binary: build a structured dtype matching the per-point record layout
        np_fields = []
        for name, size, tp, cnt in zip(fields, sizes, types, counts):
            base = '%s%d' % (_NP_KIND[tp], size)
            np_fields.append((name, base) if cnt == 1 else (name, base, (cnt,)))
        dt = np.dtype(np_fields)
        buf = f.read(n_points * dt.itemsize)
        rec = np.frombuffer(buf, dtype=dt, count=n_points)
        return np.stack([rec['x'], rec['y'], rec['z']], axis=1).astype(np.float64)


def render(xyz, args):
    import matplotlib
    matplotlib.use('Agg')  # headless-safe; PDF export works regardless
    import matplotlib.pyplot as plt

    x, y = xyz[:, 0], xyz[:, 1]
    if x.size == 0:
        raise ValueError('Point cloud is empty, nothing to render')

    fig, ax = plt.subplots(figsize=(11, 8.5))

    if args.style == 'scatter':
        ax.scatter(x, y, s=args.point_size, c='k', marker='.', linewidths=0)
    else:  # filled
        res = args.resolution
        xedges = np.arange(x.min() - res, x.max() + 2 * res, res)
        yedges = np.arange(y.min() - res, y.max() + 2 * res, res)
        hist, _, _ = np.histogram2d(x, y, bins=[xedges, yedges])
        extent = [xedges[0], xedges[-1], yedges[0], yedges[-1]]
        if args.density:
            data = np.ma.masked_equal(hist.T, 0)
            im = ax.imshow(data, origin='lower', extent=extent,
                           cmap='inferno', aspect='equal', interpolation='nearest')
            fig.colorbar(im, ax=ax, label='pontos / célula', shrink=0.7)
        else:
            ax.imshow((hist.T > 0), origin='lower', extent=extent,
                      cmap='Greys', aspect='equal', interpolation='nearest',
                      vmin=0, vmax=1)

    ax.set_aspect('equal')
    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_title(args.title or os.path.basename(args.input))
    ax.grid(True, linestyle=':', alpha=0.4)

    with open(args.output, 'xb') as output:
        fig.savefig(output, format='pdf', bbox_inches='tight')
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Render a top-down (X-Y) PDF view of a .pcd map.')
    parser.add_argument('input', help='input .pcd file')
    parser.add_argument('-o', '--output',
                        help='output .pdf (default: input with .pdf extension)')
    parser.add_argument('--style', choices=['filled', 'scatter'], default='filled',
                        help='render style (default: filled)')
    parser.add_argument('--resolution', type=float, default=0.05,
                        help='grid cell size in m for filled style (default: 0.05)')
    parser.add_argument('--point-size', type=float, default=1.0,
                        help='marker size for scatter style (default: 1.0)')
    parser.add_argument('--density', action='store_true',
                        help='filled style: shade cells by point count instead of '
                             'binary occupancy')
    parser.add_argument('--title', help='plot title (default: input filename)')
    args = parser.parse_args(argv)

    if not args.output:
        root, _ = os.path.splitext(args.input)
        args.output = root + '.pdf'

    try:
        xyz = read_pcd_xyz(args.input)
    except (OSError, ValueError) as e:
        print('error: failed to read %s: %s' % (args.input, e), file=sys.stderr)
        return 1

    try:
        render(xyz, args)
    except ImportError:
        print('error: matplotlib is required. Install with:\n'
              '  sudo apt install -y python3-matplotlib', file=sys.stderr)
        return 1
    except (OSError, ValueError) as e:
        print('error: %s' % e, file=sys.stderr)
        return 1

    print('Wrote %s (%d points, style=%s)' % (args.output, xyz.shape[0], args.style))
    return 0


if __name__ == '__main__':
    sys.exit(main())
