#!/usr/bin/env python3
"""
Reformat TUM trajectory files to place qw between tz and qx.

Input (standard TUM):
  timestamp tx ty tz qx qy qz qw  # 8 columns
or sometimes 7 columns (no orientation):
  timestamp tx ty tz              # 4 columns

Output:
  timestamp tx ty tz qw qx qy qz  # qw moved before qx

Usage:
  python format_path.py input.tum -o output.tum
  python format_path.py input.tum --in-place
  cat input.tum | python format_path.py > output.tum

Options:
  --delimiter auto|space|comma  (default: auto)
"""

import argparse
import sys
from typing import List, Optional


def parse_line(line: str, delimiter: Optional[str] = None) -> Optional[List[str]]:
    line = line.strip()
    if not line or line.startswith('#'):
        return None
    if delimiter == 'comma' or (delimiter is None and ',' in line):
        parts = [p.strip() for p in line.split(',') if p.strip() != '']
    else:
        parts = line.split()
    return parts


def reorder_columns(parts: List[str]) -> Optional[str]:
    # Expect 8 columns: ts tx ty tz qx qy qz qw
    # Want:              ts tx ty tz qw qx qy qz
    n = len(parts)
    if n == 8:
        try:
            ts, tx, ty, tz, qx, qy, qz, qw = parts
        except ValueError:
            return None
        return f"{float(ts)/1000000000} {tx} {ty} {tz} {qw} {qx} {qy} {qz}"
    elif n == 7:
        # Some files may miss one quaternion component; reject to avoid wrong mapping
        return None
    elif n == 4:
        # No orientation: pass through unchanged
        ts, tx, ty, tz = parts
        return f"{ts} {tx} {ty} {tz}"
    else:
        # Unknown format length; ignore
        return None


def process_stream(fin, fout, delimiter_opt: str = 'auto') -> int:
    count = 0
    for raw in fin:
        parts = parse_line(raw, None if delimiter_opt == 'auto' else ('comma' if delimiter_opt == 'comma' else 'space'))
        if parts is None:
            continue
        out = reorder_columns(parts)
        if out is None:
            # If the line could not be parsed into expected column count, skip it
            continue
        fout.write(out + "\n")
        count += 1
    return count


def main():
    parser = argparse.ArgumentParser(description='Move qw between tz and qx in TUM files')
    parser.add_argument('input', nargs='?', help='Input TUM file (default: stdin)')
    parser.add_argument('-o', '--output', help='Output file (default: stdout)')
    parser.add_argument('--in-place', action='store_true', help='Rewrite input file in place')
    parser.add_argument('--delimiter', choices=['auto', 'space', 'comma'], default='auto', help='Input delimiter handling')

    args = parser.parse_args()

    # Determine IO
    if args.in_place:
        if not args.input:
            print('Error: --in-place requires an input file', file=sys.stderr)
            sys.exit(1)
        with open(args.input, 'r') as fin:
            lines = fin.readlines()
        from io import StringIO
        buf_in = StringIO(''.join(lines))
        from io import StringIO as SIO
        buf_out = SIO()
        count = process_stream(buf_in, buf_out, args.delimiter)
        with open(args.input, 'w') as fout:
            fout.write(buf_out.getvalue())
        print(f'Reformatted {count} lines in-place: {args.input}', file=sys.stderr)
        return

    fin = sys.stdin if not args.input else open(args.input, 'r')
    fout = sys.stdout if not args.output else open(args.output, 'w')

    try:
        count = process_stream(fin, fout, args.delimiter)
    finally:
        if fin is not sys.stdin:
            fin.close()
        if fout is not sys.stdout:
            fout.close()

    if args.output:
        print(f'Wrote {count} lines to {args.output}', file=sys.stderr)


if __name__ == '__main__':
    main()
