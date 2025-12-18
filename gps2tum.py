#!/usr/bin/env python3
"""
Convert GPS data (latitude/longitude) to TUM trajectory format.

TUM format: timestamp tx ty tz qx qy qz qw
where:
  - timestamp: time in seconds
  - tx, ty, tz: 3D position
  - qx, qy, qz, qw: orientation quaternion (identity if not available)
"""

import argparse
import sys
import math
import csv
from typing import Tuple, List, Optional



def parse_utm_file(filepath: str) -> List[Tuple[float, float, float]]:
    """
    Parse UTM/metric coordinate data from file.
    
    Expected format:
      north(m) east(m),
      821268.5785 833016.4207,
      ...
    
    Args:
        filepath: Path to UTM coordinate file
        
    Returns:
        List of (north, east, alt) tuples (alt defaults to 0.0)
    """
    data = []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip().rstrip(',')  # Remove trailing comma
            
            # Skip empty lines and headers
            if not line or 'north' in line.lower() or 'east' in line.lower():
                continue
            
            parts = line.split()
            if len(parts) < 2:
                continue
            
            try:
                north = float(parts[0])
                east = float(parts[1])
                alt = float(parts[2]) if len(parts) > 2 else 0.0
                data.append((north, east, alt))
            except (ValueError, IndexError):
                continue
    
    return data




def convert_utm_to_tum(utm_data: List[Tuple[float, float, float]], 
                       start_timestamp: float = 0.0,
                       timestamp_step: float = 0.1,
                       relative: bool = True,
                       quat: Optional[Tuple[float, float, float, float]] = None) -> List[str]:
    """
    Convert UTM/metric coordinate data to TUM format.
    
    Args:
        utm_data: List of (north, east, alt) tuples in meters
        start_timestamp: Starting timestamp in seconds
        timestamp_step: Time step between consecutive points in seconds
        relative: If True, convert to relative coordinates (first point as origin)
        quat: Optional quaternion (qx, qy, qz, qw). If None, uses identity quaternion.
        
    Returns:
        List of TUM format strings
    """
    if not utm_data:
        return []
    
    # Default identity quaternion (no rotation)
    if quat is None:
        quat = (0.0, 0.0, 0.0, 1.0)
    
    tum_lines = []
    
    if relative:
        # Use first point as origin
        origin_north, origin_east, origin_alt = utm_data[0]
        
        for i, (north, east, alt) in enumerate(utm_data):
            timestamp = start_timestamp + i * timestamp_step
            # TUM format: x(east), y(north), z(up)
            x = east - origin_east
            y = north - origin_north
            z = alt - origin_alt
            line = f"{timestamp:.6f} {x:.6f} {y:.6f} {z:.6f} {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"
            tum_lines.append(line)
    else:
        # Use absolute UTM coordinates
        for i, (north, east, alt) in enumerate(utm_data):
            timestamp = start_timestamp + i * timestamp_step
            # TUM format: x(east), y(north), z(up)
            line = f"{timestamp:.6f} {east:.6f} {north:.6f} {alt:.6f} {quat[0]:.6f} {quat[1]:.6f} {quat[2]:.6f} {quat[3]:.6f}"
            tum_lines.append(line)
    
    return tum_lines

def get_time_params(f_input: str, length: int) -> Tuple[float, float]:
    """
    Parse time parameters from input string.
    
    Args:
        f_input: Input string in format "start_time,step" or just "start_time"
        
    Returns:
        Tuple of (start_time, step)
    """
    start_timestamp = 0.0
    end_timestamp = 0.0
    with open(f_input, 'r') as f:
        
        line = f.readline().strip()
        parts = line.split(' ')
        start_timestamp = float(parts[0]) / 1000000000
        for line in f:
            line = line.strip().rstrip(',')  # Remove trailing comma
            parts = line.split()
            end_timestamp = float(parts[0]) / 1000000000
            

    total_time = end_timestamp - start_timestamp
    step = total_time / (length - 1) if length > 1 else 0.1

                
    return start_timestamp, step

def parse_floam(filepath: str) -> List[Tuple[float, float, float]]:
    """
    Parse FLOAM pose data from file.
    
    Expected format:
      timestamp tx ty tz qx qy qz qw
      1234567890.123 1.0 2.0 3.0 0.0 0.0 0.0 1.0
      ...
    Args:
        filepath: Path to FLOAM pose file
    Returns:
        List of (tx, ty, tz) tuples
    """
    data = []
    
    with open(filepath, 'r') as f:
        for line in f:
            line = line.strip()
            
            # Skip empty lines and headers
            if not line or line.startswith('#'):
                continue
            
            parts = line.split()
            if len(parts) < 4:
                continue
            
            try:
                tx = float(parts[1])
                ty = float(parts[2])
                tz = float(parts[3])
                data.append((tx, ty, tz))
            except (ValueError, IndexError):
                continue
    
    return data


def main():
    parser = argparse.ArgumentParser(
        description='Convert GPS data (lat/lon or UTM) to TUM trajectory format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Input file formats:
  1. Lat/Lon format: timestamp latitude longitude [altitude]
     Example:
       1234567890.123 37.7749 -122.4194 10.5
  
  2. UTM/metric format: north(m) east(m)
     Example:
       north(m) east(m),
       821268.5785 833016.4207,
       821270.5785 833011.8564,

Output format (TUM):
  timestamp tx ty tz qx qy qz qw
  
Examples:
  %(prog)s input.csv -o output.txt
  %(prog)s gps_data.txt --utm -o trajectory.tum
  %(prog)s utm_path.txt --utm-input --start-time 0 --time-step 0.1 -o output.tum
        """)
    
    parser.add_argument('input', help='Input GPS data file')
    parser.add_argument('-o', '--output', help='Output TUM file (default: stdout)')
    parser.add_argument('--f-input', required=True, help='Starting timestamp in seconds (for UTM input, default: 0.0)')
    
    parser.add_argument('--quat', nargs=4, type=float, metavar=('QX', 'QY', 'QZ', 'QW'),
                        help='Quaternion orientation (default: 0 0 0 1 = identity)')
    
    args = parser.parse_args()
    
    # Parse GPS/UTM data
    try:
        # Parse UTM/metric coordinate format
        utm_data = parse_utm_file(args.input)
        if not utm_data:
            print("Error: No valid UTM data found in input file", file=sys.stderr)
            sys.exit(1)
        
        start_timestamp, step = get_time_params(args.f_input, len(utm_data))
        print(f"Parsed {len(utm_data)} UTM points", file=sys.stderr)
        
        # Convert to TUM format
        quat = tuple(args.quat) if args.quat else None
        tum_lines = convert_utm_to_tum(utm_data, 
                                        start_timestamp=start_timestamp,
                                        timestamp_step=step,
                                        relative=True,
                                        quat=quat)
        
            
    except FileNotFoundError:
        print(f"Error: Input file '{args.input}' not found", file=sys.stderr)
        sys.exit(1)
    except Exception as e:
        print(f"Error reading input file: {e}", file=sys.stderr)
        sys.exit(1)
    
    # Output
    if args.output:
        with open(args.output, 'w') as f:
            f.write('\n'.join(tum_lines) + '\n')
        print(f"Wrote {len(tum_lines)} poses to {args.output}", file=sys.stderr)
    else:
        for line in tum_lines:
            print(line)


if __name__ == '__main__':
    main()
