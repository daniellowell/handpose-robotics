#!/usr/bin/env python3
"""
Analyze handpose-robotics session logs (NDJSON format)

Usage:
  python3 analyze_log.py session.log
  python3 analyze_log.py session.log --events serial_send
  python3 analyze_log.py session.log --stats
  python3 analyze_log.py session.log --angles thumb_ip
  python3 analyze_log.py session.log --jitter index
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path


def load_log(path):
    """Load NDJSON log file, return list of events."""
    events = []
    with open(path, 'r') as f:
        for line_num, line in enumerate(f, 1):
            line = line.strip()
            if not line:
                continue
            try:
                events.append(json.loads(line))
            except json.JSONDecodeError as e:
                print(f"[WARN] Line {line_num}: {e}", file=sys.stderr)
    return events


def filter_events(events, event_types):
    """Filter events by type."""
    if not event_types:
        return events
    types = set(event_types)
    return [e for e in events if e.get('event') in types]


def print_events(events, keys=None):
    """Pretty-print events, optionally filtering keys."""
    for e in events:
        if keys:
            filtered = {k: e.get(k) for k in keys if k in e}
            print(json.dumps(filtered, indent=2))
        else:
            print(json.dumps(e, indent=2))


def compute_stats(events):
    """Compute summary statistics from session log."""
    stats = {
        'total_events': len(events),
        'event_counts': defaultdict(int),
        'session_duration': None,
        'frames_processed': 0,
        'hand_detections': 0,
        'send_rate': None,
        'avg_fps': None,
        'angle_ranges': {},
    }

    # Count event types
    for e in events:
        stats['event_counts'][e.get('event', 'unknown')] += 1

    # Session duration
    timestamps = [e['ts'] for e in events if 'ts' in e]
    if len(timestamps) >= 2:
        stats['session_duration'] = timestamps[-1] - timestamps[0]

    # Analyze serial_send events
    send_events = [e for e in events if e.get('event') == 'serial_send']
    if send_events:
        stats['frames_processed'] = len(send_events)
        stats['hand_detections'] = sum(1 for e in send_events if e.get('hand_present'))

        # Average FPS
        fps_vals = [e['fps'] for e in send_events if 'fps' in e]
        if fps_vals:
            stats['avg_fps'] = sum(fps_vals) / len(fps_vals)

        # Send rate
        elapsed = [e['elapsed_since_last'] for e in send_events if 'elapsed_since_last' in e]
        if elapsed:
            stats['send_rate'] = 1.0 / (sum(elapsed) / len(elapsed))

        # Angle ranges
        for e in send_events:
            if 'angles' in e:
                for key, val in e['angles'].items():
                    if val is not None:
                        if key not in stats['angle_ranges']:
                            stats['angle_ranges'][key] = {'min': val, 'max': val, 'vals': []}
                        stats['angle_ranges'][key]['min'] = min(stats['angle_ranges'][key]['min'], val)
                        stats['angle_ranges'][key]['max'] = max(stats['angle_ranges'][key]['max'], val)
                        stats['angle_ranges'][key]['vals'].append(val)

        # Compute angle std dev
        for key, data in stats['angle_ranges'].items():
            vals = data['vals']
            if len(vals) > 1:
                mean = sum(vals) / len(vals)
                variance = sum((x - mean) ** 2 for x in vals) / len(vals)
                data['mean'] = round(mean, 1)
                data['stddev'] = round(variance ** 0.5, 1)
            del data['vals']  # Don't print all values

    return dict(stats)


def analyze_jitter(events, finger_index):
    """Analyze jitter for a specific finger (0-5)."""
    send_events = [e for e in events if e.get('event') == 'serial_send']

    # Extract smoothed percentages for the finger
    values = []
    for e in send_events:
        pcts = e.get('smoothed_percentages')
        if pcts and len(pcts) > finger_index:
            values.append(pcts[finger_index])

    if len(values) < 2:
        print("Not enough data to analyze jitter")
        return

    # Compute frame-to-frame deltas
    deltas = [abs(values[i] - values[i-1]) for i in range(1, len(values))]

    # Stats
    avg_delta = sum(deltas) / len(deltas)
    max_delta = max(deltas)

    # Count large jumps (>10%)
    large_jumps = sum(1 for d in deltas if d > 10)

    finger_names = ["thumb", "index", "middle", "ring", "pinky", "swivel"]

    print(f"Jitter analysis for {finger_names[finger_index]}:")
    print(f"  Samples: {len(values)}")
    print(f"  Value range: {min(values)} - {max(values)}")
    print(f"  Avg frame-to-frame change: {avg_delta:.2f}%")
    print(f"  Max frame-to-frame change: {max_delta}%")
    print(f"  Large jumps (>10%): {large_jumps} ({100*large_jumps/len(deltas):.1f}%)")


def plot_angles(events, angle_key):
    """Plot angle over time (simple text-based visualization)."""
    send_events = [e for e in events if e.get('event') == 'serial_send']

    angles = []
    timestamps = []
    for e in send_events:
        if 'angles' in e and angle_key in e['angles'] and e['angles'][angle_key] is not None:
            angles.append(e['angles'][angle_key])
            timestamps.append(e.get('ts', 0))

    if not angles:
        print(f"No data found for angle '{angle_key}'")
        return

    # Normalize to 0-50 for text plot
    min_a, max_a = min(angles), max(angles)
    if max_a == min_a:
        print(f"{angle_key} is constant at {min_a}°")
        return

    print(f"{angle_key} over time ({min_a:.1f}° - {max_a:.1f}°):")
    print()

    for i, (ts, angle) in enumerate(zip(timestamps, angles)):
        # Simple bar chart
        normalized = int(50 * (angle - min_a) / (max_a - min_a))
        bar = '█' * normalized
        print(f"{i:4d} | {bar} {angle:.1f}°")

        # Only show first 50 samples to avoid spam
        if i >= 49:
            print(f"... ({len(angles) - 50} more samples)")
            break


def diagnostic_summary(events):
    """Generate comprehensive diagnostic summary with recommendations."""
    print("=" * 70)
    print("HANDPOSE-ROBOTICS SESSION DIAGNOSTIC SUMMARY")
    print("=" * 70)
    print()

    # Basic session info
    stats = compute_stats(events)

    print("📊 SESSION INFO")
    print("-" * 70)
    env = next((e for e in events if e.get('event') == 'env_snapshot'), None)
    if env:
        print(f"  Python:     {env.get('python', 'unknown')}")
        print(f"  OpenCV:     {env.get('opencv', 'unknown')}")
        print(f"  MediaPipe:  {env.get('mediapipe', 'unknown')}")
        print(f"  Port:       {env.get('port', 'unknown')}")

    live_start = next((e for e in events if e.get('event') == 'live_start'), None)
    if live_start:
        print(f"  Send rate:  {live_start.get('send_hz', '?')} Hz")
        print(f"  Smoothing:  {live_start.get('smooth', '?')}")
        print(f"  Finger open/close:  {live_start.get('finger_open_deg', '?')}° → {live_start.get('finger_close_deg', '?')}°")
        print(f"  Thumb open/close:   {live_start.get('thumb_open_deg', '?')}° → {live_start.get('thumb_close_deg', '?')}°")
        print(f"  DIP blend:  {live_start.get('dip_blend', '?')}")

    if stats['session_duration']:
        print(f"  Duration:   {stats['session_duration']:.1f}s")
    print()

    # Performance metrics
    print("⚡ PERFORMANCE")
    print("-" * 70)
    if stats['avg_fps']:
        print(f"  Average FPS:     {stats['avg_fps']:.1f}")
        if stats['avg_fps'] < 20:
            print("    ⚠️  WARNING: Low FPS detected. Camera or MediaPipe may be slow.")
        elif stats['avg_fps'] > 25:
            print("    ✓ Good camera performance")

    if stats['send_rate']:
        print(f"  Actual send rate: {stats['send_rate']:.1f} Hz")
        expected = live_start.get('send_hz', 20) if live_start else 20
        if abs(stats['send_rate'] - expected) > 2:
            print(f"    ⚠️  WARNING: Expected {expected} Hz, got {stats['send_rate']:.1f} Hz")
        else:
            print(f"    ✓ Send rate matches target ({expected} Hz)")

    if stats['frames_processed'] > 0:
        detection_rate = 100 * stats['hand_detections'] / stats['frames_processed']
        print(f"  Hand detection:   {detection_rate:.1f}% ({stats['hand_detections']}/{stats['frames_processed']} frames)")
        if detection_rate < 70:
            print("    ⚠️  WARNING: Low detection rate. Check lighting and hand visibility.")
        elif detection_rate > 90:
            print("    ✓ Excellent detection rate")
    print()

    # Angle analysis
    print("📐 JOINT ANGLE ANALYSIS")
    print("-" * 70)
    if stats['angle_ranges']:
        for angle_name, data in sorted(stats['angle_ranges'].items()):
            print(f"  {angle_name:12s}  {data['min']:6.1f}° → {data['max']:6.1f}°  "
                  f"(μ={data['mean']:6.1f}°, σ={data['stddev']:5.1f}°)")

            # Detect potential issues
            if 'pip' in angle_name or 'dip' in angle_name:
                # Fingers should vary more
                if data['max'] - data['min'] < 30:
                    print(f"    ⚠️  Low range. Finger may not be flexing much.")
                if data['stddev'] < 5:
                    print(f"    ℹ️  Low variance. Mostly static position.")

            if 'thumb_ip' in angle_name:
                if data['max'] - data['min'] < 20:
                    print(f"    ⚠️  Low range. Thumb may not be flexing much.")
    else:
        print("  No angle data available (old log format or no hand detected)")
    print()

    # Servo status analysis
    print("🔧 SERVO FEEDBACK")
    print("-" * 70)
    servo_status = [e for e in events if e.get('event') == 'servo_status']
    if servo_status:
        # Get most recent status
        latest = servo_status[-1]
        servos = latest.get('servo_angles', [])
        limits = latest.get('servo_limits', {})
        finger_names = ["thumb", "index", "middle", "ring", "pinky", "swivel"]

        if servos and limits:
            mins = limits.get('min', [])
            maxs = limits.get('max', [])
            print(f"  Latest servo positions (from {len(servo_status)} status reports):")
            for i, name in enumerate(finger_names):
                if i < len(servos) and i < len(mins) and i < len(maxs):
                    pos = servos[i]
                    min_val = mins[i]
                    max_val = maxs[i]
                    pct = 100 * (pos - min_val) / (max_val - min_val) if max_val > min_val else 0
                    print(f"    {name:8s}  {pos:3d}° (calibrated: {min_val:3d}° - {max_val:3d}°, at {pct:5.1f}%)")
        else:
            print("  Status data incomplete")
    else:
        print("  No servo status available (requires updated firmware with STATUS command)")
    print()

    # Jitter analysis for all fingers
    print("🔧 JITTER ANALYSIS")
    print("-" * 70)
    finger_names = ["thumb", "index", "middle", "ring", "pinky", "swivel"]
    send_events = [e for e in events if e.get('event') == 'serial_send']

    if send_events:
        for i, name in enumerate(finger_names):
            values = []
            for e in send_events:
                pcts = e.get('smoothed_percentages')
                if pcts and len(pcts) > i:
                    values.append(pcts[i])

            if len(values) > 1:
                deltas = [abs(values[j] - values[j-1]) for j in range(1, len(values))]
                avg_delta = sum(deltas) / len(deltas)
                max_delta = max(deltas)
                large_jumps = sum(1 for d in deltas if d > 10)
                jump_pct = 100 * large_jumps / len(deltas)

                status = "✓" if avg_delta < 3 and jump_pct < 2 else "⚠️" if avg_delta < 5 else "❌"
                print(f"  {status} {name:8s}  avg Δ={avg_delta:4.1f}%  max Δ={max_delta:3.0f}%  "
                      f"jumps={jump_pct:4.1f}%")

                if avg_delta > 5:
                    print(f"      → High jitter. Consider increasing --dip-blend or --smooth")
                if jump_pct > 5:
                    print(f"      → Frequent large jumps. Check angle thresholds.")
    else:
        print("  No data available")
    print()

    # Error analysis
    print("🔍 ERRORS & WARNINGS")
    print("-" * 70)
    errors = [e for e in events if 'error' in e.get('event', '').lower() or 'error' in e]
    warnings = [e for e in events if e.get('event') in ['mediapipe_error', 'serial_write_error', 'camera_frame_error']]

    if errors or warnings:
        for e in (errors + warnings)[:10]:  # Show first 10
            print(f"  [{e.get('event', 'unknown')}] {e.get('error', e.get('reason', 'unknown'))}")
        if len(errors + warnings) > 10:
            print(f"  ... and {len(errors + warnings) - 10} more")
    else:
        print("  ✓ No errors detected")
    print()

    # Recommendations
    print("💡 RECOMMENDATIONS")
    print("-" * 70)
    recommendations = []

    if stats['avg_fps'] and stats['avg_fps'] < 20:
        recommendations.append("• Reduce camera resolution or close other applications")

    for angle_name, data in stats['angle_ranges'].items():
        if 'pip' in angle_name and data['max'] - data['min'] < 30:
            finger = angle_name.split('_')[0]
            recommendations.append(f"• {finger.capitalize()} has low range - adjust --finger-open/--finger-close thresholds")

    # Check jitter
    for i, name in enumerate(finger_names):
        values = []
        for e in send_events:
            pcts = e.get('smoothed_percentages')
            if pcts and len(pcts) > i:
                values.append(pcts[i])

        if len(values) > 1:
            deltas = [abs(values[j] - values[j-1]) for j in range(1, len(values))]
            avg_delta = sum(deltas) / len(deltas)
            if avg_delta > 5:
                recommendations.append(f"• High jitter on {name} - try --dip-blend 0.5 or --smooth 0.5")
                break  # Only suggest once

    if not recommendations:
        recommendations.append("✓ No issues detected - system performing well!")

    for rec in recommendations:
        print(f"  {rec}")

    print()
    print("=" * 70)


def main():
    parser = argparse.ArgumentParser(description="Analyze handpose-robotics session logs")
    parser.add_argument("logfile", type=Path, help="Path to NDJSON log file")
    parser.add_argument("--events", nargs='+', help="Filter by event types")
    parser.add_argument("--keys", nargs='+', help="Show only specific keys")
    parser.add_argument("--stats", action='store_true', help="Show session statistics")
    parser.add_argument("--jitter", choices=['thumb', 'index', 'middle', 'ring', 'pinky', 'swivel'],
                        help="Analyze jitter for specific finger")
    parser.add_argument("--angles", help="Plot angle over time (e.g., thumb_ip, index_pip)")
    parser.add_argument("--diagnostic", action='store_true', help="Full diagnostic summary (default if no other options)")

    args = parser.parse_args()

    if not args.logfile.exists():
        print(f"Error: {args.logfile} not found", file=sys.stderr)
        sys.exit(1)

    events = load_log(args.logfile)

    if not events:
        print("No events found in log", file=sys.stderr)
        sys.exit(1)

    # Default to diagnostic if no specific analysis requested
    if not any([args.stats, args.jitter, args.angles, args.events, args.diagnostic]):
        args.diagnostic = True

    if args.diagnostic:
        diagnostic_summary(events)
    elif args.stats:
        stats = compute_stats(events)
        print(json.dumps(stats, indent=2))
    elif args.jitter:
        finger_map = {'thumb': 0, 'index': 1, 'middle': 2, 'ring': 3, 'pinky': 4, 'swivel': 5}
        analyze_jitter(events, finger_map[args.jitter])
    elif args.angles:
        plot_angles(events, args.angles)
    else:
        filtered = filter_events(events, args.events)
        print_events(filtered, args.keys)


if __name__ == "__main__":
    main()
