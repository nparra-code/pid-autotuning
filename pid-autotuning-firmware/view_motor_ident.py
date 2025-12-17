#!/usr/bin/env python3
"""
Motor Identification Data Viewer
Loads and re-analyzes existing motor identification data
"""

import sys
import os
import csv
import json
import argparse
from pathlib import Path
from motor_ident_server import MotorIdentAnalyzer

def load_csv_data(csv_path):
    """Load identification data from CSV file"""
    samples = []
    
    with open(csv_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # Skip header
        
        for row in reader:
            if len(row) >= 3:
                timestamp_ms = int(row[0])
                pwm_input = float(row[1])
                velocity_response = float(row[2])
                samples.append((timestamp_ms, pwm_input, velocity_response))
    
    return samples

def main():
    parser = argparse.ArgumentParser(
        description='Analyze motor identification data from CSV file'
    )
    parser.add_argument('csv_file', help='Path to CSV file')
    parser.add_argument('--sample-time', type=float, default=2.0,
                       help='Sample time in milliseconds (default: 2.0)')
    parser.add_argument('--window', type=int, default=200,
                       help='Analysis window size in samples (default: 200)')
    parser.add_argument('--threshold', type=float, default=5.0,
                       help='PWM step detection threshold (default: 5.0)')
    
    args = parser.parse_args()
    
    # Check if file exists
    if not os.path.exists(args.csv_file):
        print(f"Error: File not found: {args.csv_file}")
        sys.exit(1)
    
    print("=" * 70)
    print("Motor Identification Data Viewer")
    print("=" * 70)
    print(f"\nLoading data from: {args.csv_file}")
    
    # Load data
    samples = load_csv_data(args.csv_file)
    print(f"Loaded {len(samples)} samples")
    
    if len(samples) < 10:
        print("Error: Insufficient samples for analysis")
        sys.exit(1)
    
    # Create analyzer
    analyzer = MotorIdentAnalyzer(samples, args.sample_time)
    
    # Analyze
    print(f"\nAnalyzing with:")
    print(f"  Sample time: {args.sample_time} ms")
    print(f"  Analysis window: {args.window} samples")
    print(f"  Step threshold: {args.threshold} PWM units")
    print("\n" + "-" * 70)
    
    results = analyzer.analyze_all_steps(window=args.window)
    
    if not results:
        print("\nNo steps detected in data!")
        print("\nTry adjusting parameters:")
        print("  --threshold <value>   # Lower if steps are small")
        print("  --window <samples>    # Increase if transients are long")
        sys.exit(1)
    
    # Print results
    print(f"\n{'=' * 70}")
    print(f"Analysis Results ({len(results)} steps)")
    print("=" * 70)
    
    for i, result in enumerate(results):
        print(f"\nStep {i+1} at t={result['step_time_s']:.3f}s:")
        print(f"  PWM: {result['pwm_initial']:.1f} → {result['pwm_final']:.1f} "
              f"(Δ={result['pwm_step']:.1f})")
        print(f"  Velocity: {result['velocity_initial']:.2f} → "
              f"{result['velocity_final']:.2f} cm/s "
              f"(Δ={result['velocity_step']:.2f})")
        
        if result['dead_time_s'] is not None:
            print(f"  Dead Time: {result['dead_time_s']*1000:.2f} ms")
        else:
            print(f"  Dead Time: N/A")
        
        if result['transient_time_s'] is not None:
            print(f"  Transient Time: {result['transient_time_s']*1000:.2f} ms")
        else:
            print(f"  Transient Time: N/A")
        
        if result['settling_time_s'] is not None:
            print(f"  Settling Time (2%): {result['settling_time_s']*1000:.2f} ms")
        
        if result['rise_time_s'] is not None:
            print(f"  Rise Time (10-90%): {result['rise_time_s']*1000:.2f} ms")
        
        if result['dc_gain'] is not None:
            print(f"  DC Gain: {result['dc_gain']:.4f} cm/s per PWM")
    
    # Calculate averages
    import numpy as np
    
    dead_times = [r['dead_time_s'] for r in results if r['dead_time_s'] is not None]
    transient_times = [r['transient_time_s'] for r in results if r['transient_time_s'] is not None]
    dc_gains = [r['dc_gain'] for r in results if r['dc_gain'] is not None]
    
    print(f"\n{'=' * 70}")
    print("Average Parameters")
    print("=" * 70)
    
    if dead_times:
        print(f"Dead Time: {np.mean(dead_times)*1000:.2f} ± {np.std(dead_times)*1000:.2f} ms")
    
    if transient_times:
        print(f"Transient Time: {np.mean(transient_times)*1000:.2f} ± {np.std(transient_times)*1000:.2f} ms")
    
    if dc_gains:
        print(f"DC Gain: {np.mean(dc_gains):.4f} ± {np.std(dc_gains):.4f} cm/s per PWM")
    
    # Save results
    base_path = Path(args.csv_file).stem
    output_dir = Path(args.csv_file).parent
    
    # Save analysis JSON
    analysis_path = output_dir / f"{base_path}_reanalysis.json"
    analysis_results = {
        "source_file": args.csv_file,
        "num_steps_analyzed": len(results),
        "avg_dead_time_s": float(np.mean(dead_times)) if dead_times else None,
        "std_dead_time_s": float(np.std(dead_times)) if dead_times else None,
        "avg_transient_time_s": float(np.mean(transient_times)) if transient_times else None,
        "std_transient_time_s": float(np.std(transient_times)) if transient_times else None,
        "avg_dc_gain": float(np.mean(dc_gains)) if dc_gains else None,
        "std_dc_gain": float(np.std(dc_gains)) if dc_gains else None,
        "individual_steps": results
    }
    
    with open(analysis_path, 'w') as f:
        json.dump(analysis_results, f, indent=2)
    print(f"\nAnalysis saved to: {analysis_path}")
    
    # Save plot
    plot_path = output_dir / f"{base_path}_reanalysis_plot.png"
    analyzer.plot_step_responses(results, str(plot_path))
    print(f"Plot saved to: {plot_path}")
    
    print("\n" + "=" * 70)
    print("Analysis Complete!")
    print("=" * 70)

if __name__ == "__main__":
    main()
