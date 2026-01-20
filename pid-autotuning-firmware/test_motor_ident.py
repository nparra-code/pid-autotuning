#!/usr/bin/env python3
"""
Test script for motor identification analyzer
Generates synthetic motor response data and tests the analysis
"""

import numpy as np
import sys
from motor_ident_server import MotorIdentAnalyzer

def generate_first_order_response(time, amplitude, tau, delay):
    """
    Generate first-order system step response
    
    Args:
        time: Time array
        amplitude: Step amplitude
        tau: Time constant
        delay: Dead time
    
    Returns:
        Response array
    """
    response = np.zeros_like(time)
    for i, t in enumerate(time):
        if t < delay:
            response[i] = 0
        else:
            response[i] = amplitude * (1 - np.exp(-(t - delay) / tau))
    return response

def test_motor_ident_analyzer():
    """Test the motor identification analyzer with synthetic data"""
    
    print("=" * 60)
    print("Motor Identification Analyzer Test")
    print("=" * 60)
    
    # Simulation parameters
    sample_time_ms = 2.0  # 2ms sample time
    sample_time_s = sample_time_ms / 1000.0
    duration = 10.0  # 10 seconds
    time = np.arange(0, duration, sample_time_s)
    
    # Motor parameters to simulate
    true_dead_time = 0.025  # 25ms dead time
    true_tau = 0.15  # 150ms time constant
    true_dc_gain = 0.25  # 0.25 cm/s per PWM unit
    
    print(f"\nTrue System Parameters:")
    print(f"  Dead Time: {true_dead_time*1000:.1f} ms")
    print(f"  Time Constant (tau): {true_tau*1000:.1f} ms")
    print(f"  DC Gain: {true_dc_gain:.3f} cm/s per PWM")
    
    # Generate PWM input with steps
    pwm_input = np.zeros_like(time)
    pwm_input[time >= 1.0] = 20.0   # Step to 20 at t=1s
    pwm_input[time >= 3.0] = 60.0   # Step to 60 at t=3s
    pwm_input[time >= 5.0] = 40.0   # Step to 40 at t=5s
    pwm_input[time >= 7.0] = -20.0  # Step to -20 at t=7s
    
    # Generate velocity response
    velocity = np.zeros_like(time)
    
    # Calculate response for each PWM level
    step_times = [1.0, 3.0, 5.0, 7.0]
    step_values = [20.0, 60.0, 40.0, -20.0]
    
    for i, (step_time, step_value) in enumerate(zip(step_times, step_values)):
        mask = time >= step_time
        time_from_step = time[mask] - step_time
        
        # Calculate expected velocity based on PWM
        target_velocity = step_value * true_dc_gain
        
        # Add first-order response
        if i == 0:
            velocity[mask] = generate_first_order_response(
                time_from_step, target_velocity, true_tau, true_dead_time
            )
        else:
            # Add incremental change
            prev_velocity = step_values[i-1] * true_dc_gain
            delta_velocity = target_velocity - prev_velocity
            velocity[mask] = prev_velocity + generate_first_order_response(
                time_from_step, delta_velocity, true_tau, true_dead_time
            )
    
    # Add some noise
    noise_level = 0.1  # cm/s
    velocity += np.random.normal(0, noise_level, size=velocity.shape)
    
    # Create samples in the format expected by analyzer
    timestamps_ms = (time * 1000).astype(np.uint32)
    samples = [(int(ts), float(pwm), float(vel)) 
               for ts, pwm, vel in zip(timestamps_ms, pwm_input, velocity)]
    
    print(f"\nGenerated {len(samples)} samples")
    print(f"Duration: {duration:.1f} seconds")
    print(f"Sample rate: {1.0/sample_time_s:.0f} Hz")
    
    # Create analyzer
    print("\n" + "-" * 60)
    print("Running Analysis...")
    print("-" * 60)
    
    analyzer = MotorIdentAnalyzer(samples, sample_time_ms)
    
    # Analyze all steps
    results = analyzer.analyze_all_steps(window=750)  # 1.5 second window
    
    if not results:
        print("\nERROR: No steps detected!")
        return False
    
    print(f"\n" + "=" * 60)
    print(f"Analysis Results ({len(results)} steps)")
    print("=" * 60)
    
    # Print individual results
    for i, result in enumerate(results):
        print(f"\nStep {i+1} at t={result['step_time_s']:.3f}s:")
        print(f"  PWM: {result['pwm_initial']:.1f} → {result['pwm_final']:.1f} (Δ={result['pwm_step']:.1f})")
        print(f"  Velocity: {result['velocity_initial']:.2f} → {result['velocity_final']:.2f} cm/s")
        
        if result['dead_time_s'] is not None:
            print(f"  Dead Time: {result['dead_time_s']*1000:.2f} ms")
            error_pct = abs(result['dead_time_s'] - true_dead_time) / true_dead_time * 100
            print(f"    Error: {error_pct:.1f}%")
        
        if result['transient_time_s'] is not None:
            # Transient time should be approximately 3-4 time constants
            expected_transient = 3 * true_tau
            print(f"  Transient Time: {result['transient_time_s']*1000:.2f} ms")
            error_pct = abs(result['transient_time_s'] - expected_transient) / expected_transient * 100
            print(f"    Error: {error_pct:.1f}%")
        
        if result['rise_time_s'] is not None:
            print(f"  Rise Time: {result['rise_time_s']*1000:.2f} ms")
        
        if result['dc_gain'] is not None:
            print(f"  DC Gain: {result['dc_gain']:.4f} cm/s per PWM")
            error_pct = abs(result['dc_gain'] - true_dc_gain) / true_dc_gain * 100
            print(f"    Error: {error_pct:.1f}%")
    
    # Calculate averages
    dead_times = [r['dead_time_s'] for r in results if r['dead_time_s'] is not None]
    transient_times = [r['transient_time_s'] for r in results if r['transient_time_s'] is not None]
    dc_gains = [r['dc_gain'] for r in results if r['dc_gain'] is not None]
    
    print(f"\n" + "=" * 60)
    print("Average Parameters")
    print("=" * 60)
    
    if dead_times:
        avg_dead = np.mean(dead_times)
        std_dead = np.std(dead_times)
        error_pct = abs(avg_dead - true_dead_time) / true_dead_time * 100
        print(f"Dead Time: {avg_dead*1000:.2f} ± {std_dead*1000:.2f} ms (Error: {error_pct:.1f}%)")
    
    if transient_times:
        avg_trans = np.mean(transient_times)
        std_trans = np.std(transient_times)
        expected_transient = 3 * true_tau
        error_pct = abs(avg_trans - expected_transient) / expected_transient * 100
        print(f"Transient Time: {avg_trans*1000:.2f} ± {std_trans*1000:.2f} ms (Error: {error_pct:.1f}%)")
    
    if dc_gains:
        avg_gain = np.mean(dc_gains)
        std_gain = np.std(dc_gains)
        error_pct = abs(avg_gain - true_dc_gain) / true_dc_gain * 100
        print(f"DC Gain: {avg_gain:.4f} ± {std_gain:.4f} cm/s per PWM (Error: {error_pct:.1f}%)")
    
    # Generate plot
    print(f"\n" + "-" * 60)
    print("Generating plot...")
    plot_path = "test_motor_ident_plot.png"
    analyzer.plot_step_responses(results, plot_path)
    print(f"Plot saved to: {plot_path}")
    
    # Success criteria
    print(f"\n" + "=" * 60)
    print("Test Summary")
    print("=" * 60)
    
    success = True
    
    if len(results) >= 3:
        print("✓ Detected multiple steps")
    else:
        print("✗ Failed to detect enough steps")
        success = False
    
    if dead_times and abs(np.mean(dead_times) - true_dead_time) / true_dead_time < 0.20:
        print("✓ Dead time within 20% of true value")
    else:
        print("✗ Dead time estimation error too large")
        success = False
    
    if dc_gains and abs(np.mean(dc_gains) - true_dc_gain) / true_dc_gain < 0.20:
        print("✓ DC gain within 20% of true value")
    else:
        print("✗ DC gain estimation error too large")
        success = False
    
    if success:
        print(f"\n{'='*60}")
        print("TEST PASSED ✓")
        print("=" * 60)
    else:
        print(f"\n{'='*60}")
        print("TEST FAILED ✗")
        print("=" * 60)
    
    return success


if __name__ == "__main__":
    success = test_motor_ident_analyzer()
    sys.exit(0 if success else 1)
