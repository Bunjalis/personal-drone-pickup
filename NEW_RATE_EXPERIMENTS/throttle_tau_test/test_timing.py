#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
from compute_error import compute_model_error

def test_timing_fix():
    """Quick test to check if timing alignment is working"""
    
    # Use optimized parameters from previous run
    params = np.array([38.4, 0.07, 0.07, 80.0, 250.0, 0.5, 0.57])
    print('Testing timing fix with optimized parameters...')
    
    results = compute_model_error(parameters=params)
    
    if results:
        # Focus on first 100 timesteps for easier analysis
        n_points = min(100, len(results['time_steps']))
        
        time_steps = results['time_steps'][:n_points]
        pred_vz = results['predicted_vz'][:n_points]
        meas_vz = results['measured_vz'][:n_points]
        
        print(f'\nFirst {n_points} timesteps - Z velocity comparison:')
        print('Time     Predicted    Measured     Error      Abs Error')
        print('-' * 55)
        
        total_abs_error = 0
        for i in range(min(20, n_points)):  # Show first 20 for detailed view
            error = pred_vz[i] - meas_vz[i]
            abs_error = abs(error)
            total_abs_error += abs_error
            print(f'{time_steps[i]:.3f}    {pred_vz[i]:.6f}   {meas_vz[i]:.6f}   {error:+.6f}   {abs_error:.6f}')
        
        avg_abs_error = total_abs_error / min(20, n_points)
        print(f'\nAverage absolute error (first 20): {avg_abs_error:.6f}')
        
        # Create a simple plot to visualize alignment
        plt.figure(figsize=(12, 6))
        
        plt.subplot(1, 2, 1)
        plt.plot(time_steps, pred_vz, 'r-', linewidth=2, label='Predicted', alpha=0.8)
        plt.plot(time_steps, meas_vz, 'b--', linewidth=2, label='Measured', alpha=0.8)
        plt.title('Z Velocity: Predicted vs Measured (First 100 steps)')
        plt.xlabel('Time (s)')
        plt.ylabel('Z Velocity (m/s)')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        plt.subplot(1, 2, 2)
        velocity_errors = np.array(pred_vz) - np.array(meas_vz)
        plt.plot(time_steps, velocity_errors, 'g-', linewidth=2)
        plt.title('Z Velocity Error (Predicted - Measured)')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity Error (m/s)')
        plt.grid(True, alpha=0.3)
        plt.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        plt.tight_layout()
        plt.savefig('timing_test_velocity_comparison.png', dpi=300, bbox_inches='tight')
        print(f'\nTiming test plot saved as: timing_test_velocity_comparison.png')
        
        # Check for phase shift by computing cross-correlation
        from scipy import signal
        if len(pred_vz) > 50:
            correlation = signal.correlate(pred_vz[:50], meas_vz[:50], mode='full')
            lags = signal.correlation_lags(len(pred_vz[:50]), len(meas_vz[:50]), mode='full')
            lag_at_max = lags[np.argmax(correlation)]
            print(f'Cross-correlation suggests lag of {lag_at_max} timesteps')
            if abs(lag_at_max) <= 1:
                print('✓ Timing appears to be well aligned!')
            else:
                print('✗ Significant timing mismatch detected')
        
        return results
    else:
        print('Failed to compute model errors')
        return None

if __name__ == "__main__":
    test_timing_fix()
