#!/usr/bin/env python3
"""
Motor Identification Server
Receives PWM input and motor velocity response data for system identification
Analyzes dead time, transient time, and step response characteristics
"""

import socket
import struct
import logging
from typing import Tuple, Optional, Dict, List
import zlib
from datetime import datetime
import json
import csv
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal
from pathlib import Path

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Protocol constants
MSG_TYPE_IDENT_DATA = 0x05  # New message type for identification data
MSG_TYPE_ACK = 0x03
PROTOCOL_VERSION = 1
PORT = 8889  # Different port from data logger

# Data structure matching ESP32 side for motor identification
# timestamp_ms (uint32) + pwm_input (float) + velocity_response (float)
IDENT_SAMPLE_STRUCT = struct.Struct('<Iff')  # I=uint32, f=float, f=float
HEADER_STRUCT = struct.Struct('<BBHI')  # type, version, length, checksum (unsigned)
BATCH_HEADER_STRUCT = struct.Struct('<HH')  # sample_count, sequence_number

# Analysis configuration
LOG_DIR = "motor_ident_logs"
CSV_HEADER = ["timestamp_ms", "pwm_input", "velocity_response"]

# System identification parameters
SETTLING_THRESHOLD = 0.02  # 2% of final value
RISE_TIME_LOW = 0.10      # 10% of final value
RISE_TIME_HIGH = 0.90     # 90% of final value


class MotorIdentAnalyzer:
    """Analyzes motor identification data to extract system parameters"""
    
    def __init__(self, samples: List[Tuple], sample_time_ms: float = 2.0):
        """
        Initialize analyzer with sample data
        
        Args:
            samples: List of (timestamp_ms, pwm_input, velocity_response) tuples
            sample_time_ms: Sample time in milliseconds
        """
        self.samples = samples
        self.sample_time_ms = sample_time_ms
        self.sample_time_s = sample_time_ms / 1000.0
        
        # Extract time series
        self.timestamps = np.array([s[0] for s in samples])
        self.pwm_inputs = np.array([s[1] for s in samples])
        self.velocities = np.array([s[2] for s in samples])
        
        # Convert timestamps to seconds from start
        self.time_s = (self.timestamps - self.timestamps[0]) / 1000.0
        
    def detect_step_changes(self, threshold: float = 5.0) -> List[int]:
        """
        Detect step changes in PWM input
        
        Args:
            threshold: Minimum change to consider as a step
            
        Returns:
            List of indices where steps occur
        """
        pwm_diff = np.abs(np.diff(self.pwm_inputs))
        step_indices = np.where(pwm_diff > threshold)[0] + 1
        return step_indices.tolist()
    
    def analyze_step_response(self, step_idx: int, window: int = 200) -> Dict:
        """
        Analyze a single step response to extract system parameters
        
        Args:
            step_idx: Index where step occurs
            window: Number of samples to analyze after step
            
        Returns:
            Dictionary containing identified parameters
        """
        # Ensure we have enough data
        end_idx = min(step_idx + window, len(self.pwm_inputs))
        if end_idx - step_idx < 20:
            logger.warning(f"Insufficient data for step at index {step_idx}")
            return {}
        
        # Extract step response window
        time_window = self.time_s[step_idx:end_idx] - self.time_s[step_idx]
        pwm_window = self.pwm_inputs[step_idx:end_idx]
        vel_window = self.velocities[step_idx:end_idx]
        
        # Get initial and final values
        pwm_initial = self.pwm_inputs[step_idx - 1] if step_idx > 0 else 0
        pwm_final = pwm_window[0]
        pwm_step = pwm_final - pwm_initial
        
        # Get velocity initial value (average before step)
        pre_step_samples = 5
        if step_idx >= pre_step_samples:
            vel_initial = np.mean(self.velocities[step_idx - pre_step_samples:step_idx])
        else:
            vel_initial = self.velocities[step_idx - 1] if step_idx > 0 else 0
        
        # Detect dead time (when velocity starts to change)
        # Dead time is when velocity changes more than noise threshold
        vel_noise_threshold = 0.5  # cm/s
        vel_diff = np.abs(vel_window - vel_initial)
        dead_time_samples = np.where(vel_diff > vel_noise_threshold)[0]
        
        if len(dead_time_samples) == 0:
            logger.warning(f"No velocity response detected for step at index {step_idx}")
            dead_time = None
            transient_time = None
            steady_state_velocity = vel_initial
            settling_time = None
            rise_time = None
        else:
            dead_time_idx = dead_time_samples[0]
            dead_time = time_window[dead_time_idx]
            
            # Find steady state (last 20% of window)
            steady_start = int(len(vel_window) * 0.8)
            steady_state_velocity = np.mean(vel_window[steady_start:])
            
            # Calculate velocity change
            vel_step = steady_state_velocity - vel_initial
            
            if abs(vel_step) < vel_noise_threshold:
                logger.warning(f"Velocity change too small for step at index {step_idx}")
                transient_time = None
                settling_time = None
                rise_time = None
            else:
                # Settling time (when within 2% of steady state)
                settling_threshold = abs(vel_step) * SETTLING_THRESHOLD
                within_settling = np.abs(vel_window - steady_state_velocity) < settling_threshold
                settling_indices = np.where(within_settling)[0]
                
                if len(settling_indices) > 0 and settling_indices[0] > dead_time_idx:
                    settling_idx = settling_indices[0]
                    settling_time = time_window[settling_idx] - dead_time
                    transient_time = settling_time
                else:
                    settling_time = None
                    transient_time = time_window[-1] - dead_time
                
                # Rise time (10% to 90% of final value)
                rise_low = vel_initial + abs(vel_step) * RISE_TIME_LOW
                rise_high = vel_initial + abs(vel_step) * RISE_TIME_HIGH
                
                # For positive step
                if vel_step > 0:
                    low_idx = np.where(vel_window >= rise_low)[0]
                    high_idx = np.where(vel_window >= rise_high)[0]
                else:  # For negative step
                    low_idx = np.where(vel_window <= rise_low)[0]
                    high_idx = np.where(vel_window <= rise_high)[0]
                
                if len(low_idx) > 0 and len(high_idx) > 0:
                    rise_time = time_window[high_idx[0]] - time_window[low_idx[0]]
                else:
                    rise_time = None
        
        # Calculate DC gain (steady state velocity / PWM input)
        if pwm_step != 0:
            dc_gain = (steady_state_velocity - vel_initial) / pwm_step
        else:
            dc_gain = None
        
        # Package results
        results = {
            'step_index': step_idx,
            'step_time_s': self.time_s[step_idx],
            'pwm_initial': pwm_initial,
            'pwm_final': pwm_final,
            'pwm_step': pwm_step,
            'velocity_initial': vel_initial,
            'velocity_final': steady_state_velocity,
            'velocity_step': steady_state_velocity - vel_initial,
            'dead_time_s': dead_time,
            'transient_time_s': transient_time,
            'settling_time_s': settling_time,
            'rise_time_s': rise_time,
            'dc_gain': dc_gain,
            'time_window': time_window.tolist(),
            'vel_window': vel_window.tolist(),
            'pwm_window': pwm_window.tolist()
        }
        
        return results
    
    def analyze_all_steps(self, window: int = 200) -> List[Dict]:
        """
        Analyze all step responses in the data
        
        Args:
            window: Number of samples to analyze after each step
            
        Returns:
            List of analysis results for each step
        """
        step_indices = self.detect_step_changes()
        logger.info(f"Detected {len(step_indices)} step changes at indices: {step_indices}")
        
        results = []
        for idx in step_indices:
            result = self.analyze_step_response(idx, window)
            if result:
                results.append(result)
                logger.info(f"Step at {result['step_time_s']:.3f}s: "
                          f"Dead time={result['dead_time_s']:.4f}s, "
                          f"Transient time={result['transient_time_s']:.4f}s")
        
        return results
    
    def plot_step_responses(self, results: List[Dict], save_path: str):
        """
        Plot step responses with identified parameters
        
        Args:
            results: List of analysis results
            save_path: Path to save the plot
        """
        n_steps = len(results)
        if n_steps == 0:
            logger.warning("No steps to plot")
            return
        
        fig, axes = plt.subplots(n_steps, 2, figsize=(14, 4 * n_steps))
        if n_steps == 1:
            axes = axes.reshape(1, -1)
        
        for i, result in enumerate(results):
            time_window = np.array(result['time_window'])
            vel_window = np.array(result['vel_window'])
            pwm_window = np.array(result['pwm_window'])
            
            # Plot PWM input
            axes[i, 0].plot(time_window, pwm_window, 'b-', linewidth=2)
            axes[i, 0].axhline(y=result['pwm_initial'], color='g', linestyle='--', 
                             label=f"Initial: {result['pwm_initial']:.1f}")
            axes[i, 0].axhline(y=result['pwm_final'], color='r', linestyle='--',
                             label=f"Final: {result['pwm_final']:.1f}")
            axes[i, 0].set_xlabel('Time (s)')
            axes[i, 0].set_ylabel('PWM Input')
            axes[i, 0].set_title(f"Step {i+1}: PWM Input")
            axes[i, 0].legend()
            axes[i, 0].grid(True)
            
            # Plot velocity response
            axes[i, 1].plot(time_window, vel_window, 'b-', linewidth=2, label='Velocity')
            axes[i, 1].axhline(y=result['velocity_initial'], color='g', linestyle='--',
                             label=f"Initial: {result['velocity_initial']:.2f} cm/s")
            axes[i, 1].axhline(y=result['velocity_final'], color='r', linestyle='--',
                             label=f"Final: {result['velocity_final']:.2f} cm/s")
            
            # Mark dead time
            if result['dead_time_s'] is not None:
                axes[i, 1].axvline(x=result['dead_time_s'], color='orange', linestyle=':',
                                 label=f"Dead time: {result['dead_time_s']:.4f}s")
            
            # Mark settling time
            if result['transient_time_s'] is not None:
                settling_time_abs = result['dead_time_s'] + result['transient_time_s']
                axes[i, 1].axvline(x=settling_time_abs, color='purple', linestyle=':',
                                 label=f"Settling: {result['transient_time_s']:.4f}s")
            
            axes[i, 1].set_xlabel('Time (s)')
            axes[i, 1].set_ylabel('Velocity (cm/s)')
            axes[i, 1].set_title(f"Step {i+1}: Velocity Response")
            axes[i, 1].legend()
            axes[i, 1].grid(True)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        logger.info(f"Step response plot saved to: {save_path}")
        plt.close()


class MotorIdentLogger:
    """Handles logging of motor identification data to files"""
    
    def __init__(self, log_dir: str = LOG_DIR):
        self.log_dir = log_dir
        self.session_start = datetime.now()
        self.session_id = self.session_start.strftime("%Y%m%d_%H%M%S")
        
        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # File paths
        self.csv_path = os.path.join(self.log_dir, f"motor_ident_{self.session_id}.csv")
        self.raw_path = os.path.join(self.log_dir, f"motor_ident_{self.session_id}_raw.bin")
        self.meta_path = os.path.join(self.log_dir, f"motor_ident_{self.session_id}_meta.json")
        self.analysis_path = os.path.join(self.log_dir, f"motor_ident_{self.session_id}_analysis.json")
        self.plot_path = os.path.join(self.log_dir, f"motor_ident_{self.session_id}_plot.png")
        
        # Statistics
        self.total_samples = 0
        self.samples_list = []  # Store samples for analysis
        
        # Initialize CSV file
        self._init_csv()
        
        # Initialize metadata
        self._init_metadata()
        
        logger.info(f"Motor identification logger initialized. Session ID: {self.session_id}")
        logger.info(f"CSV log: {self.csv_path}")
        logger.info(f"Raw log: {self.raw_path}")
    
    def _init_csv(self):
        """Initialize CSV file with header"""
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(CSV_HEADER)
        logger.info(f"Created CSV file: {self.csv_path}")
    
    def _init_metadata(self):
        """Initialize metadata file"""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start.isoformat(),
            "protocol_version": PROTOCOL_VERSION,
            "total_samples": 0,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path),
            "analysis_file": os.path.basename(self.analysis_path),
            "plot_file": os.path.basename(self.plot_path)
        }
        self._write_metadata(metadata)
    
    def _write_metadata(self, metadata: dict):
        """Write metadata to JSON file"""
        with open(self.meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def log_samples(self, samples: list):
        """
        Log samples to CSV and raw binary files
        
        Args:
            samples: List of tuples containing (timestamp_ms, pwm_input, velocity_response)
        """
        # Write to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for sample in samples:
                writer.writerow(sample)
        
        # Write to raw binary
        with open(self.raw_path, 'ab') as f:
            for sample in samples:
                packed = IDENT_SAMPLE_STRUCT.pack(*sample)
                f.write(packed)
        
        # Store for later analysis
        self.samples_list.extend(samples)
        
        # Update statistics
        self.total_samples += len(samples)
        
        logger.info(f"Logged {len(samples)} samples (total: {self.total_samples} samples)")
    
    def analyze_and_finalize(self, sample_time_ms: float = 2.0):
        """
        Perform system identification analysis and finalize session
        
        Args:
            sample_time_ms: Sample time in milliseconds
        """
        logger.info("Starting motor identification analysis...")
        
        if len(self.samples_list) < 10:
            logger.warning("Insufficient samples for analysis")
            self.finalize()
            return
        
        # Create analyzer
        analyzer = MotorIdentAnalyzer(self.samples_list, sample_time_ms)
        
        # Analyze all steps
        step_results = analyzer.analyze_all_steps(window=200)
        
        # Calculate average parameters
        if step_results:
            dead_times = [r['dead_time_s'] for r in step_results if r['dead_time_s'] is not None]
            transient_times = [r['transient_time_s'] for r in step_results if r['transient_time_s'] is not None]
            dc_gains = [r['dc_gain'] for r in step_results if r['dc_gain'] is not None]
            
            avg_results = {
                "num_steps_analyzed": len(step_results),
                "avg_dead_time_s": float(np.mean(dead_times)) if dead_times else None,
                "std_dead_time_s": float(np.std(dead_times)) if dead_times else None,
                "avg_transient_time_s": float(np.mean(transient_times)) if transient_times else None,
                "std_transient_time_s": float(np.std(transient_times)) if transient_times else None,
                "avg_dc_gain": float(np.mean(dc_gains)) if dc_gains else None,
                "std_dc_gain": float(np.std(dc_gains)) if dc_gains else None,
                "individual_steps": step_results
            }
            
            logger.info(f"Analysis complete: {len(step_results)} steps analyzed")
            logger.info(f"Avg dead time: {avg_results['avg_dead_time_s']:.4f}s ± {avg_results['std_dead_time_s']:.4f}s")
            logger.info(f"Avg transient time: {avg_results['avg_transient_time_s']:.4f}s ± {avg_results['std_transient_time_s']:.4f}s")
            logger.info(f"Avg DC gain: {avg_results['avg_dc_gain']:.4f} ± {avg_results['std_dc_gain']:.4f}")
            
            # Save analysis results
            with open(self.analysis_path, 'w') as f:
                json.dump(avg_results, f, indent=2)
            logger.info(f"Analysis results saved to: {self.analysis_path}")
            
            # Plot step responses
            analyzer.plot_step_responses(step_results, self.plot_path)
        
        # Finalize
        self.finalize()
    
    def finalize(self):
        """Finalize logging session and update metadata"""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start.isoformat(),
            "end_time": datetime.now().isoformat(),
            "duration_seconds": (datetime.now() - self.session_start).total_seconds(),
            "protocol_version": PROTOCOL_VERSION,
            "total_samples": self.total_samples,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path),
            "analysis_file": os.path.basename(self.analysis_path),
            "plot_file": os.path.basename(self.plot_path)
        }
        self._write_metadata(metadata)
        logger.info(f"Session finalized: {self.total_samples} samples")


class MotorIdentServer:
    """Server to receive and analyze motor identification data"""
    
    def __init__(self, port: int = PORT):
        self.port = port
        self.logger = MotorIdentLogger()
        self.server_socket = None
    
    def calculate_crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum"""
        return zlib.crc32(data) & 0xFFFFFFFF
    
    def parse_header(self, data: bytes) -> Tuple[int, int, int, int]:
        """Parse message header"""
        msg_type, version, length, checksum = HEADER_STRUCT.unpack(data)
        return msg_type, version, length, checksum
    
    def parse_ident_data(self, data: bytes) -> list:
        """
        Parse identification data batch into list of samples
        
        Returns:
            samples: List of tuples (timestamp_ms, pwm_input, velocity_response)
        """
        samples = []
        
        # First, parse the batch header
        if len(data) < BATCH_HEADER_STRUCT.size:
            logger.error(f"Payload too small for batch header: {len(data)} bytes")
            return samples
        
        sample_count, sequence_number = BATCH_HEADER_STRUCT.unpack(
            data[:BATCH_HEADER_STRUCT.size]
        )
        
        logger.info(f"Batch header: sample_count={sample_count}, sequence={sequence_number}")
        
        # Now parse the samples (skip the 4-byte header)
        offset = BATCH_HEADER_STRUCT.size
        samples_parsed = 0
        
        while offset + IDENT_SAMPLE_STRUCT.size <= len(data) and samples_parsed < sample_count:
            sample_data = data[offset:offset + IDENT_SAMPLE_STRUCT.size]
            sample = IDENT_SAMPLE_STRUCT.unpack(sample_data)
            
            # Validate sample data
            timestamp_ms, pwm_input, velocity_response = sample
            
            # Basic sanity checks
            if timestamp_ms > 1e10 or abs(pwm_input) > 200 or abs(velocity_response) > 1000:
                logger.warning(f"Suspicious sample at offset {offset}: "
                            f"ts={timestamp_ms}, pwm={pwm_input}, vel={velocity_response}")
            
            samples.append(sample)
            offset += IDENT_SAMPLE_STRUCT.size
            samples_parsed += 1
        
        if samples_parsed != sample_count:
            logger.warning(f"Expected {sample_count} samples but parsed {samples_parsed}")
        
        logger.debug(f"Parsed {len(samples)} samples from batch")
        return samples
    
    def send_ack(self, client_socket: socket.socket):
        """Send acknowledgment to client"""
        ack_payload = b''
        checksum = self.calculate_crc32(ack_payload)
        
        header = HEADER_STRUCT.pack(
            MSG_TYPE_ACK,
            PROTOCOL_VERSION,
            len(ack_payload),
            checksum
        )
        
        client_socket.sendall(header + ack_payload)
        logger.debug("Sent ACK")
    
    def handle_client(self, client_socket: socket.socket, address: str):
        """Handle a single client connection"""
        logger.info(f"Client connected from {address}")
        
        try:
            while True:
                # Receive header
                header_data = client_socket.recv(HEADER_STRUCT.size)
                
                if not header_data:
                    logger.info("Client disconnected (no data)")
                    break
                    
                if len(header_data) < HEADER_STRUCT.size:
                    logger.warning(f"Incomplete header received")
                    break

                msg_type, version, length, checksum = self.parse_header(header_data)
                logger.info(f"Header: type={msg_type}, version={version}, "
                           f"length={length}, checksum={checksum:08X}")
                
                # Validate message type
                if msg_type != MSG_TYPE_IDENT_DATA:
                    logger.warning(f"Unexpected message type: {msg_type}")
                    if 0 < length < 65536:
                        client_socket.recv(length)
                    continue
                
                # Receive payload
                payload = b''
                while len(payload) < length:
                    chunk = client_socket.recv(min(4096, length - len(payload)))
                    if not chunk:
                        raise ConnectionError("Connection lost during payload receive")
                    payload += chunk
                
                logger.debug(f"Received {len(payload)} bytes payload")
                
                # Verify checksum
                calc_checksum = self.calculate_crc32(payload)
                if calc_checksum != checksum:
                    logger.error(f"Checksum mismatch")
                    continue
                
                # Parse data
                samples = self.parse_ident_data(payload)
                logger.info(f"Parsed {len(samples)} identification samples")
                
                # Log data to files
                self.logger.log_samples(samples)
                
                # Send acknowledgment
                self.send_ack(client_socket)
        
        except Exception as e:
            logger.error(f"Error handling client: {e}", exc_info=True)
        finally:
            client_socket.close()
            logger.info(f"Connection closed: {address}")
            self.logger.analyze_and_finalize(sample_time_ms=2.0)
    
    def start(self):
        """Start the server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        
        logger.info(f"Motor Identification Server listening on port {self.port}")
        logger.info(f"Logs will be saved to: {self.logger.log_dir}")
        
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                logger.info(f"New connection from {address}")
                
                # Handle client (single-threaded for simplicity)
                self.handle_client(client_socket, address)
        
        except KeyboardInterrupt:
            logger.info("Server shutting down...")
        finally:
            if self.server_socket:
                self.server_socket.close()
            self.logger.finalize()


if __name__ == "__main__":
    server = MotorIdentServer()
    server.start()
