#!/usr/bin/env python3
"""
PID Autotuner Server
Receives robot telemetry data, runs RNN inference, returns PID constants
"""

import socket
import struct
import numpy as np
import logging
from typing import Tuple, Optional
import zlib

# Configure logging
logging.basicConfig(level=logging.INFO, 
                   format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Protocol constants
MSG_TYPE_DATA_BATCH = 0x01
MSG_TYPE_PID_RESPONSE = 0x02
MSG_TYPE_ACK = 0x03
MSG_TYPE_ERROR = 0x04
PROTOCOL_VERSION = 1
PORT = 8888

# Data structures matching ESP32 side
ROBOT_SAMPLE_STRUCT = struct.Struct('<I12f')  # timestamp + 12 floats
# I=uint32, f=float (timestamp, 3 motor speeds, 3 currents, 3 targets, 
#                     pos_x, pos_y, heading)

HEADER_STRUCT = struct.Struct('<BBHI')  # type, version, length, checksum (unsigned)
PID_RESPONSE_STRUCT = struct.Struct('<9fBB')  # 9 floats (3x3 PID) + 2 bytes


class PIDAutotuner:
    """Mock RNN-based PID autotuner - replace with your actual model"""
    
    def __init__(self):
        self.iteration = 0
        self.converged = False
        logger.info("PID Autotuner initialized")
        
        # TODO: Load your trained RNN model here
        # self.model = load_model('pid_autotuner_rnn.h5')
    
    def predict_pid(self, samples: np.ndarray) -> Tuple[np.ndarray, bool]:
        """
        Run RNN inference to predict PID constants
        
        Args:
            samples: Robot samples array shape (n_samples, n_features)
        
        Returns:
            pid_constants: Array of shape (3, 3) for [Kp, Ki, Kd] x 3 motors
            converged: Whether tuning has converged
        """
        self.iteration += 1
        
        # Extract features for RNN (customize based on your model)
        # For example: error signals, speed tracking, oscillations, etc.
        
        # MOCK IMPLEMENTATION - Replace with actual RNN inference
        logger.info(f"Running RNN inference on {len(samples)} samples")
        
        # Simulate learning: gradually adjust PID values
        base_kp = 1.0 + self.iteration * 0.1
        base_ki = 0.1 + self.iteration * 0.01
        base_kd = 0.01 + self.iteration * 0.005
        
        # Add some variation per motor
        pid_constants = np.array([
            [base_kp, base_kp * 0.95, base_kp * 1.05],
            [base_ki, base_ki * 0.98, base_ki * 1.02],
            [base_kd, base_kd * 0.97, base_kd * 1.03]
        ])
        
        # Mock convergence after 10 iterations
        converged = self.iteration >= 10
        
        # TODO: Replace above with actual model inference:
        # features = self.preprocess(samples)
        # pid_constants = self.model.predict(features)
        # converged = self.check_convergence(pid_constants, samples)
        
        logger.info(f"Iteration {self.iteration}: Kp={pid_constants[0]}, "
                   f"Ki={pid_constants[1]}, Kd={pid_constants[2]}")
        
        return pid_constants, converged


class TelemetryServer:
    """Server to handle robot telemetry and PID autotuning"""
    
    def __init__(self, port: int = PORT):
        self.port = port
        self.autotuner = PIDAutotuner()
        self.server_socket = None
    
    def calculate_crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum"""
        return zlib.crc32(data) & 0xFFFFFFFF
    
    def parse_header(self, data: bytes) -> Tuple[int, int, int, int]:
        """Parse message header"""
        msg_type, version, length, checksum = HEADER_STRUCT.unpack(data)
        return msg_type, version, length, checksum
    
    def parse_data_batch(self, data: bytes) -> np.ndarray:
        """Parse data batch into numpy array"""
        # First 4 bytes: sample_count (uint16) + sequence_number (uint16)
        sample_count, seq_num = struct.unpack('<HH', data[:4])
        logger.info(f"Received batch: {sample_count} samples, seq={seq_num}")
        
        # Parse samples
        offset = 4
        samples = []
        
        for i in range(sample_count):
            sample_data = data[offset:offset + ROBOT_SAMPLE_STRUCT.size]
            sample = ROBOT_SAMPLE_STRUCT.unpack(sample_data)
            samples.append(sample)
            offset += ROBOT_SAMPLE_STRUCT.size
        
        # Convert to numpy array
        samples_array = np.array(samples)
        logger.info(f"Parsed samples shape: {samples_array.shape}")
        
        return samples_array
    
    def create_pid_response(self, pid_constants: np.ndarray, 
                           iteration: int, converged: bool) -> bytes:
        """Create PID response message"""
        # Flatten PID constants (Kp0, Kp1, Kp2, Ki0, Ki1, Ki2, Kd0, Kd1, Kd2)
        pid_flat = pid_constants.T.flatten()
        
        # Pack payload
        payload = PID_RESPONSE_STRUCT.pack(
            *pid_flat, iteration, 1 if converged else 0
        )
        
        # Calculate checksum
        checksum = self.calculate_crc32(payload)
        
        # Create header
        header = HEADER_STRUCT.pack(
            MSG_TYPE_PID_RESPONSE,
            PROTOCOL_VERSION,
            len(payload),
            checksum
        )
        
        return header + payload
    
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
                    logger.warning(f"Incomplete header received: {len(header_data)} bytes, expected {HEADER_STRUCT.size}")
                    logger.debug(f"Header data: {header_data.hex()}")
                    break

                msg_type, version, length, checksum = self.parse_header(header_data)
                logger.info(f"Header: type={msg_type}, version={version}, "
                           f"length={length}, checksum={checksum:08X}")
                
                # Validate message type
                if msg_type != MSG_TYPE_DATA_BATCH:
                    logger.warning(f"Unexpected message type: {msg_type} (expected {MSG_TYPE_DATA_BATCH})")
                    logger.debug(f"Raw header: {header_data.hex()}")
                    # Try to recover by reading and discarding payload if length is reasonable
                    if 0 < length < 65536:
                        discarded = client_socket.recv(length)
                        logger.debug(f"Discarded {len(discarded)} bytes of unexpected payload")
                    continue
                
                # Validate protocol version
                if version != PROTOCOL_VERSION:
                    logger.warning(f"Protocol version mismatch: {version} (expected {PROTOCOL_VERSION})")
                
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
                    logger.error(f"Checksum mismatch: calculated={calc_checksum:08X}, received={checksum:08X}")
                    continue
                
                # Parse data
                samples = self.parse_data_batch(payload)
                
                # Run RNN inference
                pid_constants, converged = self.autotuner.predict_pid(samples)
                
                # Send response
                response = self.create_pid_response(
                    pid_constants, 
                    self.autotuner.iteration, 
                    converged
                )
                client_socket.sendall(response)
                logger.info(f"Sent PID response: iteration={self.autotuner.iteration}, "
                           f"converged={converged}")
                
                if converged:
                    logger.info("Autotuning converged! Final PID constants sent.")
                    # Optionally close connection or continue for monitoring
        
        except Exception as e:
            logger.error(f"Error handling client: {e}")
        finally:
            client_socket.close()
            logger.info(f"Connection closed: {address}")
    
    def start(self):
        """Start the server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        
        logger.info(f"Server listening on port {self.port}")
        
        try:
            while True:
                client_socket, address = self.server_socket.accept()
                logger.info(f"New connection from {address}")
                
                # Handle client (single-threaded for simplicity)
                # For multiple robots, use threading.Thread
                self.handle_client(client_socket, address)
        
        except KeyboardInterrupt:
            logger.info("Server shutting down...")
        finally:
            if self.server_socket:
                self.server_socket.close()


if __name__ == "__main__":
    server = TelemetryServer()
    server.start()