#!/usr/bin/env python3
"""
Data Logger Server
Receives robot telemetry data and logs it to local files
"""

import socket
import struct
import logging
from typing import Tuple, Optional
import zlib
from datetime import datetime
import json
import csv
import os

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
ROBOT_SAMPLE_STRUCT = struct.Struct('<I15f')  # timestamp + 15 floats
# I=uint32, f=float
# timestamp_ms (uint32)
# motor_state[3] (3 floats) - current motor states
# motor_setpoint[3] (3 floats) - motor setpoints/targets
# errors[3][3] (9 floats) - error history for each motor [motor_idx][time_k]
#   where time_k: 0=current(k), 1=previous(k-1), 2=two_steps_back(k-2)

HEADER_STRUCT = struct.Struct('<BBHI')  # type, version, length, checksum (unsigned)

# Data logging configuration
LOG_DIR = "telemetry_logs"
CSV_HEADER = [
    "timestamp_ms",
    "motor_state_0", "motor_state_1", "motor_state_2",
    "motor_setpoint_0", "motor_setpoint_1", "motor_setpoint_2",
    "error_0_k", "error_0_k1", "error_0_k2",
    "error_1_k", "error_1_k1", "error_1_k2",
    "error_2_k", "error_2_k1", "error_2_k2"
]

#define PID_KP_R 3.2 // Proportional gain for right motor
#define PID_KI_R 0.001 // Integral gain for right motor
#define PID_KD_R 0.002 // Derivative gain for right motor

#define PID_KP_L 3.1 // Proportional gain for left motor
#define PID_KI_L 0.002 // Integral gain for left motor
#define PID_KD_L 0.003 // Derivative gain for left motor

#define PID_KP_B 2.8 // Proportional gain for back motor
#define PID_KI_B 0.001 // Integral gain for back motor
#define PID_KD_B 0.004 // Derivative gain for back motor

PID_KP_R = 3.2
PID_KI_R = 0.001
PID_KD_R = 0.002
PID_KP_L = 3.1
PID_KI_L = 0.002
PID_KD_L = 0.003
PID_KP_B = 2.8
PID_KI_B = 0.001
PID_KD_B = 0.004

Kps = [PID_KP_R, PID_KP_L, PID_KP_B]
Kis = [PID_KI_R, PID_KI_L, PID_KI_B]
Kds = [PID_KD_R, PID_KD_L, PID_KD_B]


class DataLogger:
    """Handles logging of telemetry data to files"""
    
    def __init__(self, log_dir: str = LOG_DIR):
        self.log_dir = log_dir
        self.session_start = datetime.now()
        self.session_id = self.session_start.strftime("%Y%m%d_%H%M%S")
        
        # Create log directory if it doesn't exist
        os.makedirs(self.log_dir, exist_ok=True)
        
        # File paths
        self.csv_path = os.path.join(self.log_dir, f"telemetry_{self.session_id}_{Kps}{Kis}{Kds}.csv")
        self.raw_path = os.path.join(self.log_dir, f"telemetry_{self.session_id}_{Kps}{Kis}{Kds}_raw.bin")
        self.meta_path = os.path.join(self.log_dir, f"telemetry_{self.session_id}_{Kps}{Kis}{Kds}_meta.json")
        
        # Statistics
        self.total_samples = 0
        self.total_batches = 0
        
        # Initialize CSV file
        self._init_csv()
        
        # Initialize metadata
        self._init_metadata()
        
        logger.info(f"Data logger initialized. Session ID: {self.session_id}")
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
            "total_batches": 0,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path)
        }
        self._write_metadata(metadata)
    
    def _write_metadata(self, metadata: dict):
        """Write metadata to JSON file"""
        with open(self.meta_path, 'w') as f:
            json.dump(metadata, f, indent=2)
    
    def log_samples(self, samples: list, sequence_number: int):
        """
        Log samples to CSV and raw binary files
        
        Args:
            samples: List of tuples containing sample data
            sequence_number: Batch sequence number
        """
        # Write to CSV
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for sample in samples:
                writer.writerow(sample)
        
        # Write to raw binary (for exact reproduction)
        with open(self.raw_path, 'ab') as f:
            for sample in samples:
                # Pack sample data
                packed = ROBOT_SAMPLE_STRUCT.pack(*sample)
                f.write(packed)
        
        # Update statistics
        self.total_samples += len(samples)
        self.total_batches += 1
        
        logger.info(f"Logged batch #{sequence_number}: {len(samples)} samples "
                   f"(total: {self.total_samples} samples in {self.total_batches} batches)")
    
    def finalize(self):
        """Finalize logging session and update metadata"""
        metadata = {
            "session_id": self.session_id,
            "start_time": self.session_start.isoformat(),
            "end_time": datetime.now().isoformat(),
            "duration_seconds": (datetime.now() - self.session_start).total_seconds(),
            "protocol_version": PROTOCOL_VERSION,
            "total_samples": self.total_samples,
            "total_batches": self.total_batches,
            "csv_file": os.path.basename(self.csv_path),
            "raw_file": os.path.basename(self.raw_path)
        }
        self._write_metadata(metadata)
        logger.info(f"Session finalized: {self.total_samples} samples, {self.total_batches} batches")


class DataLoggerServer:
    """Server to receive and log robot telemetry data"""
    
    def __init__(self, port: int = PORT):
        self.port = port
        self.data_logger = DataLogger()
        self.server_socket = None
    
    def calculate_crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum"""
        return zlib.crc32(data) & 0xFFFFFFFF
    
    def parse_header(self, data: bytes) -> Tuple[int, int, int, int]:
        """Parse message header"""
        msg_type, version, length, checksum = HEADER_STRUCT.unpack(data)
        return msg_type, version, length, checksum
    
    def parse_data_batch(self, data: bytes) -> Tuple[list, int]:
        """
        Parse data batch into list of samples
        
        Returns:
            samples: List of tuples containing sample data
            sequence_number: Batch sequence number
        """
        # First 4 bytes: sample_count (uint16) + sequence_number (uint16)
        sample_count, seq_num = struct.unpack('<HH', data[:4])
        logger.info(f"Received batch: {sample_count} samples, seq={seq_num}")
        
        # Parse samples
        offset = 4
        samples = []
        
        for i in range(sample_count):
            sample_data = data[offset:offset + ROBOT_SAMPLE_STRUCT.size]
            if len(sample_data) < ROBOT_SAMPLE_STRUCT.size:
                logger.error(f"Incomplete sample data at index {i}")
                break
            
            sample = ROBOT_SAMPLE_STRUCT.unpack(sample_data)
            samples.append(sample)
            offset += ROBOT_SAMPLE_STRUCT.size
        
        return samples, seq_num
    
    def send_ack(self, client_socket: socket.socket, sequence_number: int):
        """Send acknowledgment to client"""
        # Simple ACK message: just header with ACK type
        ack_payload = struct.pack('<H', sequence_number)
        checksum = self.calculate_crc32(ack_payload)
        
        header = HEADER_STRUCT.pack(
            MSG_TYPE_ACK,
            PROTOCOL_VERSION,
            len(ack_payload),
            checksum
        )
        
        client_socket.sendall(header + ack_payload)
        logger.debug(f"Sent ACK for sequence {sequence_number}")
    
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
                samples, seq_num = self.parse_data_batch(payload)
                
                # Log data to files
                self.data_logger.log_samples(samples, seq_num)
                
                # Send acknowledgment
                self.send_ack(client_socket, seq_num)
        
        except Exception as e:
            logger.error(f"Error handling client: {e}", exc_info=True)
        finally:
            client_socket.close()
            logger.info(f"Connection closed: {address}")
            self.data_logger.finalize()
    
    def start(self):
        """Start the server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)
        
        logger.info(f"Data Logger Server listening on port {self.port}")
        logger.info(f"Logs will be saved to: {self.data_logger.log_dir}")
        
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
            self.data_logger.finalize()


if __name__ == "__main__":
    server = DataLoggerServer()
    server.start()
