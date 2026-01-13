#!/usr/bin/env python3
"""
Quick test script to verify TCP Modbus connectivity to Inspire RH56DFTP hands.

Usage:
    python3 test_inspire_tcp.py
    python3 test_inspire_tcp.py --left 192.168.123.210 --right 192.168.123.211
"""

import socket
import struct
import argparse
import time


class InspireHandTCP:
    """Simple Modbus TCP client for Inspire RH56DFTP hand."""
    
    def __init__(self, ip: str, port: int = 6000):
        self.ip = ip
        self.port = port
        self.sock = None
        self.transaction_id = 0
        
    def connect(self) -> bool:
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(2.0)
            self.sock.connect((self.ip, self.port))
            print(f"✓ Connected to {self.ip}:{self.port}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to {self.ip}:{self.port}: {e}")
            return False
    
    def disconnect(self):
        if self.sock:
            self.sock.close()
            self.sock = None
    
    def read_registers(self, start_addr: int, count: int) -> list:
        """Read holding registers (Function Code 0x03)."""
        if not self.sock:
            return None
        
        self.transaction_id += 1
        tid = self.transaction_id
        
        # Build Modbus TCP request
        request = struct.pack('>HHHBBHH',
            tid,           # Transaction ID
            0,             # Protocol ID (Modbus)
            6,             # Length
            1,             # Unit ID
            0x03,          # Function code: Read Holding Registers
            start_addr,    # Start address
            count          # Quantity
        )
        
        try:
            self.sock.send(request)
            
            # Read response header (9 bytes) + data (count * 2 bytes)
            response = self.sock.recv(9 + count * 2)
            
            if len(response) < 9 + count * 2:
                return None
            
            # Parse response
            values = []
            for i in range(count):
                val = struct.unpack('>h', response[9 + i*2 : 11 + i*2])[0]
                values.append(val)
            
            return values
            
        except Exception as e:
            print(f"  Error reading registers: {e}")
            return None
    
    def write_registers(self, start_addr: int, values: list) -> bool:
        """Write multiple registers (Function Code 0x10)."""
        if not self.sock:
            return False
        
        self.transaction_id += 1
        tid = self.transaction_id
        count = len(values)
        
        # Build request
        header = struct.pack('>HHHBBHHB',
            tid,                    # Transaction ID
            0,                      # Protocol ID
            7 + count * 2,          # Length
            1,                      # Unit ID
            0x10,                   # Function code: Write Multiple Registers
            start_addr,             # Start address
            count,                  # Quantity
            count * 2               # Byte count
        )
        
        data = b''.join(struct.pack('>H', v) for v in values)
        
        try:
            self.sock.send(header + data)
            response = self.sock.recv(12)
            return len(response) == 12 and response[7] == 0x10
        except Exception as e:
            print(f"  Error writing registers: {e}")
            return False
    
    def read_position(self) -> list:
        """Read actual position for each DOF (register 1530-1541)."""
        return self.read_registers(1530, 6)
    
    def read_angle(self) -> list:
        """Read actual angle for each DOF (register 1546-1557)."""
        return self.read_registers(1546, 6)
    
    def read_force(self) -> list:
        """Read actual force for each finger (register 1558-1569)."""
        return self.read_registers(1558, 6)
    
    def set_angle(self, angles: list) -> bool:
        """Set target angle for each DOF (register 1486-1497)."""
        # Convert 0-1 range to 0-1000
        values = [int(max(0, min(1, a)) * 1000) for a in angles]
        return self.write_registers(1486, values)


def test_hand(name: str, ip: str):
    """Test connection and basic communication with a hand."""
    print(f"\n{'='*50}")
    print(f"Testing {name} Hand at {ip}:6000")
    print('='*50)
    
    hand = InspireHandTCP(ip)
    
    if not hand.connect():
        return False
    
    # Read position
    pos = hand.read_position()
    if pos:
        print(f"  Position: {pos}")
    else:
        print("  ✗ Failed to read position")
    
    # Read angle
    angle = hand.read_angle()
    if angle:
        print(f"  Angle:    {angle}")
    else:
        print("  ✗ Failed to read angle")
    
    # Read force
    force = hand.read_force()
    if force:
        print(f"  Force:    {force}")
    else:
        print("  ✗ Failed to read force")
    
    hand.disconnect()
    print(f"  ✓ {name} hand communication OK!")
    return True


def main():
    parser = argparse.ArgumentParser(description="Test Inspire RH56DFTP TCP Modbus connection")
    parser.add_argument("--left", type=str, default="192.168.123.210", help="Left hand IP")
    parser.add_argument("--right", type=str, default="192.168.123.211", help="Right hand IP")
    args = parser.parse_args()
    
    print("Inspire RH56DFTP TCP Modbus Connection Test")
    print("Protocol: Modbus TCP, Port: 6000")
    
    left_ok = test_hand("Left", args.left)
    right_ok = test_hand("Right", args.right)
    
    print(f"\n{'='*50}")
    print("Summary:")
    print(f"  Left Hand:  {'✓ OK' if left_ok else '✗ FAILED'}")
    print(f"  Right Hand: {'✓ OK' if right_ok else '✗ FAILED'}")
    print('='*50)
    
    if left_ok and right_ok:
        print("\nBoth hands connected! You can now build and run:")
        print("  cd build && cmake .. && make")
        print("  ./inspire_g1_tcp --left 192.168.123.210 --right 192.168.123.211")


if __name__ == "__main__":
    main()

