#!/bin/bash
# Quick Start Guide for Data Logger Server

echo "=========================================="
echo "PID Autotuning - Data Logger Server Setup"
echo "=========================================="
echo ""

# Check Python version
echo "1. Checking Python version..."
python3 --version

# Install dependencies
echo ""
echo "2. Installing Python dependencies..."
echo "   (This will install numpy, pandas, and matplotlib)"
read -p "   Install now? [Y/n]: " install
if [ "$install" != "n" ] && [ "$install" != "N" ]; then
    pip install -r requirements.txt
    echo "   ✓ Dependencies installed"
else
    echo "   Skipped. You can install later with: pip install -r requirements.txt"
fi

# Create log directory
echo ""
echo "3. Creating log directory..."
mkdir -p telemetry_logs
echo "   ✓ Created telemetry_logs/"

# Show usage
echo ""
echo "=========================================="
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To start the data logger server:"
echo "  ./data_logger_server.py"
echo "  or"
echo "  python3 data_logger_server.py"
echo ""
echo "To start the PID autotuner server:"
echo "  ./python_server.py"
echo "  or"
echo "  python3 python_server.py"
echo ""
echo "To analyze logged data:"
echo "  python3 analyze_telemetry.py telemetry_logs/<filename>.csv"
echo ""
echo "Server will listen on port 8888"
echo "Logs will be saved to: telemetry_logs/"
echo ""
echo "For more information, see SERVER_README.md"
echo ""
