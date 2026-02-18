#!/usr/bin/env python3
"""
Brage Telemetry Logger
======================
Serial logger with real-time TUI for Brage TDMA radio testing.

Usage:
    python brage_logger.py [OPTIONS]

Options:
    --port PORT        Serial port (default: auto-detect)
    --baud RATE        Baud rate (default: 230400)
    --output DIR       Output directory for logs (default: ./logs)
    --mock             Run with simulated data for testing
    --no-tui           Disable TUI, log to stdout only

Expected serial format:
    BRAGE,<timestamp_ms>,<sync_state>,<frame_num>,<rssi>,<snr>,<lq>,<msg_type>,<payload_hex>
    BRAGE_EVENT,<timestamp_ms>,<event_type>,<details>
"""

import argparse
import csv
import json
import os
import random
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass, field, asdict
from datetime import datetime
from pathlib import Path
from typing import Optional, Deque

# Serial import with graceful fallback
try:
    import serial
    import serial.tools.list_ports
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False

# Rich TUI imports with graceful fallback
try:
    from rich.console import Console, Group
    from rich.live import Live
    from rich.panel import Panel
    from rich.table import Table
    from rich.text import Text
    from rich.layout import Layout
    from rich.progress import BarColumn, Progress, TextColumn
    RICH_AVAILABLE = True
except ImportError:
    RICH_AVAILABLE = False


# =============================================================================
# Data Structures
# =============================================================================

@dataclass
class TelemetryPacket:
    """Parsed telemetry packet from Brage."""
    timestamp_ms: int
    sync_state: bool
    frame_num: int
    rssi: int
    snr: int
    lq: int
    msg_type: str
    payload_hex: str
    received_at: datetime = field(default_factory=datetime.now)
    
    @classmethod
    def from_csv_line(cls, line: str) -> Optional['TelemetryPacket']:
        """Parse a BRAGE CSV line into a TelemetryPacket."""
        try:
            parts = line.strip().split(',')
            if len(parts) < 8 or parts[0] != 'BRAGE':
                return None
            
            return cls(
                timestamp_ms=int(parts[1]),
                sync_state=parts[2] == '1',
                frame_num=int(parts[3]),
                rssi=int(parts[4]),
                snr=int(parts[5]),
                lq=int(parts[6]),
                msg_type=parts[7],
                payload_hex=parts[8] if len(parts) > 8 else '',
            )
        except (ValueError, IndexError):
            return None


@dataclass
class EventPacket:
    """Parsed event packet from Brage."""
    timestamp_ms: int
    event_type: str
    details: str
    received_at: datetime = field(default_factory=datetime.now)
    
    @classmethod
    def from_csv_line(cls, line: str) -> Optional['EventPacket']:
        """Parse a BRAGE_EVENT CSV line into an EventPacket."""
        try:
            parts = line.strip().split(',', 3)
            if len(parts) < 3 or parts[0] != 'BRAGE_EVENT':
                return None
            
            return cls(
                timestamp_ms=int(parts[1]),
                event_type=parts[2],
                details=parts[3] if len(parts) > 3 else '',
            )
        except (ValueError, IndexError):
            return None


@dataclass
class SessionStats:
    """Accumulated statistics for the session."""
    start_time: datetime = field(default_factory=datetime.now)
    packets_received: int = 0
    packets_expected: int = 0
    sync_losses: int = 0
    last_frame_num: int = -1
    last_sync_state: bool = False
    
    # Rolling window for link quality calculation
    rssi_history: Deque[int] = field(default_factory=lambda: deque(maxlen=100))
    snr_history: Deque[int] = field(default_factory=lambda: deque(maxlen=100))
    frame_received: Deque[bool] = field(default_factory=lambda: deque(maxlen=100))
    
    def update(self, packet: TelemetryPacket):
        """Update stats with a new packet."""
        self.packets_received += 1

        # Only include real signal values in averages (skip NONE frames)
        if packet.msg_type != 'NONE':
            self.rssi_history.append(packet.rssi)
            self.snr_history.append(packet.snr)

        # Track frame gaps for packet loss calculation
        # frameSeq is uint16_t on firmware, wraps at 65536
        if self.last_frame_num >= 0:
            expected_frame = (self.last_frame_num + 1) & 0xFFFF
            if packet.frame_num != expected_frame:
                if packet.frame_num > self.last_frame_num:
                    missed = packet.frame_num - self.last_frame_num - 1
                else:
                    # uint16 wraparound
                    missed = (0xFFFF - self.last_frame_num) + packet.frame_num
                self.packets_expected += missed
                for _ in range(min(missed, 100)):
                    self.frame_received.append(False)

        self.frame_received.append(True)
        self.packets_expected += 1

        # Track sync state transitions
        if self.last_sync_state and not packet.sync_state:
            self.sync_losses += 1

        self.last_frame_num = packet.frame_num
        self.last_sync_state = packet.sync_state
    
    @property
    def uptime_seconds(self) -> float:
        return (datetime.now() - self.start_time).total_seconds()
    
    @property
    def uptime_str(self) -> str:
        total_seconds = int(self.uptime_seconds)
        hours, remainder = divmod(total_seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        return f"{hours:02d}:{minutes:02d}:{seconds:02d}"
    
    @property
    def avg_rssi(self) -> float:
        if not self.rssi_history:
            return 0.0
        return sum(self.rssi_history) / len(self.rssi_history)
    
    @property
    def avg_snr(self) -> float:
        if not self.snr_history:
            return 0.0
        return sum(self.snr_history) / len(self.snr_history)
    
    @property
    def link_quality(self) -> float:
        """Calculate link quality as percentage of received frames."""
        if not self.frame_received:
            return 100.0
        return (sum(self.frame_received) / len(self.frame_received)) * 100
    
    @property
    def packet_error_rate(self) -> float:
        """Calculate overall packet error rate."""
        if self.packets_expected == 0:
            return 0.0
        return ((self.packets_expected - self.packets_received) / self.packets_expected) * 100


# =============================================================================
# Serial Handler
# =============================================================================

class SerialHandler:
    """Handles serial port communication."""
    
    def __init__(self, port: Optional[str] = None, baud: int = 230400):
        self.port = port
        self.baud = baud
        self.serial: Optional[serial.Serial] = None
        self.connected = False
        self.error_message = ""
    
    def auto_detect_port(self) -> Optional[str]:
        """Try to auto-detect a connected Brage/STM32 device."""
        if not SERIAL_AVAILABLE:
            return None
        
        ports = serial.tools.list_ports.comports()
        for port in ports:
            # Common USB-serial chip identifiers
            desc_lower = (port.description or '').lower()
            hwid_lower = (port.hwid or '').lower()
            if any(x in desc_lower for x in ['stm32', 'stlink', 'usb serial', 'ch340', 'cp210', 'ftdi', 'cdc']) or \
               'usb' in hwid_lower:
                return port.device
        
        # Fall back to first available port
        if ports:
            return ports[0].device
        
        return None
    
    def connect(self) -> bool:
        """Establish serial connection."""
        if not SERIAL_AVAILABLE:
            self.error_message = "pyserial not installed"
            return False
        
        if self.port is None:
            self.port = self.auto_detect_port()
            if self.port is None:
                self.error_message = "No serial port found"
                return False
        
        # STM32 CDC ACM ports can throw I/O errors transiently,
        # especially right after another process releases the port.
        # Retry a few times with a delay.
        for attempt in range(3):
            try:
                self.serial = serial.Serial()
                self.serial.port = self.port
                self.serial.baudrate = self.baud
                self.serial.timeout = 0.1
                self.serial.dsrdtr = False
                self.serial.rtscts = False
                self.serial.open()
                self.connected = True
                self.error_message = ""
                return True
            except serial.SerialException as e:
                self.error_message = str(e)
                self.connected = False
                if attempt < 2:
                    time.sleep(1)

        return False
    
    def disconnect(self):
        """Close serial connection."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False
    
    def readline(self) -> Optional[str]:
        """Read a line from serial port."""
        if not self.serial or not self.serial.is_open:
            return None
        
        try:
            line = self.serial.readline()
            if line:
                return line.decode('utf-8', errors='replace').strip()
        except serial.SerialException as e:
            self.error_message = str(e)
            self.connected = False
        
        return None
    
    def reconnect(self) -> bool:
        """Attempt to reconnect."""
        self.disconnect()
        time.sleep(1)
        return self.connect()


class MockSerialHandler:
    """Simulated serial handler for testing without hardware."""
    
    def __init__(self):
        self.connected = True
        self.error_message = ""
        self.port = "MOCK"
        self.baud = 230400
        self.frame_num = 0
        self.start_time = time.time()
        self.last_packet_time = 0
        self.synced = True
    
    def connect(self) -> bool:
        self.connected = True
        return True
    
    def disconnect(self):
        self.connected = False
    
    def readline(self) -> Optional[str]:
        """Generate simulated telemetry data."""
        # Simulate ~22 Hz update rate
        current_time = time.time()
        if current_time - self.last_packet_time < 0.045:
            time.sleep(0.01)
            return None
        
        self.last_packet_time = current_time
        self.frame_num += 1
        
        # Simulate occasional sync loss
        elapsed = current_time - self.start_time
        if 72 * 60 < elapsed < 72 * 60 + 5:  # Simulate timer overflow at ~72 min
            self.synced = False
        elif elapsed > 72 * 60 + 5:
            self.synced = True
        
        # Simulate occasional packet loss (2%)
        if random.random() < 0.02:
            self.frame_num += 1  # Skip a frame
        
        # Generate realistic-ish values
        rssi = random.randint(-55, -40)
        snr = random.randint(6, 12)
        lq = random.randint(95, 100)
        msg_type = random.choice(['DL', 'DL', 'DL', 'UL'])  # More DL than UL
        
        timestamp_ms = int((current_time - self.start_time) * 1000) & 0xFFFFFFFF
        
        return f"BRAGE,{timestamp_ms},{1 if self.synced else 0},{self.frame_num},{rssi},{snr},{lq},{msg_type},"
    
    def reconnect(self) -> bool:
        return True


# =============================================================================
# Data Logger
# =============================================================================

class DataLogger:
    """Handles logging data to files."""
    
    def __init__(self, output_dir: str = "./logs"):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Create timestamped filenames
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.raw_file = self.output_dir / f"{timestamp}_raw.log"
        self.data_file = self.output_dir / f"{timestamp}_data.csv"
        self.session_file = self.output_dir / f"{timestamp}_session.json"
        
        # Open files
        self.raw_handle = open(self.raw_file, 'w')
        self.data_handle = open(self.data_file, 'w', newline='')
        self.csv_writer = csv.writer(self.data_handle)
        
        # Write CSV header
        self.csv_writer.writerow([
            'received_at', 'timestamp_ms', 'sync_state', 'frame_num',
            'rssi', 'snr', 'lq', 'msg_type', 'payload_hex'
        ])
        
        self.session_info = {
            'start_time': datetime.now().isoformat(),
            'end_time': None,
            'total_packets': 0,
            'sync_losses': 0,
            'notes': ''
        }
    
    def log_raw(self, line: str):
        """Log raw serial line."""
        timestamp = datetime.now().isoformat()
        self.raw_handle.write(f"[{timestamp}] {line}\n")
        self.raw_handle.flush()
    
    def log_packet(self, packet: TelemetryPacket):
        """Log parsed packet to CSV."""
        self.csv_writer.writerow([
            packet.received_at.isoformat(),
            packet.timestamp_ms,
            1 if packet.sync_state else 0,
            packet.frame_num,
            packet.rssi,
            packet.snr,
            packet.lq,
            packet.msg_type,
            packet.payload_hex
        ])
        self.data_handle.flush()
        self.session_info['total_packets'] += 1
    
    def log_event(self, event: EventPacket):
        """Log event to raw file."""
        self.log_raw(f"EVENT: {event.event_type} - {event.details}")
    
    def finalize(self, stats: SessionStats, notes: str = ""):
        """Write session summary and close files."""
        self.session_info['end_time'] = datetime.now().isoformat()
        self.session_info['sync_losses'] = stats.sync_losses
        self.session_info['packet_error_rate'] = stats.packet_error_rate
        self.session_info['avg_rssi'] = stats.avg_rssi
        self.session_info['avg_snr'] = stats.avg_snr
        self.session_info['notes'] = notes
        
        with open(self.session_file, 'w') as f:
            json.dump(self.session_info, f, indent=2)
        
        self.raw_handle.close()
        self.data_handle.close()
    
    @property
    def filenames(self) -> dict:
        return {
            'raw': str(self.raw_file),
            'data': str(self.data_file),
            'session': str(self.session_file)
        }


# =============================================================================
# TUI Display
# =============================================================================

def create_rssi_bar(rssi: float, width: int = 20) -> str:
    """Create a text-based RSSI bar."""
    # RSSI typically ranges from -120 (bad) to -30 (excellent)
    # Map to 0-100%
    percent = max(0, min(100, (rssi + 120) / 90 * 100))
    filled = int(width * percent / 100)
    empty = width - filled
    
    if percent > 70:
        color = "green"
    elif percent > 40:
        color = "yellow"
    else:
        color = "red"
    
    return f"[{color}]{'█' * filled}[/{color}][dim]{'░' * empty}[/dim]"


def create_snr_bar(snr: float, width: int = 20) -> str:
    """Create a text-based SNR bar."""
    # SNR typically ranges from -5 (bad) to 15 (excellent)
    percent = max(0, min(100, (snr + 5) / 20 * 100))
    filled = int(width * percent / 100)
    empty = width - filled
    
    if percent > 70:
        color = "green"
    elif percent > 40:
        color = "yellow"
    else:
        color = "red"
    
    return f"[{color}]{'█' * filled}[/{color}][dim]{'░' * empty}[/dim]"


def build_display(
    serial_handler,
    stats: SessionStats,
    last_packet: Optional[TelemetryPacket],
    events: Deque[str],
    logger: DataLogger
) -> Panel:
    """Build the TUI display panel."""
    
    # Connection status
    if serial_handler.connected:
        conn_status = f"[green]● CONNECTED[/green] ({serial_handler.port} @ {serial_handler.baud})"
    else:
        conn_status = f"[red]● DISCONNECTED[/red] ({serial_handler.error_message})"
    
    # Sync status
    if last_packet:
        if last_packet.sync_state:
            sync_status = "[green bold]◉ SYNCED[/green bold]"
        else:
            sync_status = "[red bold]◯ UNSYNCED[/red bold]"
    else:
        sync_status = "[dim]◯ NO DATA[/dim]"
    
    # Build stats table
    stats_table = Table(show_header=False, box=None, padding=(0, 2))
    stats_table.add_column("Label", style="bold")
    stats_table.add_column("Value")
    
    stats_table.add_row("Uptime", stats.uptime_str)
    stats_table.add_row("Packets RX", str(stats.packets_received))
    stats_table.add_row("Sync Losses", f"[{'red' if stats.sync_losses > 0 else 'green'}]{stats.sync_losses}[/]")
    stats_table.add_row("PER", f"{stats.packet_error_rate:.2f}%")
    
    # Build signal table
    signal_table = Table(show_header=False, box=None, padding=(0, 2))
    signal_table.add_column("Label", style="bold")
    signal_table.add_column("Bar", width=22)
    signal_table.add_column("Value", justify="right")
    
    rssi = last_packet.rssi if last_packet else 0
    snr = last_packet.snr if last_packet else 0
    lq = stats.link_quality
    
    signal_table.add_row("RSSI", create_rssi_bar(rssi), f"{rssi} dBm")
    signal_table.add_row("SNR", create_snr_bar(snr), f"{snr} dB")
    signal_table.add_row("Avg RSSI", create_rssi_bar(stats.avg_rssi), f"{stats.avg_rssi:.1f} dBm")
    signal_table.add_row("Avg SNR", create_snr_bar(stats.avg_snr), f"{stats.avg_snr:.1f} dB")
    signal_table.add_row("Link Quality", "", f"[{'green' if lq > 99 else 'yellow' if lq > 95 else 'red'}]{lq:.1f}%[/]")
    
    # Frame info
    frame_info = ""
    if last_packet:
        frame_info = f"Frame: {last_packet.frame_num}  |  Type: {last_packet.msg_type}  |  Device time: {last_packet.timestamp_ms} ms"
    
    # Recent events
    events_text = "\n".join(events) if events else "[dim]No events[/dim]"
    
    # File info
    files_text = f"[dim]Logging to: {logger.output_dir}[/dim]"
    
    # Compose layout using Group so Rich renderables display properly
    content = Group(
        Text.from_markup(f"{conn_status}    {sync_status}\n"),
        Text.from_markup("[bold]Session Statistics[/bold]"),
        stats_table,
        Text(""),
        Text.from_markup("[bold]Signal Quality[/bold]"),
        signal_table,
        Text(""),
        Text.from_markup(f"[bold]Current Frame[/bold]\n{frame_info}\n"),
        Text.from_markup(f"[bold]Recent Events[/bold]\n{events_text}\n"),
        Text.from_markup(f"{files_text}"),
        Text.from_markup("[dim]Press Ctrl+C to stop logging[/dim]"),
    )

    return Panel(
        content,
        title="[bold blue]Brage Telemetry Logger[/bold blue]",
        subtitle=f"[dim]{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}[/dim]"
    )


# =============================================================================
# Main Application
# =============================================================================

def run_with_tui(serial_handler, logger: DataLogger):
    """Run the logger with TUI display."""
    console = Console()
    stats = SessionStats()
    last_packet: Optional[TelemetryPacket] = None
    events: Deque[str] = deque(maxlen=5)
    running = True
    
    def read_serial():
        nonlocal last_packet, running
        while running:
            line = serial_handler.readline()
            if line:
                logger.log_raw(line)
                
                # Try to parse as telemetry packet
                packet = TelemetryPacket.from_csv_line(line)
                if packet:
                    stats.update(packet)
                    logger.log_packet(packet)
                    last_packet = packet
                    continue
                
                # Try to parse as event
                event = EventPacket.from_csv_line(line)
                if event:
                    logger.log_event(event)
                    events.appendleft(f"[{event.event_type}] {event.details}")
                    continue
                
                # Unknown line format - just log it
                if line and not line.startswith('#'):
                    events.appendleft(f"[dim]{line[:50]}...[/dim]" if len(line) > 50 else f"[dim]{line}[/dim]")
    
    # Start serial reading thread
    serial_thread = threading.Thread(target=read_serial, daemon=True)
    serial_thread.start()
    
    try:
        with Live(console=console, refresh_per_second=4, screen=True) as live:
            while True:
                display = build_display(serial_handler, stats, last_packet, events, logger)
                live.update(display)
                time.sleep(0.1)
    except KeyboardInterrupt:
        running = False
        console.print("\n[yellow]Stopping logger...[/yellow]")
    
    # Finalize
    logger.finalize(stats)
    console.print(f"\n[green]Logs saved to:[/green]")
    for name, path in logger.filenames.items():
        console.print(f"  {name}: {path}")
    
    console.print(f"\n[bold]Session Summary:[/bold]")
    console.print(f"  Duration: {stats.uptime_str}")
    console.print(f"  Packets received: {stats.packets_received}")
    console.print(f"  Packet error rate: {stats.packet_error_rate:.2f}%")
    console.print(f"  Sync losses: {stats.sync_losses}")
    console.print(f"  Avg RSSI: {stats.avg_rssi:.1f} dBm")
    console.print(f"  Avg SNR: {stats.avg_snr:.1f} dB")


def run_without_tui(serial_handler, logger: DataLogger):
    """Run the logger with simple stdout output."""
    stats = SessionStats()
    
    print(f"Brage Logger started - logging to {logger.output_dir}")
    print(f"Port: {serial_handler.port} @ {serial_handler.baud}")
    print("Press Ctrl+C to stop\n")
    
    try:
        while True:
            line = serial_handler.readline()
            if line:
                logger.log_raw(line)
                print(f"[{datetime.now().strftime('%H:%M:%S')}] {line}")
                
                packet = TelemetryPacket.from_csv_line(line)
                if packet:
                    stats.update(packet)
                    logger.log_packet(packet)
                
                event = EventPacket.from_csv_line(line)
                if event:
                    logger.log_event(event)
    
    except KeyboardInterrupt:
        print("\nStopping logger...")
    
    logger.finalize(stats)
    print(f"\nLogs saved to: {logger.output_dir}")
    print(f"Session duration: {stats.uptime_str}")
    print(f"Packets received: {stats.packets_received}")


def main():
    parser = argparse.ArgumentParser(description="Brage Telemetry Logger")
    parser.add_argument('--port', '-p', help="Serial port (default: auto-detect)")
    parser.add_argument('--baud', '-b', type=int, default=230400, help="Baud rate (default: 230400)")
    parser.add_argument('--output', '-o', default='./logs', help="Output directory (default: ./logs)")
    parser.add_argument('--mock', action='store_true', help="Use simulated data for testing")
    parser.add_argument('--no-tui', action='store_true', help="Disable TUI, use simple stdout")
    
    args = parser.parse_args()
    
    # Check dependencies
    if not args.mock and not SERIAL_AVAILABLE:
        print("Error: pyserial not installed. Install with: pip install pyserial")
        print("       Or use --mock to test with simulated data.")
        sys.exit(1)
    
    if not args.no_tui and not RICH_AVAILABLE:
        print("Warning: rich not installed. Falling back to simple output.")
        print("         Install with: pip install rich")
        args.no_tui = True
    
    # Create serial handler
    if args.mock:
        serial_handler = MockSerialHandler()
        print("Running in MOCK mode with simulated data")
    else:
        serial_handler = SerialHandler(port=args.port, baud=args.baud)
        if not serial_handler.connect():
            print(f"Error: Could not connect to serial port: {serial_handler.error_message}")
            sys.exit(1)
    
    # Create logger
    logger = DataLogger(output_dir=args.output)
    
    # Run
    try:
        if args.no_tui:
            run_without_tui(serial_handler, logger)
        else:
            run_with_tui(serial_handler, logger)
    finally:
        serial_handler.disconnect()


if __name__ == "__main__":
    main()
