import serial
import wave
import sys

# Configuration - ADJUST THESE FOR YOUR SETUP
SERIAL_PORT = 'COM5'  # Windows: COM3, Linux: /dev/ttyUSB0, Mac: /dev/cu.usbserial-*
BAUD_RATE = 921600   # Change from 115200
SAMPLE_RATE = 48000
CHANNELS = 1              # Stereo from your TDM setup
SAMPLE_WIDTH = 2      # 16-bit

def capture_audio(output_file, duration=1):
    print(f"Connecting to {SERIAL_PORT}...")
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=2)
    
    # Clear buffer
    ser.reset_input_buffer()
    
    # Send record command
    print("Sending record command...")
    ser.write(b'r')
    
    # Wait for START_RECORDING marker
    while True:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"ESP32: {line}")
        if "START_RECORDING" in line:
            print("Recording started...")
            break
    
    # Calculate expected data size
    expected_bytes = SAMPLE_RATE * duration * SAMPLE_WIDTH * CHANNELS
    audio_data = bytearray()
    
    print(f"Capturing {expected_bytes} bytes...")
    
    # Capture audio data
    while len(audio_data) < expected_bytes:
        chunk = ser.read(min(4096, expected_bytes - len(audio_data)))
        if chunk:
            audio_data.extend(chunk)
            progress = 100 * len(audio_data) / expected_bytes
            print(f"Progress: {progress:.1f}% ({len(audio_data)}/{expected_bytes} bytes)", end='\r')
    
    ser.close()
    print(f"\nCapture complete: {len(audio_data)} bytes received")
    
    # Save as WAV file
    with wave.open(output_file, 'wb') as wav_file:
        wav_file.setnchannels(CHANNELS)
        wav_file.setsampwidth(SAMPLE_WIDTH)
        wav_file.setframerate(SAMPLE_RATE)
        wav_file.writeframes(audio_data)
    
    print(f"Audio saved to: {output_file}")
    print(f"Format: {SAMPLE_RATE}Hz, {CHANNELS} channels, {SAMPLE_WIDTH*8}-bit")

if __name__ == "__main__":
    output = sys.argv[1] if len(sys.argv) > 1 else "recording.wav"
    capture_audio(output, duration=5)
