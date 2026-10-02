"""ReSpeaker microphone and LED utility layer for the voice subsystem.

This file is relevant because it provides low-level access to the USB mic array
hardware, which supports bring-up, audio experiments, and LED-assisted voice
feedback outside the higher-level controller.
"""

import pyaudio
import numpy as np
from typing import Optional, Callable, Dict
import time
 
# Optional: USB control for LEDs (install with: pip install pyusb)
try:
    import usb.core
    import usb.util
    USB_AVAILABLE = True
except ImportError:
    USB_AVAILABLE = False
    print("Note: pyusb not installed. LED control will be disabled.")
    print("Install with: pip install pyusb")
 
 
class ReSpeakerAudioCapture:
    """
    Manages ReSpeaker Mic Array V3.0 USB audio interface.
    
    Features:
    - Auto-detection of ReSpeaker device
    - Audio stream management (blocking and callback modes)
    - Optional LED control via USB
    - Works with wake word detection and speech recognition
    """
    
    # ReSpeaker V3.0 USB identifiers
    RESPEAKER_VENDOR_ID = 0x2886  # Seeed Studio
    RESPEAKER_PRODUCT_ID = 0x0018  # ReSpeaker Mic Array V3.0
    
    # Audio configuration defaults
    DEFAULT_RATE = 16000  # Optimal for speech recognition
    DEFAULT_CHANNELS = 1  # Mono (use channel 0 - beamformed output)
    DEFAULT_CHUNK = 512
    DEFAULT_FORMAT = pyaudio.paInt16
    
    # LED control parameters
    NUM_LEDS = 12
    
    def __init__(
        self,
        rate: int = DEFAULT_RATE,
        channels: int = DEFAULT_CHANNELS,
        chunk: int = DEFAULT_CHUNK,
        format: int = DEFAULT_FORMAT
    ):
        """
        Initialize ReSpeaker audio capture.
        
        Args:
            rate: Sample rate in Hz (default: 16000)
            channels: Number of channels (1=mono, 6=all mics)
            chunk: Frames per buffer (default: 512)
            format: PyAudio format (default: paInt16)
        """
        self.rate = rate
        self.channels = channels
        self.chunk = chunk
        self.format = format
        
        self.pa = None
        self.stream = None
        self.device_index = None
        self.usb_device = None
        
        # Find ReSpeaker device
        self._find_respeaker_device()
        
    def _find_respeaker_device(self):
        """Locate ReSpeaker in system audio devices."""
        self.pa = pyaudio.PyAudio()
        
        print("Searching for ReSpeaker device...")
        
        # Search for ReSpeaker in audio devices
        for i in range(self.pa.get_device_count()):
            try:
                dev_info = self.pa.get_device_info_by_index(i)
                name = dev_info['name'].lower()
                
                # Look for ReSpeaker identifiers
                if 'respeaker' in name or 'seeed' in name or 'xvf3000' in name:
                    self.device_index = i
                    print(f"✓ Found ReSpeaker at device index {i}")
                    print(f"  Name: {dev_info['name']}")
                    print(f"  Input channels: {dev_info['maxInputChannels']}")
                    print(f"  Sample rate: {dev_info['defaultSampleRate']} Hz")
                    break
            except Exception as e:
                continue
        
        if self.device_index is None:
            print("⚠ ReSpeaker not found by name. Using default input device.")
            self.device_index = self.pa.get_default_input_device_info()['index']
        
        # Try to find USB device for LED control (optional)
        if USB_AVAILABLE:
            try:
                self.usb_device = usb.core.find(
                    idVendor=self.RESPEAKER_VENDOR_ID,
                    idProduct=self.RESPEAKER_PRODUCT_ID
                )
                if self.usb_device:
                    print("✓ ReSpeaker USB device found (LED control available)")
                else:
                    print("⚠ ReSpeaker USB device not found (LED control disabled)")
            except Exception as e:
                print(f"⚠ USB device access failed: {e}")
                self.usb_device = None
        else:
            print("⚠ USB control not available (pyusb not installed)")
    
    def get_device_info(self) -> Dict:
        """Get information about the ReSpeaker device."""
        if self.device_index is not None:
            return self.pa.get_device_info_by_index(self.device_index)
        return {}
    
    def open_stream(self, callback: Optional[Callable] = None):
        """
        Open audio stream from ReSpeaker.
        
        Args:
            callback: Optional callback function for non-blocking mode
                     Signature: callback(in_data, frame_count, time_info, status)
        """
        try:
            if callback:
                # Non-blocking mode with callback
                self.stream = self.pa.open(
                    rate=self.rate,
                    channels=self.channels,
                    format=self.format,
                    input=True,
                    input_device_index=self.device_index,
                    frames_per_buffer=self.chunk,
                    stream_callback=callback
                )
            else:
                # Blocking mode
                self.stream = self.pa.open(
                    rate=self.rate,
                    channels=self.channels,
                    format=self.format,
                    input=True,
                    input_device_index=self.device_index,
                    frames_per_buffer=self.chunk
                )
            
            print(f"Audio stream opened: {self.rate}Hz, {self.channels} channel(s)")
            return self.stream
            
        except Exception as e:
            print(f"Error opening audio stream: {e}")
            raise
    
    def start_stream(self):
        """Start the audio stream."""
        if self.stream and not self.stream.is_active():
            self.stream.start_stream()
            print("Audio stream started")
    
    def stop_stream(self):
        """Stop the audio stream."""
        if self.stream and self.stream.is_active():
            self.stream.stop_stream()
            print("Audio stream stopped")
    
    def close_stream(self):
        """Close the audio stream."""
        if self.stream:
            if self.stream.is_active():
                self.stream.stop_stream()
            self.stream.close()
            self.stream = None
            print("Audio stream closed")
    
    def read_audio(self, num_frames: Optional[int] = None) -> bytes:
        """
        Read audio data from stream (blocking mode).
        
        Args:
            num_frames: Number of frames to read (default: chunk size)
            
        Returns:
            Raw audio data as bytes
        """
        if not self.stream:
            raise RuntimeError("Audio stream not opened. Call open_stream() first.")
        
        frames = num_frames if num_frames is not None else self.chunk
        
        try:
            return self.stream.read(frames, exception_on_overflow=False)
        except IOError as e:
            # Handle buffer overflow gracefully
            print(f"Audio buffer overflow (this is normal on busy systems)")
            return b'\x00' * (frames * self.channels * 2)
    
    def read_audio_array(self, num_frames: Optional[int] = None) -> np.ndarray:
        """
        Read audio data as numpy array (blocking mode).
        
        Args:
            num_frames: Number of frames to read
            
        Returns:
            Audio data as numpy array (int16)
        """
        audio_bytes = self.read_audio(num_frames)
        audio_array = np.frombuffer(audio_bytes, dtype=np.int16)
        
        # Reshape if multi-channel
        if self.channels > 1:
            audio_array = audio_array.reshape(-1, self.channels)
        
        return audio_array
    
    # ==================== LED Control Methods ====================
    
    def set_led_color(self, led_index: int, r: int, g: int, b: int):
        """
        Set color of a specific LED.
        
        Args:
            led_index: LED index (0-11)
            r: Red value (0-255)
            g: Green value (0-255)
            b: Blue value (0-255)
        """
        if not self.usb_device or not USB_AVAILABLE:
            return  # Silently skip if USB not available
        
        if not (0 <= led_index < self.NUM_LEDS):
            return
        
        try:
            # USB control transfer for LED control
            self.usb_device.ctrl_transfer(
                0x40,  # bmRequestType
                0xC5,  # bRequest (LED control command)
                led_index,  # wValue (LED index)
                0,  # wIndex
                [r, g, b]  # RGB data
            )
        except Exception as e:
            # Silently fail - LED control is optional
            pass
    
    def set_all_leds(self, r: int, g: int, b: int):
        """Set all LEDs to the same color."""
        for i in range(self.NUM_LEDS):
            self.set_led_color(i, r, g, b)
    
    def clear_leds(self):
        """Turn off all LEDs."""
        self.set_all_leds(0, 0, 0)
    
    def led_pattern_wakeword(self):
        """Display wake word detected pattern (blue flash)."""
        self.set_all_leds(0, 100, 255)
        time.sleep(0.2)
        self.clear_leds()
    
    def led_pattern_listening(self):
        """Display listening pattern (pulsing green)."""
        for brightness in range(0, 256, 32):
            self.set_all_leds(0, brightness, 0)
            time.sleep(0.05)
        for brightness in range(255, -1, -32):
            self.set_all_leds(0, brightness, 0)
            time.sleep(0.05)
    
    def led_pattern_processing(self):
        """Display processing pattern (rotating blue)."""
        for i in range(self.NUM_LEDS):
            self.set_led_color(i, 0, 50, 255)
            time.sleep(0.05)
            self.set_led_color(i, 0, 0, 0)
    
    def led_pattern_success(self):
        """Display success pattern (green flash)."""
        self.set_all_leds(0, 255, 0)
        time.sleep(0.3)
        self.clear_leds()
    
    def led_pattern_error(self):
        """Display error pattern (red flash)."""
        self.set_all_leds(255, 0, 0)
        time.sleep(0.3)
        self.clear_leds()
    
    # ==================== Cleanup ====================
    
    def cleanup(self):
        """Clean up all resources."""
        self.close_stream()
        if self.pa:
            self.pa.terminate()
            self.pa = None
        self.clear_leds()
        print("Audio capture cleaned up")
    
    def __enter__(self):
        """Context manager entry."""
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.cleanup()
 
 
# ==================== Utility Functions ====================
 
def list_audio_devices():
    """List all available audio devices."""
    pa = pyaudio.PyAudio()
    print("\n" + "=" * 70)
    print("Available Audio Devices")
    print("=" * 70)
    
    for i in range(pa.get_device_count()):
        try:
            info = pa.get_device_info_by_index(i)
            if info['maxInputChannels'] > 0:
                print(f"\nDevice {i}: {info['name']}")
                print(f"  Input Channels: {info['maxInputChannels']}")
                print(f"  Sample Rate: {info['defaultSampleRate']} Hz")
        except Exception as e:
            print(f"Error reading device {i}: {e}")
    
    pa.terminate()
    print("=" * 70 + "\n")
 
 
def test_respeaker_audio(duration: int = 5):
    """
    Test ReSpeaker audio capture.
    
    Args:
        duration: Recording duration in seconds
    """
    print(f"\nTesting ReSpeaker audio capture for {duration} seconds...")
    print("Make some noise during the recording!\n")
    
    try:
        with ReSpeakerAudioCapture() as capture:
            capture.open_stream()
            capture.start_stream()
            
            # LED feedback
            capture.led_pattern_listening()
            
            print("Recording...")
            frames = []
            num_chunks = int(capture.rate / capture.chunk * duration)
            
            for i in range(num_chunks):
                data = capture.read_audio()
                frames.append(data)
                
                # Progress indicator
                if i % 10 == 0:
                    print(f"  {i * capture.chunk / capture.rate:.1f}s / {duration}s")
            
            print("Recording complete!")
            capture.led_pattern_success()
            
            # Analyze audio
            audio_array = np.frombuffer(b''.join(frames), dtype=np.int16)
            print(f"\nAudio Statistics:")
            print(f"  Total samples: {len(audio_array)}")
            print(f"  Mean: {np.mean(audio_array):.2f}")
            print(f"  Std Dev: {np.std(audio_array):.2f}")
            print(f"  Min: {np.min(audio_array)}")
            print(f"  Max: {np.max(audio_array)}")
            
            rms = np.sqrt(np.mean(np.square(audio_array.astype(float))))
            print(f"  RMS Level: {rms:.2f}")
            
            if rms > 100:
                print("\n✓ Audio capture working well!")
            elif rms > 10:
                print("\n⚠ Audio captured but very quiet")
            else:
                print("\n✗ No audio detected")
                
    except Exception as e:
        print(f"\n✗ Test failed: {e}")
        import traceback
        traceback.print_exc()
 
 
def test_respeaker_leds():
    """Test ReSpeaker LED patterns."""
    if not USB_AVAILABLE:
        print("⚠ LED testing requires pyusb: pip install pyusb")
        return
    
    print("\nTesting ReSpeaker LED patterns...")
    
    try:
        with ReSpeakerAudioCapture() as capture:
            if not capture.usb_device:
                print("⚠ USB device not accessible for LED control")
                return
            
            print("Testing individual LEDs (red)...")
            for i in range(capture.NUM_LEDS):
                capture.set_led_color(i, 255, 0, 0)
                time.sleep(0.1)
                capture.set_led_color(i, 0, 0, 0)
            
            time.sleep(0.5)
            
            print("Testing all LEDs (green)...")
            capture.set_all_leds(0, 255, 0)
            time.sleep(1)
            capture.clear_leds()
            
            time.sleep(0.5)
            
            print("Testing wake word pattern...")
            capture.led_pattern_wakeword()
            time.sleep(0.5)
            
            print("Testing listening pattern...")
            capture.led_pattern_listening()
            time.sleep(0.5)
            
            print("Testing success pattern...")
            capture.led_pattern_success()
            time.sleep(0.5)
            
            print("Testing error pattern...")
            capture.led_pattern_error()
            
            print("✓ LED test complete")
            
    except Exception as e:
        print(f"✗ LED test failed: {e}")
 
 
# ==================== Main Test Interface ====================
 
if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        command = sys.argv[1]
        
        if command == "list":
            list_audio_devices()
        elif command == "test-audio":
            duration = int(sys.argv[2]) if len(sys.argv) > 2 else 5
            test_respeaker_audio(duration)
        elif command == "test-leds":
            test_respeaker_leds()
        else:
            print(f"Unknown command: {command}")
            print("Available commands: list, test-audio, test-leds")
    else:
        # Show usage
        print("=" * 70)
        print("ReSpeaker Mic Array V3.0 - Audio Capture Module")
        print("=" * 70)
        print("\nUsage:")
        print("  python audio_capture.py list")
        print("    → List all audio devices")
        print("\n  python audio_capture.py test-audio [duration]")
        print("    → Test audio capture (default: 5 seconds)")
        print("\n  python audio_capture.py test-leds")
        print("    → Test LED patterns")
        print("\nExample Integration:")
        print("  from audio_capture import ReSpeakerAudioCapture")
        print("  ")
        print("  with ReSpeakerAudioCapture() as capture:")
        print("      capture.open_stream()")
        print("      capture.start_stream()")
        print("      audio_data = capture.read_audio()")
        print("      # Use audio_data with wake_word.py or speech_to_text.py")
        print("=" * 70)
 
