
import sounddevice as sd
import numpy as np
import vosk
import pyaudio

# model = vosk.Model("model")
# rec = vosk.KaldiRecognizer(model, 16000)

p = pyaudio.PyAudio()

# Print device info
for i in range(p.get_device_count()):
    device_info = p.get_device_info_by_index(i)
    print(f"Device {i}: {device_info['name']}")
    print(f"  - Input Channels: {device_info['maxInputChannels']}")
    print(f"  - Output Channels: {device_info['maxOutputChannels']}")
    print(f"  - Sample Rate: {device_info['defaultSampleRate']}\n")

p.terminate()



# devices = sd.query_devices()
# for i, dev in enumerate(devices):
#     # print(f"Device {i}: {dev['name']}")
#     if dev['max_input_channels'] > 0:  # Only list input devices
#         print(f"Device {i}: {dev['name']}")




def check_microphone(device=None, duration=2, threshold=0.01):
    """
    Checks if the microphone is receiving audio.
    
    Parameters:
        device (int or None): Audio input device ID (None for default).
        duration (int): Duration to record (in seconds).
        threshold (float): Amplitude threshold for detecting sound.
    
    Returns:
        bool: True if sound is detected, False otherwise.
    """
    
    print("Recording...")
    
    # Record audio
    try:
        audio_data = sd.rec(int(duration * 44100), samplerate=44100, channels=1, dtype='float32', device=device)
        sd.wait()  # Wait until recording is finished
    except Exception as e:
        print(f"Error accessing microphone: {e}")
        return False

    # Check if the signal is above the threshold
    amplitude = np.max(np.abs(audio_data))  # Get peak amplitude
    print(f"Max amplitude: {amplitude:.5f}")

    if amplitude > threshold:
        print("Sound detected! 🎤✅")
        return True
    else:
        print("No significant sound detected. 🔇")
        return False

# Run the check
check_microphone(device = 4)
