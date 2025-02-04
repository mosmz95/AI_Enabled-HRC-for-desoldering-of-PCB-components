import pyaudio
import wave
import numpy as np
import librosa

# Audio settings
FORMAT = pyaudio.paInt16
CHANNELS = 2  # Your mic records in Stereo
TARGET_CHANNELS = 1  # Convert to Mono
INPUT_RATE = 44100  # Your mic default sample rate
TARGET_RATE = 16000  # Vosk expects 16kHz
CHUNK = 1024
RECORD_SECONDS = 20
OUTPUT_FILENAME = "sony_mic_16k.wav"

p = pyaudio.PyAudio()

# Use your Sony WH-1000XM4 Bluetooth mic (replace with correct index)
stream = p.open(format=FORMAT, channels=CHANNELS, rate=INPUT_RATE,
                input=True, frames_per_buffer=CHUNK,
                input_device_index=12)  # Change index if needed

print("Recording...")
frames = []

for _ in range(0, int(INPUT_RATE / CHUNK * RECORD_SECONDS)):
    data = stream.read(CHUNK)
    frames.append(data)

print("Recording finished!")

stream.stop_stream()
stream.close()
p.terminate()

# Convert to Mono & 16kHz
print("Processing audio...")
audio_data = np.frombuffer(b"".join(frames), dtype=np.int16)
audio_data = audio_data.reshape((-1, CHANNELS))  # Reshape to Stereo format
audio_mono = np.mean(audio_data, axis=1).astype(np.int16)  # Convert to Mono

# Resample to 16kHz
audio_resampled = librosa.resample(audio_mono.astype(float), orig_sr=INPUT_RATE, target_sr=TARGET_RATE).astype(np.int16)

# Save processed file
with wave.open(OUTPUT_FILENAME, "wb") as wf:
    wf.setnchannels(TARGET_CHANNELS)
    wf.setsampwidth(2)  # 16-bit audio
    wf.setframerate(TARGET_RATE)
    wf.writeframes(audio_resampled.tobytes())

print(f"Audio saved as {OUTPUT_FILENAME}")
