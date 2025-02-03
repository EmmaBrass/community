import numpy as np
from pydub import AudioSegment
from pydub.generators import WhiteNoise, Sine
import os

def generate_static_sounds(duration):
        """
        Generate varied electronic static audio files with white noise and modulation.

        :param output_dir: Directory to save the generated audio files.
        :param duration: Duration of each audio file in seconds.

        :returns: The audi data as a series of bytes.
        """
        # Generate base white noise
        noise = WhiteNoise().to_audio_segment(duration * 1000)  # Duration in milliseconds

        # Add random amplitude modulation (low-frequency sine wave)
        mod_freq = np.random.uniform(0.1, 10)  # Modulation frequency in Hz
        mod_depth = np.random.uniform(-15, -5)  # Modulation depth in dB
        sine_wave = Sine(mod_freq).to_audio_segment(duration * 1000).apply_gain(mod_depth)
        modulated_noise = noise.overlay(sine_wave)

        # Apply random low-pass and high-pass filters
        low_cutoff = np.random.uniform(500, 5000)  # Low-pass cutoff frequency
        high_cutoff = np.random.uniform(50, 400)   # High-pass cutoff frequency
        filtered_noise = modulated_noise.low_pass_filter(low_cutoff).high_pass_filter(high_cutoff)

        # Add slight random gain for variation
        final_noise = filtered_noise.apply_gain(np.random.uniform(-10, 15))

        # Export to file
        wav_file = "static_sound.wav"
        final_noise.export(wav_file, format="wav")
        print(f"Generated: {wav_file}")

        # Convert audio to bytes
        with open(wav_file, 'rb') as f:
            audio_data = f.read()
            audio_uint8 = list(audio_data)
            
        return audio_uint8

generate_static_sounds(5)