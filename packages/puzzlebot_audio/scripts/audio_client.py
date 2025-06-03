#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import AudioInfo
from scipy.io import wavfile
from std_msgs.msg import Float32MultiArray
import sounddevice as sd
import soundfile as sf
import numpy as np

import os
import ament_index_python.packages
package_prefix = ament_index_python.packages.get_package_prefix('puzzlebot_audio')
codebook_path = os.path.join(package_prefix, 'share', 'puzzlebot_audio', 'data')

class AudioClient(Node):
    def __init__(self, audio):
        super().__init__('audio_client')
        self.client = self.create_client(AudioInfo, 'audio_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AudioInfo.Request()
        self.audio = audio.flatten()

    # def load_audio(self, audio_path):
    #     fs, signal = wavfile.read(audio_path)

    #     return signal
    
    
    
    def eliminate_noise(self, audio, fs, threshold, padding_ms=100):
        # Placeholder for noise elimination logic
        # This function should implement the noise elimination algorithm
        # For now, we just return the original audio

        frame_len = int(0.02 * fs)
        hop_len = int(0.008 * fs)
        clean_audio = []

        audio = audio.astype(np.float32)
        audio = audio / (np.max(np.abs(audio)) + 1e-8)

        num_frames = int((len(audio) - frame_len) / hop_len) + 1
        energy = np.zeros(num_frames)
        
        for i in range(num_frames):
            start = (i) * hop_len
            frame = audio[start:start + frame_len]
            energy[i] = np.sum(frame ** 2) / frame_len         
        
        energy_thresh = threshold * np.max(energy)
        voice_flags = (energy > energy_thresh)
        indices = np.where(voice_flags)[0]

        if len(indices) == 0:
            self.get_logger().info('Audio is empty, do it again')
            return False, audio
        
        padding_frames = int(fs * padding_ms / 1000)
        start_sample = max(0, indices[0] * hop_len - padding_frames)
        end_sample = min(len(audio), indices[-1] * hop_len + frame_len + padding_frames)
        return True, audio[start_sample:end_sample]

        

    def send_request(self):
        # for filename in os.listdir(codebook_path):
        #     if filename.endswith('.wav'):
        #         audioPath = os.path.join(codebook_path, filename)
        #         audio = self.load_audio(audioPath)
        flag, clean_audio = self.eliminate_noise(self.audio, fs=16000, threshold=0.01) #0.018
        
        if not flag:
            self.get_logger().warn('No voice detected in the audio.')
            return None
        
        
        
        audio_msg = Float32MultiArray()
        audio_msg.data = clean_audio.astype(float).tolist()

        audio = np.array(audio_msg.data)
        sf.write('audio_debug_ros2_client.wav', audio, 16000)

        self.request.audio = audio_msg  # Replace with actual audio data
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def record_audio(duration=3, fs=16000):
    sd.sleep(500)
    print('Recording')
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()
    print(f'Audio recorded {audio}')
    sf.write('audio_original_ros2_client.wav', audio, 16000)
    return audio

def main(args=None):
    audio = record_audio(duration=5, fs=16000)
    rclpy.init(args=args)
    audio_client = AudioClient(audio)

    try:
        response = audio_client.send_request()
        print(f'Response received: {response.result}')
        print(f'Codebook name: {response.prediction}')
    except Exception as e:
        print(f'Service call failed: {e}')
    finally:
        audio_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()