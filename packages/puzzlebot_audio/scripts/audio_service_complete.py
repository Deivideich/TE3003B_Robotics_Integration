#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import AudioRequest

# Audio libraries
from scipy.signal import lfilter
import numpy as np
import librosa
from scipy.special import logsumexp
import soundfile as sf
import sounddevice as sd

from std_msgs.msg import String
from std_msgs.msg import Bool

from dataclasses import dataclass
import glob

import os
import ament_index_python.packages
package_prefix = ament_index_python.packages.get_package_prefix('puzzlebot_audio')
data_path = os.path.join(package_prefix, 'share', 'puzzlebot_audio', 'data')

class AudioService(Node):

    def __init__(self):
        super().__init__('audio_server')
        self.srv = self.create_service(AudioRequest, 'audio_info', self.audio_info_callback)
        self._default_callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.get_logger().info('Audio service ready.')
        self.audio = None
        self.HMM_data = []
        self.HMM_name = []

        self.init_models()

    def init_models(self):
        hmm_paths = glob.glob(os.path.join(data_path, '*_HMM.npz'))
        centroidsPath = os.path.join(data_path, 'allphonema.npz')

        for hmm in hmm_paths:
            data = np.load(hmm)
            self.HMM_name.append(str(data['name']))
            self.HMM_data.append([data['PI'], data['A'], data['B']])
        
        for model in self.HMM_data:
            self.get_logger().info(f'PI = {model[0]}')
            self.get_logger().info(f'A = {model[1]}')
            self.get_logger().info(f'B = {model[2]}')
        
        dataCentroids = np.load(centroidsPath)
        self.centroids = dataCentroids['centroids']


    def eliminate_noise(self, audio, fs, threshold, padding_ms=100):

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

    def pre_emphasis(self, signal, alpha=0.95):
        b = [1 -alpha]
        a = [1]
        return lfilter(b, a, signal)

    def obtain_observation_sequence(self):
        O = np.zeros(len(self.mfcc_list), dtype=int)
        for t, vec in enumerate(self.mfcc_list):
            distances = [self.euclidean_distance(vec, c) for c in self.centroids]
            idx_minor_distance = np.argmin(distances)
            O[t] = idx_minor_distance
        return O
    
    
    def signal_treatment(self, audio, fs):
        frame_len = int(fs * 0.02)  # With 20 ms, we have a frame length of 320 samples
        hop_len = int(fs * 0.008) #With 8 ms, we have a hop length of 128 samples
        signal = self.pre_emphasis(audio)
        mfccs = librosa.feature.mfcc(y=signal, sr=fs, n_mfcc=12, n_fft=frame_len, hop_length=hop_len)
        mfccs = mfccs.T
        for c in mfccs:
            self.mfcc_list.append(c)
            
    def forward_algorithm(self, PI, A, B, O):
        T = len(O)
        N, _ = A.shape

        log_PI = np.log(PI + 1e-300)
        log_A =  np.log(A + 1e-300)
        log_B =  np.log(B + 1e-300)

        alpha = np.zeros((N, T))

        # Initialization
        
        for i in range(N):
            alpha[i, 0] = log_PI[i] + log_B[i, O[0]]

        for t in range(1, T):
            for i in range(N):
                    alpha[i, t] = logsumexp(alpha[:, t - 1]  + log_A[:, i])  + log_B[i, O[t]]

        forward_prob = logsumexp(alpha[:, T - 1])
        return forward_prob
    

    def euclidean_distance(self, a, b):
        return np.linalg.norm(a - b)

    def testing_loop(self, audio):
        

        self.mfcc_list = []
        self.signal_treatment(audio=audio.flatten(), fs=16000)
        Obs = self.obtain_observation_sequence()


        for i, model in enumerate(self.HMM_data):
            likelihood = self.forward_algorithm(PI=model[0], A=model[1], B=model[2], O=Obs)
            self.get_logger().info(f'Likelihood {self.HMM_name[i]} = {likelihood}')
            if likelihood > self.max_likelihood:
                self.max_likelihood = likelihood
                self.idx_best_model = i

        return self.HMM_name[self.idx_best_model]
    
    def record_audio(self, duration=3, fs=16000):
        try:
            mic_index = 8  # o el correcto
            sd.sleep(1000)
            self.get_logger().info(f"Recording with device: {mic_index}")
            
            audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='float32', device=mic_index)
            sd.wait()

            # Opcional: reducir volumen
            audio = np.clip(audio * 1.0, -1.0, 1.0)

            # Guardar como WAV con calidad
            sf.write('audio_debug_ros2.wav', audio, fs, subtype='PCM_16')
            self.get_logger().info(f"Audio saved: shape={audio.shape}, dtype={audio.dtype}")
            return audio
        except Exception as e:
            self.get_logger().error(f"Error during audio recording: {e}")
            return np.zeros((fs * duration, 1), dtype=np.float32)


    def audio_info_callback(self, request, response):
        # Here you would implement the logic to get the audio information
        # For demonstration purposes, we'll just return a dummy response
        duration = request.duration
        prediction_msg = String()
        success_msg = None
        try:
            self.max_likelihood = -np.inf
            self.idx_best_model = -1
            audio = self.record_audio(duration=duration.data, fs=16000)
            audio = audio.astype(np.float32)
            flag, clean_audio = self.eliminate_noise(audio, fs=16000, threshold=0.01)
            
            sf.write('audio_debug_ros2_cut.wav', clean_audio, 16000)
            best_hmm = self.testing_loop(clean_audio)
            prediction_msg.data = best_hmm
            success_msg = True
            
        except Exception as e:
            self.get_logger().error(f'Error processing audio: {e}')
            prediction_msg.data = 'Error'
            success_msg = False
        
        response.prediction = prediction_msg
        response.result = success_msg
        return response
    
def main(args=None):
    rclpy.init(args=args)
    audio_service = AudioService()
    rclpy.spin(audio_service)
    audio_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()