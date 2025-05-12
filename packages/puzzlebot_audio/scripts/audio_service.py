#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import AudioInfo

# Audio libraries
from scipy.signal import lfilter
from scipy.signal.windows import hamming
import numpy as np
import librosa
from spectrum import poly2lsf
from spectrum import lsf2poly

from std_msgs.msg import String
from std_msgs.msg import Bool

from dataclasses import dataclass

import os
import ament_index_python.packages
package_prefix = ament_index_python.packages.get_package_prefix('puzzlebot_audio')
codebook_path = os.path.join(package_prefix, 'share', 'puzzlebot_audio', 'data')

@dataclass
class Codebook:
    name: str
    codebook: np.ndarray

class AudioService(Node):

    def __init__(self):
        super().__init__('audio_server')
        self.srv = self.create_service(AudioInfo, 'audio_info', self.audio_info_callback)
        self.get_logger().info('Audio service ready.')
        self.audio = None
        self.codebooks = []
        self.init_codebooks()


    def init_codebooks(self):
        for filename in os.listdir(codebook_path):
            if filename.endswith('.npz'):
                codebook = np.load(os.path.join(codebook_path, filename))
                c = Codebook(name=str(codebook['name']), codebook=codebook['codebook'])
                self.codebooks.append(c)
                # self.get_logger().info(f'Codebook loaded: {filename}')

    def pre_emphasis(self, signal, alpha=0.95):
        return lfilter([1, -alpha], [1], signal)

    def framming_hamming(self, signal, frame_length=320, frame_step=128):
        frames = []
        window = hamming(frame_length, sym=False)

        for start in range(0, len(signal) - frame_length + 1, frame_step):
            end = start + frame_length
            frame = signal[start:end] * window
            frames.append(frame)
        return np.array(frames)
    
    def extract_lpcs(self, frames, order=12):
        return librosa.lpc(frames, order=order)

    def convert_to_lpc(self, lsf):
        return lsf2poly(lsf)
    
    def convert_to_lsf(self, lpc):
        return poly2lsf(lpc)
    
    def signal_treatment(self, audio, word_lpcs, word_frames, word_lsf):
        audio = self.pre_emphasis(audio)
        # self.get_logger().info('Pre-emphasis completed.')
        frames = self.framming_hamming(audio)
        # self.get_logger().info('Framing completed.')
        word_frames.append(frames)
        # self.get_logger().info('Windowing completed.')

        for window in frames:
            # self.get_logger().info('Extracting LPCs...')
            # self.get_logger().info('frames: {}'.format(window))
            lpc = self.extract_lpcs(window)
            # self.get_logger().info('LPCs extracted.')
            lsf = self.convert_to_lsf(lpc)
            # self.get_logger().info('LSFs extracted.')
            word_frames.append(window)
            word_lpcs.append(lpc)
            word_lsf.append(lsf)

        return word_frames, word_lpcs, word_lsf

    def euclidean_distance(self, lsf1, lsf2):
        return np.linalg.norm(lsf1 - lsf2)

    def testing_loop(self, audio):
        word_frames = []
        word_lpcs = []
        word_lsf = []
        min_distance_global = np.inf

        # self.get_logger().info('audio shape: {}'.format(audio.shape))

        # self.get_logger().info('Starting testing loop...')
        word_frames, word_lpcs, word_lsf = self.signal_treatment(audio, word_lpcs, word_frames, word_lsf)

        # self.get_logger().info('Signal treatment completed.')
        for codebook in self.codebooks:
            total_dist = 0
            for j, lsf in enumerate(word_lsf):
                dist = [self.euclidean_distance(lsf, codebook.codebook[i]) for i in range(len(codebook.codebook))]
                total_dist += np.min(dist)
            if total_dist < min_distance_global:
                min_distance_global = total_dist
                best_codebook = codebook.name
        # self.get_logger().info(f'Best codebook: {best_codebook} with distance: {min_distance_global}')
        return best_codebook


    def audio_info_callback(self, request, response):
        # Here you would implement the logic to get the audio information
        # For demonstration purposes, we'll just return a dummy response
        audio = request.audio
        prediction_msg = String()
        success_msg = None
        try:
            audio = np.array(audio.data)
            best_codebook = self.testing_loop(audio)
            prediction_msg.data = best_codebook
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