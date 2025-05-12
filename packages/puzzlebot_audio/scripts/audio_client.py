#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.srv import AudioInfo
from scipy.io import wavfile
from std_msgs.msg import Float64MultiArray

import os
import ament_index_python.packages
package_prefix = ament_index_python.packages.get_package_prefix('puzzlebot_audio')
codebook_path = os.path.join(package_prefix, 'share', 'puzzlebot_audio', 'data')

class AudioClient(Node):
    def __init__(self):
        super().__init__('audio_client')
        self.client = self.create_client(AudioInfo, 'audio_info')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = AudioInfo.Request()

    def load_audio(self, audio_path):
        fs, signal = wavfile.read(audio_path)

        return signal
    
    def send_request(self):
        for filename in os.listdir(codebook_path):
            if filename.endswith('.wav'):
                audioPath = os.path.join(codebook_path, filename)
                audio = self.load_audio(audioPath)
                
        audio_msg = Float64MultiArray()
        audio_msg.data = audio.astype(float).tolist()

        self.request.audio = audio_msg  # Replace with actual audio data
        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


def main(args=None):
    rclpy.init(args=args)
    audio_client = AudioClient()

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