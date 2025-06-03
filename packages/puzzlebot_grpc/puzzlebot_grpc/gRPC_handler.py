#!/usr/bin/env python3
import signal
import threading
from concurrent import futures

import rclpy
from rclpy.node import Node
from puzzlebot_interfaces.msg import TrailerArray
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import grpc
import sys

sys.path.append('../include')
from puzzlebot_grpc import puzzlebot_grpc_pb2 as pz_proto
from puzzlebot_grpc import puzzlebot_grpc_pb2_grpc as pz_grpc

class PuzzlebotRPCImpl(pz_grpc.PuzzlebotRPCServicer):
    def __init__(self, node):
        self.node = node
        self.trailer_data = {}
        self.compressed_image = None
        print("Initialized gRPC Server")

    def setup(self):
        self.node.declare_parameter("numOfTrailers", 3)
        self.num_of_trailers = 3#self.get_parameter('numOfTrailers').get_parameter_value().integer_value
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.trailer_array_sub = self.node.create_subscription(TrailerArray, '/trailer_array', self.update_trailers_data, qos)
        self.compressed_image_sub = self.node.create_subscription(CompressedImage, '/video_source/compressed', self.update_compressed_image, qos)

    def update_compressed_image(self, msg):
        self.compressed_image = msg.data

    def update_trailers_data(self, msg):
        for i in range(self.num_of_trailers):
            self.trailer_data[msg.ids[i]] = msg.boxes[i]

    def GetTrailersData(self, request, context):
        print("gRPC call from:", context.peer())
        results = pz_proto.TrailersArray()
        
        for key in self.trailer_data.keys():
            results.ids.append(key)
            results.boxes.append(self.trailer_data[key])

        return results

    def GetImage(self, request, context):
        print("gRPC image request from:", context.peer())
        results = pz_proto.CompressedImage()

        if self.compressed_image is None:
            print("No image received yet.")
            return results

        try:
            compressed_image = self.compressed_image.tobytes()
            results.img = compressed_image
        except Exception as e:
            print("Failed to process image:", str(e))

        return results
    
    def GetAudio(self, request, context):
        print("gRPC Audio request from: " + context.peer())

        audio = request.audio

        with open('audio.wav', 'wb') as f:
            f.write(request.audio)

terminate = threading.Event()

def terminate_server(signum, frame):
    print("Got signal {}, {}".format(signum, frame))
    rclpy.shutdown()
    terminate.set()

def main(args=None):
    print("---- ROS-gRPC Wrapper ----")
    signal.signal(signal.SIGINT, terminate_server)

    rclpy.init(args=args)
    node = rclpy.create_node('object_position_wrapper')
    print("ROS Node started")

    service = PuzzlebotRPCImpl(node)
    service.setup()
    
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    pz_grpc.add_PuzzlebotRPCServicer_to_server(service, server)  # Use correct name
    server.add_insecure_port('[::]:7042')
    server.start()
    print("gRPC Server listening on [::]:7042")

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    terminate.wait()
    print("Shutting down gRPC server...")
    server.stop(1).wait()
    print("Destroying ROS node...")
    node.destroy_node()
    rclpy.shutdown()
    print("Exited cleanly")

if __name__ == '__main__':
    main()
