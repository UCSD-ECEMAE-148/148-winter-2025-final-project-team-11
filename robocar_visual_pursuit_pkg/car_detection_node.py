#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
import cv2
import depthai as dai
import numpy as np
from pathlib import Path
import os
import sys
import yaml
from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import SetParametersResult

# Node name and topics
NODE_NAME = 'car_detection_node'
CENTROID_TOPIC_NAME = '/centroid'
BOUNDING_AREA_TOPIC_NAME = '/area'

class CarDetection(Node):
    def __init__(self, config_file=None, model_path=None):
        super().__init__(NODE_NAME, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        # Load config if provided
        if config_file and os.path.exists(config_file):
            self.get_logger().info(f'Loading config from: {config_file}')
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                if config and '/**' in config and 'ros__parameters' in config['/**']:
                    config = config['/**']['ros__parameters']
                
            # Extract parameters from config
            confidence_threshold = config.get('confidence_threshold', 0.5)
            camera_centerline = config.get('camera_centerline', 0.4)
            error_threshold = config.get('error_threshold', 0.15)
            iou_threshold = config.get('iou_threshold', 0.5)
            max_lost_frames = config.get('max_lost_frames', 10)
            if model_path is None:
                model_path = config.get('model_path', '/home/projects/ros2_ws/src/robocar_visual_pursuit_pkg/models/v8-weights_openvino_2022.1_5shave-fast.blob')
        else:
            # Default values
            confidence_threshold = 0.5
            camera_centerline = 0.5
            error_threshold = 0.15
            iou_threshold = 0.5
            max_lost_frames = 10
            if model_path is None:
                model_path = '/home/projects/ros2_ws/src/robocar_visual_pursuit_pkg/models/v8-weights_openvino_2022.1_5shave-fast.blob'
        
        # Publisher for centroid error
        self.centroid_error_publisher = self.create_publisher(Float32, CENTROID_TOPIC_NAME, 10)
        self.centroid_error = Float32()
        self.bounding_area_publisher = self.create_publisher(Float32, BOUNDING_AREA_TOPIC_NAME, 10)
        self.bounding_area = Float32()
        
        # Declare parameters
        self.declare_parameter('model_path', model_path)
        self.declare_parameter('confidence_threshold', confidence_threshold)
        self.declare_parameter('camera_centerline', camera_centerline)
        self.declare_parameter('error_threshold', error_threshold)
        self.declare_parameter('iou_threshold', iou_threshold)
        self.declare_parameter('debug_cv', 1)
        self.declare_parameter('max_lost_frames', max_lost_frames)
        
        # Add parameter callback
        self.add_on_set_parameters_callback(self.parameters_callback)
        
        # Get parameters
        self.model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.error_threshold = self.get_parameter('error_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.debug_cv = self.get_parameter('debug_cv').value
        self.max_lost_frames = self.get_parameter('max_lost_frames').value
        
        # Variables for tracking
        self.last_error = 0.0
        self.last_bounding_area = 0.0
        self.lost_frames = 0
        
        # Check if model file exists
        if not Path(self.model_path).exists():
            self.get_logger().error(f"Model file not found: {self.model_path}")
            self.get_logger().error("Please provide a valid model path")
            return
        
        self.get_logger().info(
            f'\nModel path: {self.model_path}'
            f'\nConfidence threshold: {self.confidence_threshold}'
            f'\nCamera centerline: {self.camera_centerline}'
            f'\nIoU threshold: {self.iou_threshold}'
            f'\nDebug CV: {self.debug_cv}'
            f'\nMax lost frames: {self.max_lost_frames}')
        
        # Initialize DepthAI pipeline
        self.initialize_pipeline()
        
        # Timer for processing frames
        self.timer = self.create_timer(0.033, self.process_frame)  # ~30Hz
    
    def parameters_callback(self, params):
        """Handle parameter updates"""
        self.get_logger().info(f'Received parameter update request: {[p.name for p in params]}')
        
        try:
            for param in params:
                self.get_logger().info(f'Processing parameter: {param.name} = {param.value}')
                
                # Update local variables based on parameter name
                if param.name == 'confidence_threshold':
                    self.confidence_threshold = param.value
                elif param.name == 'camera_centerline':
                    self.camera_centerline = param.value
                elif param.name == 'error_threshold':
                    self.error_threshold = param.value
                elif param.name == 'iou_threshold':
                    self.iou_threshold = param.value
                elif param.name == 'max_lost_frames':
                    self.max_lost_frames = param.value
                elif param.name == 'model_path':
                    if param.value != self.model_path:
                        self.get_logger().warn('Model path changed - requires node restart')
                
                # Log the updated value
                self.get_logger().info(f'Updated {param.name} to {param.value}')
            
            self.get_logger().info('Parameters updated successfully')
            return SetParametersResult(successful=True)
            
        except Exception as e:
            self.get_logger().error(f'Error updating parameters: {e}')
            return SetParametersResult(successful=False, reason=str(e))
    
    def initialize_pipeline(self):
        self.get_logger().info("Initializing DepthAI pipeline...")
        
        # Create pipeline
        self.pipeline = dai.Pipeline()
        
        # Define sources & outputs
        self.camRgb = self.pipeline.create(dai.node.ColorCamera)
        self.detectionNetwork = self.pipeline.create(dai.node.YoloDetectionNetwork)
        self.xoutRgb = self.pipeline.create(dai.node.XLinkOut)
        self.nnOut = self.pipeline.create(dai.node.XLinkOut)
        
        # Stream names
        self.xoutRgb.setStreamName("rgb")
        self.nnOut.setStreamName("nn")
        
        # Camera properties
        self.camRgb.setPreviewSize(640, 640)  # Must match model input
        self.camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        self.camRgb.setInterleaved(False)
        self.camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        self.camRgb.setFps(30)
        
        # Configure neural network
        self.detectionNetwork.setBlobPath(self.model_path)
        self.detectionNetwork.setConfidenceThreshold(self.confidence_threshold)
        self.detectionNetwork.setNumClasses(1)  # Only cars
        self.detectionNetwork.setCoordinateSize(4)
        self.detectionNetwork.setIouThreshold(self.iou_threshold)
        self.detectionNetwork.setNumInferenceThreads(2)
        self.detectionNetwork.input.setBlocking(False)
        
        # Link nodes
        self.camRgb.preview.link(self.detectionNetwork.input)
        self.detectionNetwork.passthrough.link(self.xoutRgb.input)
        self.detectionNetwork.out.link(self.nnOut.input)
        
        try:
            # Connect to device and start pipeline
            self.device = dai.Device(self.pipeline)
            
            # Output queues
            self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            self.qDet = self.device.getOutputQueue(name="nn", maxSize=4, blocking=False)
            
            self.get_logger().info(f"Connected to {self.device.getDeviceName()}")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize DepthAI device: {e}")
            raise
    
    def process_frame(self):
        # Update parameters that might have changed (from calibration node)
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.camera_centerline = self.get_parameter('camera_centerline').value
        self.debug_cv = self.get_parameter('debug_cv').value
        
        try:
            # Get data
            inRgb = self.qRgb.tryGet()
            inDet = self.qDet.tryGet()
            
            if inRgb is None or inDet is None:
                return
            
            frame = inRgb.getCvFrame()
            detections = inDet.detections
            
            height, width = frame.shape[:2]
            
            # Calculate center position from parameters
            centerX = int(width * self.camera_centerline)
            
            # Find best detection (highest confidence)
            best_detection = None
            best_confidence = 0
            
            for det in detections:
                if det.confidence > best_confidence:
                    best_detection = det
                    best_confidence = det.confidence
            
            if best_detection:
                # Reset lost frames counter
                self.lost_frames = 0
                
                # Denormalize bounding box
                x1 = int(best_detection.xmin * width)
                y1 = int(best_detection.ymin * height)
                x2 = int(best_detection.xmax * width)
                y2 = int(best_detection.ymax * height)
                
                
                # Calculate center of bounding box
                x_center = (x1 + x2) // 2
                y_center = (y1 + y2) // 2
                
                # Calculate error from center (normalized between -1 and 1)
                # This matches the approach used in the original lane detection
                center_error = float((x_center - centerX) / centerX)
                
                # Publish centroid error
                self.centroid_error.data = center_error
                self.centroid_error_publisher.publish(self.centroid_error)
                self.bounding_area.data = float(x2 - x1 + y2 - y1)
                self.bounding_area_publisher.publish(self.bounding_area)
                
                # Update last error
                
                self.last_error = center_error
                self.last_bounding_area = self.bounding_area.data
                
                # self.get_logger().info(f"Target detected: error={center_error:.3f}")
            else:
                # Increment lost frames counter
                self.lost_frames += 1
                
                if self.lost_frames <= self.max_lost_frames:
                    # Use last known error when target is temporarily lost
                    self.centroid_error.data = float(self.last_error)
                    self.centroid_error_publisher.publish(self.centroid_error)
                    self.bounding_area.data = float(self.last_bounding_area)
                    self.bounding_area_publisher.publish(self.bounding_area)
                    self.get_logger().info(f"Target lost, using last error: {self.last_error:.3f}")
                else:
                    # Target lost for too long, reset error
                    self.centroid_error.data = 0.0
                    self.centroid_error_publisher.publish(self.centroid_error)
                    self.bounding_area.data = float(1000)
                    self.bounding_area_publisher.publish(self.bounding_area)
                    self.get_logger().info("No target detected, zeroing error")
            
            # Debug visualization if enabled
            if self.debug_cv:
                self.visualize_debug(frame, detections, centerX, width, height)
                
        except Exception as e:
            self.get_logger().error(f"Error processing frame: {e}")
    
    def visualize_debug(self, frame, detections, centerX, width, height):
        display_frame = frame.copy()
        
        # Draw vertical center line (similar to lane detection visualization)
        cv2.line(display_frame, (centerX, 0), (centerX, height), (255, 255, 255), 1)
        
        # Calculate threshold lines (similar to the lane detection visualization)
        threshold_width = int(self.error_threshold * width/2)
        start_point_thresh_pos_x = centerX - threshold_width
        start_point_thresh_neg_x = centerX + threshold_width
        
        # Draw threshold lines
        cv2.line(display_frame, (start_point_thresh_pos_x, 0), 
                 (start_point_thresh_pos_x, height), (0, 0, 255), 2)
        cv2.line(display_frame, (start_point_thresh_neg_x, 0), 
                 (start_point_thresh_neg_x, height), (0, 0, 255), 2)
        
        # Find best detection (highest confidence)
        best_detection = None
        best_confidence = 0
        for det in detections:
            if det.confidence > best_confidence:
                best_detection = det
                best_confidence = det.confidence
        
        # Process all detections
        for det in detections:
            # Denormalize bounding box
            x1 = int(det.xmin * width)
            y1 = int(det.ymin * height)
            x2 = int(det.xmax * width)
            y2 = int(det.ymax * height)
            
            # Calculate center of bounding box
            x_center = (x1 + x2) // 2
            y_center = (y1 + y2) // 2
            
            # Calculate error from center
            center_error = float((x_center - centerX) / centerX)
            
            # Determine if this is the best detection
            is_best = (det == best_detection)
            
            # Determine box color - orange for best, blue for others
            boxColor = (0, 165, 255) if is_best else (255, 0, 0)
            
            # Draw bounding box
            cv2.rectangle(display_frame, (x1, y1), (x2, y2), boxColor, 2)
            
            # Draw center-to-center line
            cv2.line(display_frame, (centerX, y_center), (x_center, y_center), boxColor, 1)
            
            # Add object label and confidence
            label = f"car {int(det.confidence * 100)}%"
            cv2.putText(display_frame, label, (x1 + 5, y1 + 15), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Add error for best detection
            if is_best:
                error_text = f"Err: {center_error:.2f}"
                cv2.putText(display_frame, error_text, (x1 + 5, y1 + 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, boxColor, 1)
                
                # Draw crosshair for best target
                crosshair_size = 15
                cv2.line(display_frame, (x_center - crosshair_size, y_center), 
                         (x_center + crosshair_size, y_center), boxColor, 2)
                cv2.line(display_frame, (x_center, y_center - crosshair_size), 
                         (x_center, y_center + crosshair_size), boxColor, 2)
        
        # Show status
        status = "TRACKING" if best_detection else f"NO TARGET ({self.lost_frames}/{self.max_lost_frames})"
        status_color = (0, 255, 0) if best_detection else (0, 0, 255)
        cv2.putText(display_frame, status, (10, 25), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        
        # Show display frame
        cv2.imshow("Car Detection", display_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    config_file = None
    model_path = None
    
    if len(sys.argv) > 1:
        config_file = sys.argv[1]
    if len(sys.argv) > 2:
        model_path = sys.argv[2]
    
    node = CarDetection(config_file, model_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
