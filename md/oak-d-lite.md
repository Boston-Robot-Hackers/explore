# Oak-D-Lite Camera Integration

## Key Capabilities

### Hardware Specifications

**RGB Camera:**
- 13MP IMX214 sensor with 4208x3120 resolution
- 81° DFOV / 69° HFOV / 54° VFOV
- Available in Auto-Focus (AF) and Fixed-Focus (FF) variants
  - Auto-Focus: Recommended for close objects between 8cm to 50cm
  - Fixed-Focus: Recommended for high-vibration applications (drones, lawnmowers, vehicles)

**Stereo Cameras:**
- 640×480 resolution monochrome cameras
- 7.5cm baseline between left and right cameras
- 300,000-point stereo depth capability
- Ideal depth range: 40cm - 8m

**Processing Power:**
- RVC2 architecture with 4 TOPS total processing (1.4 TOPS for AI)
- Intel Movidius Myriad-X VPU as vision engine
- Real-time neural network inferencing for object detection and semantic segmentation
- On-device AI inference (no need to stream to host for processing)

**Video Encoding:**
- H.264, H.265, MJPEG encoding
- 4K/30FPS and 1080P/60FPS support

**Sensors & Connectivity:**
- Integrated BMI270 6-axis IMU (Inertial Measurement Unit)
- USB-C connectivity (USB 3.2 Gen1, 5Gbps)
- Power consumption: Up to 5W

**Form Factor:**
- Compact housing: 28×91×18mm
- Lighter and more affordable than OAK-D standard

### AI & Depth Capabilities

**Spatial AI:**
- Combines depth perception with AI object detection
- Provides 3D spatial coordinates (X, Y, Z) of detected objects
- Real-time depth map generation from stereo cameras

**On-Device Processing:**
- Runs neural networks directly on Myriad-X VPU
- Reduces latency and bandwidth requirements
- No GPU needed on host computer
- Can run multiple neural networks simultaneously

## Libraries & Packages

### DepthAI SDK (Primary)

**DepthAI** is the official Python library from Luxonis for Oak cameras.

**Key Features:**
- Python API for camera control and configuration
- Built-in pipeline system for video, depth, and AI processing
- Direct access to RGB, depth, and mono camera streams
- Neural network inference management
- Device management and configuration

**Installation:**
```bash
pip install depthai
```

**Model Zoo:**
- **DepthAI Model Zoo**: 200+ pre-trained models available
- **OpenVINO Open Model Zoo**: Additional models compatible with Oak devices
- Model categories:
  - Object Detection (MobileNet-SSD, YOLO variants)
  - Segmentation
  - Monocular Depth Estimation
  - Facial Landmark Detection
  - Text Detection
  - Classification
  - Pose Estimation

#### AI Model Categories Explained

**1. Object Detection (MobileNet-SSD, YOLO variants)**

*What it does:* Identifies and locates multiple objects in an image by drawing bounding boxes around them and classifying what they are.

*How it works:*
- Analyzes the image and identifies regions containing objects
- Draws rectangular bounding boxes around each detected object
- Assigns a class label (e.g., "person", "bottle", "chair")
- Provides confidence score (0.0-1.0) for each detection
- With spatial detection enabled: Provides 3D coordinates (X, Y, Z) of each object

*Output:*
- Bounding box: (x, y, width, height) in pixels
- Class: "bottle", "person", "car", etc.
- Confidence: 0.85 (85% confident)
- Spatial coordinates: (X: 1.2m, Y: -0.3m, Z: 2.5m) from camera

*Use cases for Dome2:*
- Detecting soda cans, bottles, obstacles
- Identifying furniture, people, pets
- Finding specific objects during exploration
- Navigation hazard detection

*Popular models:*
- MobileNet-SSD: Fast, efficient, good for real-time on edge devices
- YOLO (You Only Look Once): More accurate, slightly slower
- EfficientDet: Best accuracy, higher computational cost

**2. Segmentation**

*What it does:* Labels every pixel in the image with what object or category it belongs to, creating a detailed mask of the scene.

*Types:*
- **Semantic Segmentation**: Labels pixels by category (all people are one color, all chairs another)
- **Instance Segmentation**: Separates individual instances (person #1 vs person #2)

*How it works:*
- Processes entire image pixel by pixel
- Assigns each pixel to a category (person, floor, wall, furniture, etc.)
- Creates colored mask overlay showing object boundaries
- Unlike object detection, it shows exact shape, not just bounding box

*Output:*
- Segmentation mask: Image where each pixel color represents a class
- Example: Blue = floor, Green = wall, Red = furniture, Yellow = person

*Use cases for Dome2:*
- Identifying navigable floor area vs obstacles
- Understanding room layout (walls, doors, open space)
- Detecting small objects that might not trigger object detection
- Creating semantic maps (this area is "kitchen", this is "hallway")

*Popular models:*
- DeepLabV3: Good accuracy, moderate speed
- U-Net: Medical imaging, but works for robotics
- Mask R-CNN: Instance segmentation

**3. Monocular Depth Estimation**

*What it does:* Estimates distance to every point in a scene using only a single camera image (no stereo required).

*How it works:*
- Uses deep learning trained on millions of images to understand depth cues
- Analyzes visual cues like size, perspective, occlusion, texture gradients
- Produces depth map showing estimated distance for every pixel
- Different from Oak-D-Lite's stereo depth (which uses two cameras)

*Output:*
- Depth map: Grayscale image where brightness indicates distance
- Closer objects = darker, farther objects = lighter (or vice versa)
- Per-pixel depth estimates in meters

*Use cases for Dome2:*
- **NOT needed for Dome2** - Oak-D-Lite already has stereo depth (more accurate)
- Useful if you only had the RGB camera without stereo
- Can provide depth in areas where stereo fails (textureless surfaces, far distances)
- Fusion with stereo depth for improved accuracy

*Popular models:*
- MiDaS: General purpose monocular depth
- DPT (Dense Prediction Transformer): State-of-the-art accuracy

**4. Facial Landmark Detection**

*What it does:* Detects faces and identifies specific points on the face like eyes, nose, mouth, jawline.

*How it works:*
- First detects faces in the image (bounding box)
- Then identifies key facial points (landmarks)
- Typically detects 5, 68, or 468 landmarks depending on model
- Tracks facial features across frames

*Output:*
- Face bounding box
- Landmark points: (x, y) coordinates for each feature
  - 5-point: 2 eyes, nose tip, 2 mouth corners
  - 68-point: Full face outline, eyebrows, eyes, nose, mouth, jawline
  - 468-point: Very detailed mesh (MediaPipe)

*Use cases for Dome2:*
- **Limited use** for autonomous exploration robot
- Could detect people and track where they're looking
- Human-robot interaction (is person looking at robot?)
- Safety: Stop if person is in the way

*Popular models:*
- MediaPipe Face Mesh: 468 landmarks, very detailed
- Dlib 68-point: Classic facial landmarks
- BlazeFace: Fast face detection

**5. Text Detection**

*What it does:* Finds and reads text in images (OCR - Optical Character Recognition).

*Two stages:*
- **Detection**: Finds where text appears in the image (bounding boxes)
- **Recognition**: Reads the actual text content

*How it works:*
- Scans image for text-like patterns
- Draws bounding boxes around text regions
- Extracts text regions and recognizes characters
- Handles various fonts, sizes, orientations

*Output:*
- Text bounding boxes: (x, y, width, height)
- Recognized text: "EXIT", "Room 101", "Coca-Cola"
- Confidence scores
- Text orientation/rotation

*Use cases for Dome2:*
- Reading room labels/signs ("Kitchen", "Bathroom")
- Identifying labeled objects ("Recycle", "Emergency Exit")
- Reading product labels on detected objects
- Navigation using text-based landmarks

*Popular models:*
- EAST (Efficient and Accurate Scene Text): Fast detection
- CRAFT (Character Region Awareness): Accurate detection
- Tesseract: Classic OCR recognition
- PaddleOCR: End-to-end detection + recognition

**6. Classification**

*What it does:* Assigns a single label to an entire image (or region). Unlike object detection, it doesn't locate objects, just identifies what category the image belongs to.

*How it works:*
- Takes entire image (or cropped region) as input
- Processes through neural network
- Outputs probability distribution over classes
- Returns top-K predictions with confidence scores

*Output:*
- Class label: "living_room", "kitchen", "bedroom"
- Confidence: 0.92 (92% certain)
- Top-5 predictions with scores

*Use cases for Dome2:*
- Room type classification ("This is a kitchen", "This is a bedroom")
- Scene understanding (indoor vs outdoor)
- Object classification after detection (what type of bottle?)
- Surface classification (carpet, hardwood, tile)

*Common workflow:*
1. Object detection finds a bottle (bounding box)
2. Classification model identifies: "Coca-Cola can" vs "water bottle" vs "wine bottle"

*Popular models:*
- ResNet: Standard classification backbone
- EfficientNet: Best accuracy/speed tradeoff
- MobileNet: Fast, efficient for edge devices
- VGG: Classic, still effective

**7. Pose Estimation**

*What it does:* Detects human bodies and identifies locations of joints/keypoints (shoulders, elbows, knees, etc.).

*Types:*
- **2D Pose**: Keypoint (x, y) coordinates in image
- **3D Pose**: Keypoint (x, y, z) coordinates in 3D space

*How it works:*
- Detects people in the image
- Identifies body keypoints (typically 17-33 points)
- Connects keypoints to form skeleton
- Tracks pose across frames

*Output:*
- Keypoint coordinates: (x, y) for each joint
- Common keypoints (17-point COCO format):
  - Head: nose, eyes, ears
  - Upper body: shoulders, elbows, wrists
  - Lower body: hips, knees, ankles
- Skeleton connections showing body structure
- Confidence per keypoint

*Use cases for Dome2:*
- **Limited use** for autonomous exploration
- Detecting people and their posture/activity
- Safety: Is person sitting, standing, fallen?
- Human activity recognition (person is waving, walking, sitting)
- Human-robot interaction (gesture recognition)

*Popular models:*
- OpenPose: 25-point skeleton, very accurate
- MediaPipe Pose: 33-point, fast, runs on mobile
- PoseNet: Lightweight, good for real-time
- BlazePose: Google's fast pose estimation

#### Model Selection for Dome2

**Primary need: Object Detection**
- MobileNet-SSD for soda can/bottle detection
- Fast, efficient, spatial coordinates included
- Perfect for Raspberry Pi 5

**Secondary useful models:**
- **Segmentation**: Floor/obstacle identification for navigation
- **Text Detection**: Reading room labels or object labels
- **Classification**: Room type identification

**Not needed:**
- Monocular Depth: Already have stereo depth
- Facial Landmarks: Not needed for exploration
- Pose Estimation: Not needed for exploration

**Popular Pre-trained Models:**
- MobileNetv2 SSD (default) - PASCAL 2007 VOC classes including "bottle"
- Tiny YOLOv3/v4
- YOLOv5/v6/v7
- EfficientDet

### Custom Model Training

**Training Resources:**
- **depthai-ml-training**: Official GitHub repo with Google Colab notebooks
- **Roboflow Integration**: Train and deploy custom models with simplified workflow
- **Tutorial notebooks**: Step-by-step training for MobileNet SSD v2 and Tiny YOLO

**Training Process:**
1. Collect and label dataset (Roboflow, CVAT, LabelImg)
2. Use Colab notebooks for training (free GPU instances)
3. Convert model to OpenVINO format (.blob)
4. Deploy to Oak-D-Lite

## ROS2 Integration

### Official Package: depthai-ros

**Repository:** [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros)

**Status:** Actively maintained (last update: January 2026)

**Supported ROS2 Distributions:**
- ROS2 Humble
- ROS2 Iron
- ROS2 Jazzy (likely compatible, verify)

**Installation:**
```bash
# For ROS2 Humble/Iron
sudo apt install ros-<distro>-depthai-ros

# Or build from source for latest features
cd ~/ros2_ws/src
git clone https://github.com/luxonis/depthai-ros.git
cd ~/ros2_ws
colcon build --packages-select depthai-ros
```

### Key ROS2 Features

**Published Topics:**
- `/color/image` - RGB camera stream (sensor_msgs/Image)
- `/stereo/depth` - Depth map (sensor_msgs/Image)
- `/stereo/points` - Point cloud (sensor_msgs/PointCloud2)
- `/nn/detections` - Object detections with 3D coordinates (custom message)
- `/left/image` - Left mono camera
- `/right/image` - Right mono camera
- `/imu/data` - IMU data (sensor_msgs/Imu)

**Launch Files:**
```bash
ros2 launch depthai_ros_driver camera.launch.py
```

**Composable Nodes:**
- Detection2DOverlay - Visualize detections on RGB image
- SegmentationOverlay - Visualize segmentation masks
- Custom filters for multi-topic data fusion

### ROS2 Driver Package: depthai_ros_driver

**Features:**
- Composable node architecture for efficient processing
- Parameter-based configuration
- Multiple camera mode support (RGB, depth, stereo, mono)
- Neural network configuration via parameters
- TF frame publishing for spatial transforms

**Configuration:**
- Launch parameters for camera resolution, FPS, neural network selection
- Runtime parameter updates via ROS2 parameter server
- YAML configuration file support

### Alternative Drivers

**novelte_depthai_driver:** Community-maintained ROS2 driver with additional features
- GitHub: [Novelte/novelte_depthai_driver](https://github.com/Novelte/novelte_depthai_driver)

**Serafadam/depthai_ros_driver:** Another community option
- GitHub: [Serafadam/depthai_ros_driver](https://github.com/Serafadam/depthai_ros_driver)

## Detecting Soda Cans on a Moving Robot

### Approach Overview

For Dome2 autonomous exploration robot, detecting soda cans involves:
1. Running object detection on Oak-D-Lite's Myriad-X VPU
2. Publishing detections with 3D spatial coordinates to ROS2
3. Integrating with robot's map and localization

### Option 1: Pre-trained Model (Quick Start)

**Use MobileNet-SSD with COCO or VOC classes:**

The default PASCAL VOC classes include "bottle" which covers soda cans reasonably well.

**Pros:**
- Zero training required
- Works immediately
- Good general object detection

**Cons:**
- Generic "bottle" class (not specific to soda cans)
- May miss brand/type distinctions
- Lower accuracy for specific can types

**ROS2 Integration:**
```python
# Launch with pre-trained model
ros2 launch depthai_ros_driver camera.launch.py \
  nn_type:=mobilenet \
  enable_spatial_detection:=true
```

**Detection Output:**
- Class: "bottle"
- Confidence score: 0.0-1.0
- 2D bounding box: (x, y, width, height)
- 3D spatial coordinates: (X, Y, Z) in meters from camera
- Distance from camera

### Option 2: Custom Trained Model (Best Accuracy)

**Train MobileNet-SSD on custom soda can dataset:**

**Steps:**

1. **Collect Dataset:**
   - Capture images of soda cans in various lighting, angles, backgrounds
   - Include different can types, brands, conditions (crushed, upright, sideways)
   - 500-1000 labeled images recommended
   - Use Roboflow or Label Studio for annotation

2. **Train Model:**
   - Use DepthAI Colab notebook: [Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb](https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb)
   - Or use Roboflow's automated training pipeline
   - Export as OpenVINO .blob file

3. **Deploy to Oak-D-Lite:**
```python
# Load custom model
pipeline = depthai.Pipeline()
nn = pipeline.create(depthai.node.MobileNetSpatialDetectionNetwork)
nn.setBlobPath("/path/to/custom_sodacan_model.blob")
nn.setConfidenceThreshold(0.5)
```

4. **ROS2 Integration:**
```bash
ros2 launch depthai_ros_driver camera.launch.py \
  nn_type:=mobilenet \
  model_path:=/path/to/custom_sodacan_model.blob \
  enable_spatial_detection:=true
```

**Pros:**
- High accuracy for specific use case
- Can detect specific can types/brands
- Optimized for your environment

**Cons:**
- Requires dataset collection and labeling
- Training time (1-2 hours on Colab GPU)
- Ongoing maintenance if environment changes

### Handling Motion (Moving Robot)

**Challenges:**
- Motion blur from robot movement
- Changing viewpoint requires robust detection
- Need to filter false positives

**Solutions:**

**1. Image Quality:**
- Use Fixed-Focus variant (better for vibration)
- Ensure adequate lighting
- Limit robot speed during detection (<0.5 m/s)

**2. Temporal Filtering:**
```python
# Track detections across frames
detection_history = []

def filter_detections(current_detections):
    # Only accept detections seen in 3+ consecutive frames
    confirmed = []
    for det in current_detections:
        if count_in_history(det, detection_history) >= 3:
            confirmed.append(det)
    return confirmed
```

**3. Spatial Filtering:**
```python
# Only detect cans within reasonable range
MIN_DEPTH = 0.4  # 40cm
MAX_DEPTH = 3.0  # 3m

def filter_by_depth(detections):
    return [d for d in detections if MIN_DEPTH < d.spatialCoordinates.z < MAX_DEPTH]
```

**4. Map Integration:**
```python
# Transform detection to map frame using TF
# Subscribe to /tf to get camera->map transform
# Publish detected can positions in map coordinates

def detection_to_map(detection, camera_pose, tf_buffer):
    # Get camera position in map frame
    transform = tf_buffer.lookup_transform('map', 'camera', time)

    # Transform detection spatial coordinates
    can_position_map = transform_point(detection.spatialCoordinates, transform)

    # Publish as marker or custom ObjectDetection message
    return can_position_map
```

### Determining Object Position on Map

**The Challenge:**

When Oak-D-Lite detects a soda can, it provides 3D coordinates relative to the camera (e.g., "2 meters in front, 0.5 meters to the right"). But we need to know where that can is on the map (e.g., "at position X=5.2m, Y=3.1m in the room").

**The Solution: Coordinate Frame Transformations**

This requires transforming coordinates through multiple reference frames using ROS2's TF (Transform) system.

#### Coordinate Frames Involved

**1. Camera Frame (`camera_link`)**
- Origin: Camera's optical center
- X-axis: Right
- Y-axis: Down
- Z-axis: Forward (depth direction)
- Oak-D-Lite provides detections in this frame

**2. Robot Base Frame (`base_link`)**
- Origin: Center of robot
- X-axis: Forward direction
- Y-axis: Left
- Z-axis: Up
- Camera mounted at fixed position/orientation relative to robot

**3. Map Frame (`map`)**
- Origin: Starting point when mapping began (or arbitrary fixed point)
- X-axis: East (or arbitrary direction)
- Y-axis: North (or arbitrary direction)
- Z-axis: Up
- Global reference frame for the entire mapped area

#### The Transformation Chain

To get from camera detection to map position:

```
Object in Camera Frame
    ↓ (camera → base_link transform: fixed, from URDF)
Object in Robot Base Frame
    ↓ (base_link → map transform: changes as robot moves, from SLAM)
Object in Map Frame
```

#### Step-by-Step Process

**Step 1: Oak-D-Lite Detection**

Oak-D-Lite detects a soda can and reports:
```python
detection = {
    'class': 'bottle',
    'confidence': 0.87,
    'spatialCoordinates': {
        'x': 0.3,   # 0.3m to the right of camera
        'y': 0.1,   # 0.1m below camera center
        'z': 2.0    # 2.0m in front of camera
    }
}
```

This is in the `camera_link` frame.

**Step 2: Get Robot's Current Position on Map**

SLAM Toolbox (or AMCL) continuously publishes the robot's position on the map via TF.

At the moment of detection, the robot might be at:
- Map position: (X=5.0m, Y=3.0m)
- Orientation: 45° (facing northeast)

**Step 3: Get Camera-to-Robot Transform**

The camera is mounted on the robot at a fixed position (from robot URDF):
- 0.15m forward from robot center
- 0.0m sideways
- 0.25m up from robot base
- Facing forward (same direction as robot)

This transform is static and published by `robot_state_publisher`.

**Step 4: Transform Detection to Map Frame**

Using ROS2 TF, we combine all transforms:

```python
import rclpy
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import PointStamped

class ObjectDetectionMapper:
    def __init__(self):
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def detection_callback(self, detection_msg):
        # Step 1: Create point in camera frame
        point_camera = PointStamped()
        point_camera.header.frame_id = 'camera_link'
        point_camera.header.stamp = self.get_clock().now().to_msg()
        point_camera.point.x = detection_msg.spatialCoordinates.x
        point_camera.point.y = detection_msg.spatialCoordinates.y
        point_camera.point.z = detection_msg.spatialCoordinates.z

        try:
            # Step 2: Transform to map frame
            # TF automatically chains camera→base_link→map
            point_map = self.tf_buffer.transform(
                point_camera,
                'map',  # target frame
                timeout=rclpy.duration.Duration(seconds=1.0)
            )

            # Step 3: Now we have object position on map!
            object_position = {
                'x': point_map.point.x,  # e.g., 6.8m
                'y': point_map.point.y,  # e.g., 4.2m
                'z': point_map.point.z   # e.g., 0.1m (floor height)
            }

            # Step 4: Add to object catalog
            self.catalog.add_detection(
                class_name='soda_can',
                position=object_position,
                confidence=detection_msg.confidence,
                timestamp=point_camera.header.stamp
            )

        except Exception as e:
            self.get_logger().warn(f'TF transform failed: {e}')
```

#### Example Calculation

**Given:**
- Robot position on map: (5.0, 3.0, 0.0) at 45° orientation
- Camera offset from robot: (0.15, 0.0, 0.25) facing forward
- Detection in camera frame: (0.3, 0.1, 2.0)

**Transform chain:**

1. **Camera to Robot Base:**
   - Camera is 0.15m forward, 0.25m up from robot center
   - Detection at (0.3, 0.1, 2.0) in camera frame
   - Becomes approximately (2.15, 0.3, 0.35) in robot frame
   - (accounting for camera position and detection coordinates)

2. **Robot Base to Map:**
   - Robot at (5.0, 3.0) on map, facing 45°
   - Point (2.15, 0.3) in robot frame needs rotation by 45°
   - After rotation and translation: approximately (6.8, 4.2) on map

**Result:** The soda can is at position (6.8m, 4.2m) on the map.

#### Handling Transform Timing

**Critical timing consideration:**

The robot is moving during exploration. Between when the camera captures an image and when we process it, the robot has moved. We must use the transform at the exact time of image capture.

```python
# BAD - uses current robot position (wrong!)
point_map = self.tf_buffer.transform(point_camera, 'map')

# GOOD - uses robot position at image capture time
point_camera.header.stamp = image_timestamp  # From camera message
point_map = self.tf_buffer.transform(
    point_camera,
    'map',
    timeout=Duration(seconds=1.0)
)
```

ROS2 TF maintains a history buffer (typically 10 seconds) of all transform changes, allowing us to look up the exact robot position at any past time.

#### De-duplication

Once we have map positions, we need to avoid counting the same can multiple times as the robot moves:

```python
class ObjectCatalog:
    def __init__(self):
        self.detected_objects = []
        self.dedup_distance = 0.3  # meters

    def add_detection(self, class_name, position, confidence, timestamp):
        # Check if this object already exists in catalog
        for existing_obj in self.detected_objects:
            if existing_obj.class_name == class_name:
                # Calculate distance on map
                distance = math.sqrt(
                    (position['x'] - existing_obj.position['x'])**2 +
                    (position['y'] - existing_obj.position['y'])**2
                )

                if distance < self.dedup_distance:
                    # Same object seen again, update confidence/timestamp
                    existing_obj.update(confidence, timestamp)
                    return  # Don't add duplicate

        # New object, add to catalog
        self.detected_objects.append(
            DetectedObject(class_name, position, confidence, timestamp)
        )
```

#### Visualization

For debugging and visualization, publish map markers:

```python
from visualization_msgs.msg import Marker

def publish_object_marker(self, obj_position, obj_id):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.ns = 'detected_objects'
    marker.id = obj_id
    marker.type = Marker.CYLINDER  # or SPHERE
    marker.action = Marker.ADD

    # Position on map
    marker.pose.position.x = obj_position['x']
    marker.pose.position.y = obj_position['y']
    marker.pose.position.z = 0.1  # Slightly above floor

    # Appearance
    marker.scale.x = 0.1  # diameter
    marker.scale.y = 0.1
    marker.scale.z = 0.2  # height
    marker.color.r = 1.0  # red
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.color.a = 0.8  # transparency

    self.marker_pub.publish(marker)
```

These markers appear in RViz/Foxglove showing exactly where detected objects are on the map.

#### TF Setup Requirements

**Required TF Publishers:**

1. **`robot_state_publisher`** (publishes `base_link → camera_link`)
   - Reads robot URDF
   - Publishes static transforms for all robot links
   - Includes camera mount position

2. **SLAM Toolbox** (publishes `map → odom`)
   - Publishes map frame to odometry frame transform
   - Updates as SLAM refines map and robot position

3. **Odometry source** (publishes `odom → base_link`)
   - From wheel encoders, visual odometry, or IMU fusion
   - Provides local odometry estimate

**Complete TF Tree:**
```
map (global reference)
  ↓
odom (odometry frame)
  ↓
base_link (robot center)
  ↓
camera_link (Oak-D-Lite)
```

**Verification:**

Check TF tree is complete:
```bash
ros2 run tf2_tools view_frames
# Generates PDF showing complete transform tree

# Or check live:
ros2 run tf2_ros tf2_echo map camera_link
# Should show transform from map to camera
```

#### Summary: Map Position Algorithm

**For each detected object:**

1. Oak-D-Lite provides (x, y, z) in camera frame
2. Use timestamp from camera message
3. Look up TF transform: `camera_link → map` at that timestamp
4. Apply transform to get (x, y, z) in map frame
5. Check catalog for existing object at similar map position
6. If new: Add to catalog
7. If existing: Update confidence/timestamp
8. Publish visualization marker at map position
9. Report to TUI: "Soda can detected at (6.8m, 4.2m)"

**Result:** Each detected object has a permanent map position, de-duplicated, and can be visualized or queried later.

#### Monitoring Localization Quality

**The Problem:**

If the robot's localization is poor (uncertain position on map), then object positions we calculate will also be inaccurate. We need to know when to trust our detections.

**Localization Quality Indicators:**

**1. Pose Covariance from SLAM Toolbox**

SLAM Toolbox publishes pose estimates with covariance (uncertainty) on the `/pose` topic:

```python
from geometry_msgs.msg import PoseWithCovarianceStamped

class LocalizationMonitor:
    def __init__(self):
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/pose',  # or '/slam_pose' depending on configuration
            self.pose_callback,
            10
        )
        self.localization_quality = 'unknown'

    def pose_callback(self, msg):
        # Covariance is 6x6 matrix (x, y, z, roll, pitch, yaw)
        # We care about x, y position uncertainty
        cov_xx = msg.pose.covariance[0]   # x position variance
        cov_yy = msg.pose.covariance[7]   # y position variance
        cov_yaw = msg.pose.covariance[35] # yaw variance

        # Standard deviation (sqrt of variance)
        std_x = math.sqrt(cov_xx)
        std_y = math.sqrt(cov_yy)
        std_yaw = math.sqrt(cov_yaw)

        # Combined position uncertainty
        position_uncertainty = math.sqrt(std_x**2 + std_y**2)

        # Classify localization quality
        if position_uncertainty < 0.1:  # Less than 10cm uncertainty
            self.localization_quality = 'excellent'
        elif position_uncertainty < 0.3:  # Less than 30cm
            self.localization_quality = 'good'
        elif position_uncertainty < 0.5:  # Less than 50cm
            self.localization_quality = 'fair'
        else:
            self.localization_quality = 'poor'

        # Log when quality changes
        self.get_logger().info(
            f'Localization: {self.localization_quality} '
            f'(±{position_uncertainty*100:.1f}cm, ±{math.degrees(std_yaw):.1f}°)'
        )
```

**2. Using Localization Quality for Object Detection**

Only record object positions when localization is good:

```python
def add_detection_with_quality_check(self, detection, position, timestamp):
    if self.localization_quality in ['excellent', 'good']:
        # High confidence in robot position, record object
        self.catalog.add_detection(
            class_name=detection.class_name,
            position=position,
            confidence=detection.confidence,
            localization_quality=self.localization_quality,
            timestamp=timestamp
        )
    elif self.localization_quality == 'fair':
        # Medium confidence, record but mark as uncertain
        self.get_logger().warn(
            f'Recording {detection.class_name} with fair localization'
        )
        self.catalog.add_detection(
            class_name=detection.class_name,
            position=position,
            confidence=detection.confidence * 0.7,  # Reduce confidence
            localization_quality='uncertain',
            timestamp=timestamp
        )
    else:
        # Poor localization, skip recording
        self.get_logger().warn(
            f'Skipping {detection.class_name} - poor localization'
        )
```

**3. When Does Localization Degrade?**

Common scenarios causing poor localization:

- **Feature-poor environments**: Long hallways, uniform walls, few distinctive features
- **Kidnapped robot**: Robot moved without odometry (picked up, wheels slipped)
- **Dynamic obstacles**: People, furniture moved since mapping
- **Fast rotation**: Spinning too quickly, SLAM can't track
- **Loop closure failure**: Revisiting area but SLAM doesn't recognize it

**4. Recovery Actions**

When localization quality drops:

```python
def handle_poor_localization(self):
    if self.localization_quality == 'poor':
        self.get_logger().warn('Poor localization detected!')

        # Action 1: Slow down robot
        self.reduce_robot_speed(max_velocity=0.1)  # 10cm/s

        # Action 2: Rotate slowly to gather more features
        self.perform_scan_rotation()

        # Action 3: Navigate to known good area with features
        # (e.g., area with distinctive furniture or landmarks)

        # Action 4: Pause object detection until quality improves
        self.pause_object_detection = True
```

**5. Visualizing Localization Quality**

Display uncertainty in TUI and RViz:

```python
from visualization_msgs.msg import Marker

def publish_uncertainty_marker(self, pose_msg):
    marker = Marker()
    marker.header.frame_id = 'map'
    marker.header.stamp = self.get_clock().now().to_msg()
    marker.type = Marker.CYLINDER
    marker.action = Marker.ADD

    # Position at robot location
    marker.pose = pose_msg.pose.pose

    # Scale represents uncertainty (diameter)
    uncertainty = math.sqrt(
        pose_msg.pose.covariance[0] +
        pose_msg.pose.covariance[7]
    )
    marker.scale.x = uncertainty * 4  # 2-sigma circle (95% confidence)
    marker.scale.y = uncertainty * 4
    marker.scale.z = 0.01  # Thin disk

    # Color represents quality (green=good, red=poor)
    if self.localization_quality == 'excellent':
        marker.color.r, marker.color.g, marker.color.b = 0.0, 1.0, 0.0
    elif self.localization_quality == 'good':
        marker.color.r, marker.color.g, marker.color.b = 0.5, 1.0, 0.0
    elif self.localization_quality == 'fair':
        marker.color.r, marker.color.g, marker.color.b = 1.0, 1.0, 0.0
    else:
        marker.color.r, marker.color.g, marker.color.b = 1.0, 0.0, 0.0

    marker.color.a = 0.3  # Semi-transparent

    self.uncertainty_marker_pub.publish(marker)
```

This creates a circle around the robot showing position uncertainty.

#### Multi-Frame Object Detection for Improved Positioning

**The Opportunity:**

When the same object is detected from multiple robot positions (different frames), we can:
1. **Triangulate** to get a better object position estimate
2. **Verify** the object is real (not a false positive)
3. **Potentially improve** robot localization using the object as a landmark

**Approach 1: Position Averaging with Weighted Confidence**

Track multiple observations of the same object and combine them:

```python
class DetectedObject:
    def __init__(self, class_name, initial_position, confidence, timestamp):
        self.class_name = class_name
        self.observations = [
            {
                'position': initial_position,
                'confidence': confidence,
                'timestamp': timestamp,
                'weight': confidence
            }
        ]
        self.position = initial_position  # Current best estimate

    def add_observation(self, new_position, confidence, timestamp):
        # Add new observation
        self.observations.append({
            'position': new_position,
            'confidence': confidence,
            'timestamp': timestamp,
            'weight': confidence
        })

        # Compute weighted average of all observations
        total_weight = sum(obs['weight'] for obs in self.observations)

        weighted_x = sum(
            obs['position']['x'] * obs['weight']
            for obs in self.observations
        ) / total_weight

        weighted_y = sum(
            obs['position']['y'] * obs['weight']
            for obs in self.observations
        ) / total_weight

        weighted_z = sum(
            obs['position']['z'] * obs['weight']
            for obs in self.observations
        ) / total_weight

        # Update position estimate
        self.position = {
            'x': weighted_x,
            'y': weighted_y,
            'z': weighted_z
        }

        # Track position uncertainty
        self.position_std_dev = self.compute_std_dev()

        self.get_logger().info(
            f'{self.class_name} refined: {len(self.observations)} observations, '
            f'uncertainty: ±{self.position_std_dev*100:.1f}cm'
        )

    def compute_std_dev(self):
        # Calculate standard deviation of position estimates
        positions_x = [obs['position']['x'] for obs in self.observations]
        positions_y = [obs['position']['y'] for obs in self.observations]

        std_x = np.std(positions_x)
        std_y = np.std(positions_y)

        return math.sqrt(std_x**2 + std_y**2)
```

**Approach 2: Kalman Filter for Object Position**

Use a Kalman filter to fuse multiple observations optimally:

```python
import numpy as np

class KalmanObjectTracker:
    def __init__(self, initial_position, initial_uncertainty=0.5):
        # State: [x, y]
        self.state = np.array([initial_position['x'], initial_position['y']])

        # Covariance (uncertainty in position)
        self.P = np.eye(2) * initial_uncertainty**2

        # Process noise (object doesn't move, so very small)
        self.Q = np.eye(2) * 0.01**2

    def predict(self):
        # Object is static, so prediction is just adding process noise
        self.P = self.P + self.Q

    def update(self, measurement, measurement_uncertainty):
        # measurement: new observed position [x, y]
        # measurement_uncertainty: uncertainty in this observation (from localization quality)

        # Measurement noise covariance
        R = np.eye(2) * measurement_uncertainty**2

        # Kalman gain
        K = self.P @ np.linalg.inv(self.P + R)

        # Update state
        innovation = measurement - self.state
        self.state = self.state + K @ innovation

        # Update covariance
        self.P = (np.eye(2) - K) @ self.P

        return self.state, np.sqrt(np.trace(self.P))  # Return position and uncertainty

# Usage:
class ObjectCatalog:
    def add_detection(self, class_name, position, confidence, localization_uncertainty):
        # Find existing object
        existing_obj = self.find_nearby_object(class_name, position)

        if existing_obj:
            # Update existing object with Kalman filter
            measurement = np.array([position['x'], position['y']])
            existing_obj.tracker.predict()
            new_position, uncertainty = existing_obj.tracker.update(
                measurement,
                localization_uncertainty
            )

            existing_obj.position = {
                'x': new_position[0],
                'y': new_position[1],
                'z': position['z']
            }
            existing_obj.uncertainty = uncertainty

            self.get_logger().info(
                f'{class_name} updated: uncertainty now ±{uncertainty*100:.1f}cm'
            )
        else:
            # New object
            new_obj = DetectedObject(class_name, position, confidence)
            new_obj.tracker = KalmanObjectTracker(position, localization_uncertainty)
            self.detected_objects.append(new_obj)
```

**Approach 3: Using Objects as Landmarks (Advanced)**

Once an object has been observed multiple times and has low position uncertainty, it can be used as a landmark to improve robot localization:

```python
class ObjectBasedLocalization:
    """
    Uses well-localized objects as landmarks to refine robot position.
    This is a simplified landmark-based SLAM approach.
    """

    def __init__(self):
        self.landmark_objects = []  # Objects with low uncertainty

    def promote_to_landmark(self, detected_object):
        """
        Promote an object to landmark status if:
        1. Observed from 3+ different positions
        2. Position uncertainty < 10cm
        3. High detection confidence
        """
        if (len(detected_object.observations) >= 3 and
            detected_object.position_std_dev < 0.1 and
            detected_object.avg_confidence > 0.8):

            self.landmark_objects.append(detected_object)
            self.get_logger().info(
                f'{detected_object.class_name} promoted to landmark at '
                f'({detected_object.position["x"]:.2f}, {detected_object.position["y"]:.2f})'
            )

    def refine_robot_pose(self, robot_pose_estimate, current_detections):
        """
        When robot sees a known landmark, use it to refine robot position.
        """
        for detection in current_detections:
            # Check if this detection matches a known landmark
            landmark = self.find_matching_landmark(detection)

            if landmark:
                # We know:
                # 1. Object's true position on map (from landmark)
                # 2. Object's position relative to camera (from detection)
                # 3. Camera's position relative to robot (from URDF)

                # Work backwards to compute where robot SHOULD be
                predicted_robot_pose = self.compute_robot_pose_from_landmark(
                    landmark_map_position=landmark.position,
                    detection_camera_coords=detection.spatialCoordinates,
                    camera_to_robot_transform=self.get_camera_transform()
                )

                # Compare with current SLAM estimate
                pose_difference = self.compute_pose_difference(
                    robot_pose_estimate,
                    predicted_robot_pose
                )

                if pose_difference > 0.3:  # More than 30cm difference
                    self.get_logger().warn(
                        f'Landmark {landmark.class_name} suggests robot position '
                        f'differs by {pose_difference*100:.1f}cm from SLAM estimate. '
                        f'Possible localization drift!'
                    )

                    # Could publish correction to SLAM or trigger relocalization
                    # This is advanced - typically SLAM Toolbox handles this via loop closure
```

**Benefits of Multi-Frame Detection:**

1. **Improved Object Position:**
   - 10 observations from different angles → position uncertainty drops from ±50cm to ±5cm
   - Outlier rejection (spurious detections filtered out)

2. **False Positive Filtering:**
   - Real objects are consistently detected from multiple viewpoints
   - False positives (reflections, shadows) won't be consistent

3. **Confidence Building:**
   - Object observed 5+ times with consistent position → very likely real
   - Object observed once → might be hallucination

4. **Localization Verification:**
   - Seeing expected landmark objects confirms robot is where SLAM thinks
   - NOT seeing expected landmarks → possible localization error

**Practical Example:**

```
Frame 1: Robot at (2.0, 1.0), detects can at (3.5, 2.2) - uncertainty ±40cm
Frame 2: Robot at (2.5, 1.5), detects can at (3.6, 2.1) - uncertainty ±30cm
Frame 3: Robot at (3.0, 2.0), detects can at (3.5, 2.3) - uncertainty ±35cm

After triangulation: Can position refined to (3.53, 2.20) - uncertainty ±8cm

The can is now a reliable landmark!
```

**Implementation Strategy for Dome2:**

**Phase 1 (MVP):**
- Monitor localization quality from SLAM covariance
- Only record objects when localization is "good" or better
- Simple position averaging for repeat detections

**Phase 2 (Enhanced):**
- Implement Kalman filter for object position tracking
- Promote well-observed objects to landmarks
- Use landmark count as mission quality metric

**Phase 3 (Advanced - Future):**
- Use landmarks to detect localization drift
- Trigger SLAM loop closure when landmarks don't match expected positions
- Object-assisted relocalization after complete localization loss

### Recommended Architecture for Dome2

**ROS2 Node Structure:**

```
Oak-D-Lite Hardware
    ↓
depthai_ros_driver (publishes /nn/detections)
    ↓
dome2_perception/object_detector.py (filters and validates)
    ↓
dome2_perception/object_catalog.py (tracks in map frame)
    ↓
/detected_objects topic (custom ObjectCatalog message)
    ↓
Mission Manager / TUI
```

**dome2_perception/object_detector.py responsibilities:**
- Subscribe to `/nn/detections` from depthai_ros_driver
- Filter by confidence threshold (>0.7)
- Filter by depth range (0.4m - 3.0m)
- Temporal filtering (3+ consecutive frames)
- Transform to map frame using TF
- Publish to `/detected_objects`

**dome2_perception/object_catalog.py responsibilities:**
- Maintain list of all detected objects with map positions
- De-duplicate (don't count same can twice)
- Associate detections across time
- Provide service to query object catalog

### Performance Considerations

**Frame Rate:**
- Oak-D-Lite can process 30 FPS with MobileNet-SSD
- ROS2 topic publishing at 15-30 Hz (configurable)
- Robot movement: Slower speeds improve detection quality

**Latency:**
- On-device inference: ~30ms
- ROS2 message transport: ~10-20ms
- Total detection latency: ~50ms (real-time)

**Computational Load:**
- Oak-D-Lite handles all AI processing
- Host CPU only receives detection results
- Minimal impact on robot's main compute resources
- Perfect for Raspberry Pi 5 host

### Example Configuration for Dome2

**config.yaml addition:**
```yaml
perception:
  camera:
    type: "oak-d-lite"
    model_path: "/home/dome2/models/sodacan_mobilenet.blob"  # or use default
    confidence_threshold: 0.7
    enable_spatial_detection: true
    fps: 15  # Balance between quality and compute

  object_detection:
    classes: ["soda_can"]  # or ["bottle"] for pre-trained
    min_depth: 0.4  # meters
    max_depth: 3.0  # meters
    temporal_filter_frames: 3  # require 3 consecutive detections

  catalog:
    deduplication_distance: 0.3  # meters (don't count same can twice)
    max_objects: 100
```

**Launch:**
```bash
# Start Oak-D-Lite with object detection
ros2 launch dome2_bringup exploration_full.launch.py
```

## Nav2 Navigation Control

### Yes, We Use Nav2

The Dome2 autonomous exploration system uses **Nav2** (ROS2 Navigation Stack) for all autonomous navigation:

- **Path planning**: Computes obstacle-free paths from current position to goal
- **Path following**: Drives robot along planned path
- **Obstacle avoidance**: Dynamically adjusts path to avoid obstacles
- **Recovery behaviors**: Handles stuck situations (rotate, back up, wait)

### How Nav2 Works

**Navigation flow:**

```
Exploration Node
    ↓ (sends goal pose)
Nav2 NavigateToPose Action Server
    ↓
Planner (computes path)
    ↓
Controller (follows path, sends velocity commands)
    ↓
/cmd_vel topic → Robot motors
```

**Key Nav2 components:**

1. **Planner** (global planner):
   - Computes complete path from start to goal
   - Uses costmap (obstacles from SLAM + lidar)
   - Runs when new goal received or path blocked

2. **Controller** (local planner):
   - Follows planned path
   - Adjusts for dynamic obstacles
   - Publishes velocity commands to `/cmd_vel`
   - Runs at 10-20 Hz

3. **Behavior Server**:
   - Recovery behaviors when stuck
   - Spin, backup, wait actions

### Controlling Robot Speed

**Method 1: Nav2 Parameters (Recommended)**

Configure maximum velocities in Nav2 parameters file:

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Linear velocity limits
      max_vel_x: 0.4              # m/s (maximum forward speed)
      min_vel_x: -0.2             # m/s (maximum backward speed)
      max_vel_y: 0.0              # m/s (differential drive, no lateral)
      min_vel_y: 0.0

      # Angular velocity limits
      max_vel_theta: 0.8          # rad/s (maximum rotation speed)
      min_vel_theta: -0.8         # rad/s

      # Acceleration limits (how quickly speed changes)
      acc_lim_x: 0.3              # m/s^2 (forward acceleration)
      acc_lim_y: 0.0
      acc_lim_theta: 1.0          # rad/s^2 (angular acceleration)

      # Deceleration limits
      decel_lim_x: -0.5           # m/s^2 (braking)
      decel_lim_y: 0.0
      decel_lim_theta: -2.0       # rad/s^2

      # Speed limits when approaching goal
      max_vel_x_goal: 0.2         # m/s (slow down near goal)
      max_vel_theta_goal: 0.4     # rad/s
```

**To slow down robot globally:**

```yaml
# Conservative exploration speeds for better object detection
max_vel_x: 0.2              # Slow to 20 cm/s
max_vel_theta: 0.4          # Slow rotation to 0.4 rad/s (~23°/s)
```

**Method 2: Runtime Parameter Updates**

Change speed dynamically during mission:

```python
import rclpy
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType

class NavigationController:
    def __init__(self):
        self.param_client = self.create_client(
            SetParameters,
            '/controller_server/set_parameters'
        )

    def set_max_speed(self, linear_speed, angular_speed):
        """
        Change Nav2 speed limits at runtime.

        Args:
            linear_speed: m/s (e.g., 0.2 for slow, 0.5 for fast)
            angular_speed: rad/s (e.g., 0.4 for slow, 1.0 for fast)
        """
        request = SetParameters.Request()

        # Set linear velocity
        param1 = Parameter()
        param1.name = 'FollowPath.max_vel_x'
        param1.value.type = ParameterType.PARAMETER_DOUBLE
        param1.value.double_value = linear_speed

        # Set angular velocity
        param2 = Parameter()
        param2.name = 'FollowPath.max_vel_theta'
        param2.value.type = ParameterType.PARAMETER_DOUBLE
        param2.value.double_value = angular_speed

        request.parameters = [param1, param2]

        future = self.param_client.call_async(request)
        self.get_logger().info(
            f'Updated Nav2 speeds: {linear_speed} m/s, {angular_speed} rad/s'
        )

# Usage:
nav_controller = NavigationController()

# Slow down for object detection
nav_controller.set_max_speed(linear_speed=0.2, angular_speed=0.4)

# Speed up when just navigating
nav_controller.set_max_speed(linear_speed=0.5, angular_speed=1.0)
```

**Method 3: Speed Zones via Costmap**

Create "speed restriction zones" in the costmap:

```yaml
# Add to nav2_params.yaml
global_costmap:
  global_costmap:
    plugins: ["static_layer", "obstacle_layer", "inflation_layer", "speed_filter"]

    speed_filter:
      plugin: "nav2_costmap_2d::SpeedFilter"
      enabled: True
      filter_info_topic: "/speed_filter_info"
      speed_limit_topic: "/speed_limit"
```

### Commanding Specific Behaviors

**Turn in Place (Rotate)**

**Option 1: Send Rotation Goal**

```python
from nav2_msgs.action import Spin
from rclpy.action import ActionClient
import math

class NavigationController:
    def __init__(self):
        self.spin_client = ActionClient(self, Spin, 'spin')

    def rotate_in_place(self, angle_degrees):
        """
        Rotate robot in place by specified angle.

        Args:
            angle_degrees: Rotation angle (positive = counter-clockwise)
        """
        goal = Spin.Goal()
        goal.target_yaw = math.radians(angle_degrees)

        self.spin_client.wait_for_server()
        future = self.spin_client.send_goal_async(goal)

        self.get_logger().info(f'Rotating {angle_degrees}°')
        return future

# Usage:
nav_controller = NavigationController()

# Rotate 90° counter-clockwise
nav_controller.rotate_in_place(90)

# Rotate 180° (turn around)
nav_controller.rotate_in_place(180)

# Rotate 360° (full scan)
nav_controller.rotate_in_place(360)
```

**Option 2: Navigate to Pose with Same Position**

```python
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import math
import tf_transformations

class NavigationController:
    def __init__(self):
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def get_current_pose(self):
        """Get robot's current pose from TF"""
        transform = self.tf_buffer.lookup_transform(
            'map',
            'base_link',
            rclpy.time.Time()
        )

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position = transform.transform.translation
        pose.pose.orientation = transform.transform.rotation

        return pose

    def turn_to_heading(self, target_heading_degrees):
        """
        Turn to absolute heading on map.

        Args:
            target_heading_degrees: Target heading (0=east, 90=north)
        """
        # Get current position
        current_pose = self.get_current_pose()

        # Create goal at same position but new heading
        goal_pose = PoseStamped()
        goal_pose.header = current_pose.header
        goal_pose.pose.position = current_pose.pose.position

        # Convert heading to quaternion
        target_yaw = math.radians(target_heading_degrees)
        quat = tf_transformations.quaternion_from_euler(0, 0, target_yaw)
        goal_pose.pose.orientation.x = quat[0]
        goal_pose.pose.orientation.y = quat[1]
        goal_pose.pose.orientation.z = quat[2]
        goal_pose.pose.orientation.w = quat[3]

        # Send goal to Nav2
        goal = NavigateToPose.Goal()
        goal.pose = goal_pose

        future = self.nav_client.send_goal_async(goal)
        self.get_logger().info(f'Turning to heading {target_heading_degrees}°')

        return future

# Usage:
nav_controller = NavigationController()

# Turn to face north
nav_controller.turn_to_heading(90)

# Turn to face east
nav_controller.turn_to_heading(0)
```

**Scan Rotation (360° for Better Localization)**

```python
def perform_scan_rotation(self, angular_velocity=0.3):
    """
    Slowly rotate 360° to improve localization.
    Use when localization quality is poor.

    Args:
        angular_velocity: rad/s (slow = better feature matching)
    """
    # Temporarily slow down rotation
    original_speed = self.get_parameter('FollowPath.max_vel_theta')
    self.set_max_speed(linear_speed=0.0, angular_speed=angular_velocity)

    # Perform 360° rotation
    self.rotate_in_place(360)

    # Wait for rotation to complete
    time.sleep(2 * math.pi / angular_velocity)  # Time for full rotation

    # Restore original speed
    self.set_max_speed(linear_speed=0.4, angular_speed=original_speed)

    self.get_logger().info('Scan rotation complete')
```

### Emergency Stop

**Method 1: Cancel Nav2 Goal**

```python
def emergency_stop(self):
    """Stop robot immediately by canceling current navigation goal"""
    # Cancel NavigateToPose action
    if self.nav_client.gh:
        cancel_future = self.nav_client.gh.cancel_goal_async()
        self.get_logger().warn('EMERGENCY STOP - Goal canceled')

    # Also publish zero velocity directly
    stop_cmd = Twist()
    stop_cmd.linear.x = 0.0
    stop_cmd.angular.z = 0.0
    self.cmd_vel_pub.publish(stop_cmd)
```

**Method 2: Direct Velocity Command**

```python
from geometry_msgs.msg import Twist

class NavigationController:
    def __init__(self):
        # Create publisher to /cmd_vel (bypasses Nav2)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def publish_velocity(self, linear_x, angular_z):
        """
        Directly command robot velocity (bypasses Nav2).
        Use only for emergency stop or manual control.

        Args:
            linear_x: m/s forward velocity
            angular_z: rad/s rotational velocity
        """
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)

    def stop(self):
        """Emergency stop"""
        self.publish_velocity(0.0, 0.0)
        self.get_logger().warn('STOP command sent')

# Usage:
nav_controller = NavigationController()
nav_controller.stop()
```

### Integration with Object Detection

**Slow down when detecting objects:**

```python
class ExplorationWithDetection:
    def __init__(self):
        self.nav_controller = NavigationController()
        self.detection_mode = False

    def detection_callback(self, detection_msg):
        """When object detected, slow down for better quality"""
        if len(detection_msg.detections) > 0 and not self.detection_mode:
            # Objects detected - slow down
            self.detection_mode = True
            self.nav_controller.set_max_speed(
                linear_speed=0.15,   # 15 cm/s (slow)
                angular_speed=0.3    # 0.3 rad/s (slow turn)
            )
            self.get_logger().info('Objects detected - slowing down')

        elif len(detection_msg.detections) == 0 and self.detection_mode:
            # No objects - return to normal speed
            self.detection_mode = False
            self.nav_controller.set_max_speed(
                linear_speed=0.4,    # 40 cm/s (normal)
                angular_speed=0.8    # 0.8 rad/s (normal)
            )
            self.get_logger().info('No objects - normal speed')
```

**Pause navigation for high-quality detection:**

```python
def pause_for_detection(self, duration_seconds=2.0):
    """
    Stop robot to take high-quality image for object detection.
    Eliminates motion blur.
    """
    # Stop robot
    self.nav_controller.stop()

    # Wait for robot to settle
    time.sleep(0.5)

    # Trigger high-quality detection
    self.get_logger().info('Paused for high-quality detection')

    # Wait
    time.sleep(duration_seconds)

    # Resume navigation by sending same goal again
    # Nav2 will continue toward current goal
```

### Nav2 Configuration for Dome2

**Recommended settings for exploration with object detection:**

```yaml
# nav2_params.yaml - Dome2 specific configuration

controller_server:
  ros__parameters:
    controller_frequency: 20.0

    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"

      # Conservative speeds for reliable object detection
      max_vel_x: 0.3              # 30 cm/s max
      min_vel_x: -0.1             # 10 cm/s backward
      max_vel_theta: 0.6          # ~34°/s rotation

      # Smooth acceleration (less jerky)
      acc_lim_x: 0.2              # Gentle acceleration
      acc_lim_theta: 0.8          # Gentle rotation

      # Goal tolerance (how close to get to goal)
      xy_goal_tolerance: 0.15     # 15cm position tolerance
      yaw_goal_tolerance: 0.2     # ~11° heading tolerance

      # Trajectory scoring (how Nav2 picks best path)
      path_distance_bias: 32.0    # Stay close to global path
      goal_distance_bias: 20.0    # Head toward goal
      occdist_scale: 0.02         # Avoid obstacles

      # Dynamic obstacle handling
      critics: ["RotateToGoal", "Oscillation", "BaseObstacle", "GoalAlign", "PathAlign", "PathDist", "GoalDist"]

planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]

    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.2              # 20cm path planning tolerance
      use_astar: false            # Dijkstra (more reliable)
      allow_unknown: true         # Can plan through unexplored areas

recoveries_server:
  ros__parameters:
    costmap_topic: local_costmap/costmap_raw
    footprint_topic: local_costmap/published_footprint
    cycle_frequency: 10.0
    recovery_plugins: ["spin", "backup", "wait"]

    spin:
      plugin: "nav2_recoveries/Spin"
      simulate_ahead_time: 2.0

    backup:
      plugin: "nav2_recoveries/BackUp"
      simulate_ahead_time: 2.0

    wait:
      plugin: "nav2_recoveries/Wait"
      wait_duration: 5.0
```

### Typical Navigation Patterns

**Pattern 1: Navigate to Frontier**

```python
def navigate_to_frontier(self, frontier_pose):
    """Standard exploration navigation"""
    goal = NavigateToPose.Goal()
    goal.pose = frontier_pose

    future = self.nav_client.send_goal_async(goal)
    # Nav2 handles path planning, obstacle avoidance, etc.
    return future
```

**Pattern 2: Navigate with Detection Pauses**

```python
def navigate_with_detection(self, goal_pose):
    """Navigate while periodically pausing for object detection"""
    # Start navigation
    goal_handle = self.navigate_to_frontier(goal_pose)

    while not self.is_goal_reached():
        # Navigate for 3 seconds
        time.sleep(3.0)

        # Pause for detection
        self.pause_for_detection(duration_seconds=2.0)

        # Resume (Nav2 continues to same goal automatically)

    self.get_logger().info('Goal reached with detection pauses')
```

**Pattern 3: Adaptive Speed Navigation**

```python
def navigate_adaptive_speed(self, goal_pose):
    """Automatically adjust speed based on localization quality"""
    goal_handle = self.navigate_to_frontier(goal_pose)

    while not self.is_goal_reached():
        if self.localization_quality == 'poor':
            # Poor localization - slow down
            self.nav_controller.set_max_speed(0.1, 0.3)
        elif self.localization_quality == 'fair':
            # Fair - moderate speed
            self.nav_controller.set_max_speed(0.2, 0.5)
        else:
            # Good/excellent - normal speed
            self.nav_controller.set_max_speed(0.4, 0.8)

        time.sleep(1.0)
```

### Summary: Nav2 Control

**Speed Control:**
- Configure in `nav2_params.yaml` for persistent settings
- Update at runtime via parameter service for dynamic control
- Slow down during object detection for better quality

**Rotation:**
- Use `Spin` action for in-place rotation
- Send `NavigateToPose` with same position, new heading
- 360° scan rotation to improve poor localization

**Emergency Control:**
- Cancel Nav2 goal via action client
- Publish zero velocity directly to `/cmd_vel`

**Integration:**
- Nav2 runs continuously in background
- Exploration node sends goal poses
- Object detection triggers speed adjustments
- Localization quality affects navigation behavior

## Summary & Recommendations

**For Dome2 Project:**

1. **Start with pre-trained MobileNet-SSD** detecting "bottle" class
   - Fastest path to working system
   - Validate ROS2 integration and workflow
   - Test with actual robot movement

2. **Collect dataset during initial testing**
   - Save images of soda cans in your environment
   - Label and train custom model later for improved accuracy

3. **Use depthai-ros official driver** for ROS2 Jazzy
   - Well-maintained and documented
   - Composable node architecture
   - Direct spatial detection support

4. **Implement robust filtering**
   - Temporal filtering for motion
   - Depth filtering for range
   - Map-based deduplication

5. **Fixed-Focus variant recommended**
   - Better for moving robot platform
   - Avoids autofocus hunting during motion
   - More reliable in vibration environments

## Sources

- [OAK-D Lite Product Page - Luxonis](https://shop.luxonis.com/products/oak-d-lite-1)
- [OAK-D Lite Documentation - Luxonis](https://docs.luxonis.com/hardware/products/OAK-D%20Lite)
- [DepthAI ROS Documentation](https://docs.luxonis.com/software/ros/depthai-ros/)
- [depthai-ros GitHub Repository](https://github.com/luxonis/depthai-ros)
- [OAK-D Lite Camera ROS2 Setup - Medium](https://robofoundry.medium.com/oak-d-lite-camera-ros2-setup-1e74ed03350d)
- [Object Detection With Depth Measurement - Learn OpenCV](https://learnopencv.com/object-detection-with-depth-measurement-with-oak-d/)
- [OAK-D Understanding Neural Network Inference - PyImageSearch](https://pyimagesearch.com/2022/12/19/oak-d-understanding-and-running-neural-network-inference-with-depthai-api/)
- [Luxonis OAK-D Custom Model Deployment - Roboflow](https://blog.roboflow.com/luxonis-oak-d-custom-model/)
- [DepthAI Custom Training Tutorial](https://colab.research.google.com/github/luxonis/depthai-ml-training/blob/master/colab-notebooks/Easy_Object_Detection_With_Custom_Data_Demo_Training.ipynb)
- [Introduction to OpenCV AI Kit (OAK) - PyImageSearch](https://pyimagesearch.com/2022/11/28/introduction-to-opencv-ai-kit-oak/)

