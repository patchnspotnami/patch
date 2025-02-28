# coding=utf-8
import os
import argparse
import blobconverter
import cv2
import time
import depthai as dai
import numpy as np
from .MultiMsgSync import TwoStageHostSeqSync


#import ROS stuff
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from depthai_ros_msgs.msg import TrackedFeatures
from std_msgs.msg import String
from cv_bridge import CvBridge
#from image_transport import ImageTransport

parser = argparse.ArgumentParser()
parser.add_argument("-name", "--name", type=str, help="Name of the person for database saving")

args = parser.parse_args()

def frame_norm(frame, bbox):
    normVals = np.full(len(bbox), frame.shape[0])
    normVals[::2] = frame.shape[1]
    return (np.clip(np.array(bbox), 0, 1) * normVals).astype(int)

VIDEO_SIZE = (1072, 1072)
databases = "databases"
if not os.path.exists(databases):
    os.mkdir(databases)
 
class PatchCamPublisher(Node):
    def __init__(self, pipeline):
        super().__init__('patch_cam_publisher')
        self.image_publisher = self.create_publisher(CompressedImage, 'camera/image/compressed', 10)
        self.name_publisher = self.create_publisher(String, 'identified_person', 10)
        
        self.device = dai.Device(pipeline)  # start the pipeline
        
        self.facerec = FaceRecognition(databases, args.name)
        self.sync = TwoStageHostSeqSync()
        self.text = TextHelper()
        
        self.queues = {}
        # Create output queues
        for name in ["color", "detection", "recognition", "depth", "spatialData"]:
            self.queues[name] = self.device.getOutputQueue(name)

        
        
      
        self.timer = self.create_timer(0.1, self.cam_timer_callback)  # 10 Hz
        
    def cam_timer_callback(self):
        for name, q in self.queues.items():
                # Add all msgs (color frames, object detections and face recognitions) to the Sync class.
                if q.has():
                    self.sync.add_msg(q.get(), name)

        msgs = self.sync.get_msgs()


        if msgs is not None:
            frame = msgs["color"].getCvFrame()
            dets = msgs["detection"].detections
            jspatial = msgs["spatialData"]
                            
            # Encode the image as JPEG
            _, buffer = cv2.imencode('.jpg', frame)
            compressed_image = buffer.tobytes()
                
 
            # Create CompressedImage message
            img_msg = CompressedImage()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.format = "jpeg"
            img_msg.data = compressed_image

            # Publish the image message
            self.image_publisher.publish(img_msg)
            self.get_logger().info('Published compressed image')               

            for i, detection in enumerate(dets):
                bbox = frame_norm(frame, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
                cv2.rectangle(frame, (bbox[0], bbox[1]), (bbox[2], bbox[3]), (10, 245, 10), 2)
                if (len(jspatial) > 0):
           
                    for depth in jspatial[0].spatialLocations:
 
                        self.text.putText(frame, f"Z: {int(depth.spatialCoordinates.z)} mm", (bbox[0] + 40, bbox[1] + 65))
                features = np.array(msgs["recognition"][i].getFirstLayerFp16())
                conf, name = self.facerec.new_recognition(features)
                # Create ROS String message
                av_x = (bbox[0] + bbox[2])/2
                av_y = (bbox[1] + bbox[3])/2
                name_msg = String()
                name_msg.data = name + "  " +str(av_x) + "  " + str(av_y)
                # Publish the name message
                self.name_publisher.publish(name_msg)
     
                self.text.putText(frame, f"{name} {(100*conf):.0f}%", (bbox[0] + 10,bbox[1] + 35))

#            cv2.imshow("color", cv2.resize(frame, (800,800)))

        cv2.waitKey(1) == ord('q')
        
      

 
    

class TextHelper:
    def __init__(self) -> None:
        self.bg_color = (0, 0, 0)
        self.color = (255, 255, 255)
        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA
    def putText(self, frame, text, coords):
        cv2.putText(frame, text, coords, self.text_type, 1.0, self.bg_color, 4, self.line_type)
        cv2.putText(frame, text, coords, self.text_type, 1.0, self.color, 2, self.line_type)

class FaceRecognition:
    def __init__(self, db_path, name) -> None:
        self.read_db(db_path)
        self.name = name
        self.bg_color = (0, 0, 0)
        self.color = (255, 255, 255)
        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA
        self.printed = True

    def cosine_distance(self, a, b):
        if a.shape != b.shape:
            raise RuntimeError("array {} shape not match {}".format(a.shape, b.shape))
        a_norm = np.linalg.norm(a)
        b_norm = np.linalg.norm(b)
        return np.dot(a, b.T) / (a_norm * b_norm)

    def new_recognition(self, results):
        conf = []
        max_ = 0
        label_ = None
        for label in list(self.labels):
            for j in self.db_dic.get(label):
                conf_ = self.cosine_distance(j, results)
                if conf_ > max_:
                    max_ = conf_
                    label_ = label

        conf.append((max_, label_))
        name = conf[0] if conf[0][0] >= 0.5 else (1 - conf[0][0], "UNKNOWN")
        # self.putText(frame, f"name:{name[1]}", (coords[0], coords[1] - 35))
        # self.putText(frame, f"conf:{name[0] * 100:.2f}%", (coords[0], coords[1] - 10))

        if name[1] == "UNKNOWN":
            self.create_db(results)
        return name

    def read_db(self, databases_path):
        self.labels = []
        for file in os.listdir(databases_path):
            filename = os.path.splitext(file)
            if filename[1] == ".npz":
                self.labels.append(filename[0])

        self.db_dic = {}
        for label in self.labels:
            with np.load(f"{databases_path}/{label}.npz") as db:
                self.db_dic[label] = [db[j] for j in db.files]

    def putText(self, frame, text, coords):
        cv2.putText(frame, text, coords, self.text_type, 1, self.bg_color, 4, self.line_type)
        cv2.putText(frame, text, coords, self.text_type, 1, self.color, 1, self.line_type)

    def create_db(self, results):
        if self.name is None:
            if not self.printed:
                print("Wanted to create new DB for this face, but --name wasn't specified")
                self.printed = True
            return
        print('Saving face...')
        try:
            with np.load(f"{databases}/{self.name}.npz") as db:
                db_ = [db[j] for j in db.files][:]
        except Exception as e:
            db_ = []
        db_.append(np.array(results))
        np.savez_compressed(f"{databases}/{self.name}", *db_)
        self.adding_new = False
        
        
def create_pipeline():
    print("Creating pipeline...")
    pipeline = dai.Pipeline()

    # Define sources and outputs
    monoLeft = pipeline.create(dai.node.MonoCamera)
    monoRight = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    spatialLocationCalculator = pipeline.create(dai.node.SpatialLocationCalculator)

    xoutDepth = pipeline.create(dai.node.XLinkOut)
    xoutSpatialData = pipeline.create(dai.node.XLinkOut)
    xinSpatialCalcConfig = pipeline.create(dai.node.XLinkIn)

    xoutDepth.setStreamName("depth")
    xoutSpatialData.setStreamName("spatialData")
    xinSpatialCalcConfig.setStreamName("spatialCalcConfig")

    print("Creating Color Camera...")
    cam = pipeline.create(dai.node.ColorCamera)
    # For ImageManip rotate you need input frame of multiple of 16
    cam.setPreviewSize(1072, 1072)
    cam.setVideoSize(VIDEO_SIZE)
    cam.setFps(4)  # restrict frame per second
    cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    cam.setInterleaved(False)
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)

    host_face_out = pipeline.create(dai.node.XLinkOut)
    host_face_out.setStreamName('color')
    cam.video.link(host_face_out.input)

    # ImageManip as a workaround to have more frames in the pool.
    # cam.preview can only have 4 frames in the pool before it will
    # wait (freeze). Copying frames and setting ImageManip pool size to
    # higher number will fix this issue.
    copy_manip = pipeline.create(dai.node.ImageManip)
    cam.preview.link(copy_manip.inputImage)
    copy_manip.setNumFramesPool(20)
    copy_manip.setMaxOutputFrameSize(1072*1072*3)

    # ImageManip that will crop the frame before sending it to the Face detection NN node
    face_det_manip = pipeline.create(dai.node.ImageManip)
    face_det_manip.initialConfig.setResize(300, 300)
    copy_manip.out.link(face_det_manip.inputImage)

    # NeuralNetwork
    print("Creating Face Detection Neural Network...")
    face_det_nn = pipeline.create(dai.node.MobileNetDetectionNetwork)
    face_det_nn.setConfidenceThreshold(0.5)
    face_det_nn.setBlobPath(blobconverter.from_zoo(name="face-detection-retail-0004", shaves=6))
    # Link Face ImageManip -> Face detection NN node
    face_det_manip.out.link(face_det_nn.input)

    face_det_xout = pipeline.create(dai.node.XLinkOut)
    face_det_xout.setStreamName("detection")
    face_det_nn.out.link(face_det_xout.input)


    # Properties
    monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoLeft.setCamera("left")                 
    monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
    monoRight.setCamera("right")

    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setSubpixel(True)


    # Linking
    monoLeft.out.link(stereo.left)
    monoRight.out.link(stereo.right)

    spatialLocationCalculator.passthroughDepth.link(xoutDepth.input)
    stereo.depth.link(spatialLocationCalculator.inputDepth)

    spatialLocationCalculator.out.link(xoutSpatialData.input)
    #xinSpatialCalcConfig.out.link(spatialLocationCalculator.inputConfig)




     # Script node will take the output from the face detection NN as an input and set ImageManipConfig
    # to the 'age_gender_manip' to crop the initial frame
    script = pipeline.create(dai.node.Script)
    script.setProcessor(dai.ProcessorType.LEON_CSS)

    face_det_nn.out.link(script.inputs['face_det_in'])
# We also interested in sequence number for syncing
    face_det_nn.passthrough.link(script.inputs['face_pass'])

    copy_manip.out.link(script.inputs['preview'])

    with open("/home/patchcam/oak_ws/src/patch_cam/patch_cam/script.py", "r") as f:
        script.setScript(f.read())
    
    
    script.outputs['spatial_cfg'].link(spatialLocationCalculator.inputConfig)

    print("Creating Head pose estimation NN")

    headpose_manip = pipeline.create(dai.node.ImageManip)
    headpose_manip.initialConfig.setResize(60, 60)
    headpose_manip.setWaitForConfigInput(True)
    script.outputs['manip_cfg'].link(headpose_manip.inputConfig)
    script.outputs['manip_img'].link(headpose_manip.inputImage)

    headpose_nn = pipeline.create(dai.node.NeuralNetwork)
    headpose_nn.setBlobPath(blobconverter.from_zoo(name="head-pose-estimation-adas-0001", shaves=6))
    headpose_manip.out.link(headpose_nn.input)

    headpose_nn.out.link(script.inputs['headpose_in'])
    headpose_nn.passthrough.link(script.inputs['headpose_pass'])

    print("Creating face recognition ImageManip/NN")

    face_rec_manip = pipeline.create(dai.node.ImageManip)
    face_rec_manip.initialConfig.setResize(112, 112)
    face_rec_manip.inputConfig.setWaitForMessage(True)

    script.outputs['manip2_cfg'].link(face_rec_manip.inputConfig)
    script.outputs['manip2_img'].link(face_rec_manip.inputImage)

    face_rec_nn = pipeline.create(dai.node.NeuralNetwork)
    face_rec_nn.setBlobPath(blobconverter.from_zoo(name="face-recognition-arcface-112x112", zoo_type="depthai", shaves=6))
    face_rec_manip.out.link(face_rec_nn.input)

    arc_xout = pipeline.create(dai.node.XLinkOut)
    arc_xout.setStreamName('recognition')
    face_rec_nn.out.link(arc_xout.input)
    
    return pipeline

def main():
    pipeline = create_pipeline()
    rclpy.init()  # start ros2
    patchcam_recog_pub = PatchCamPublisher(pipeline)
    rclpy.spin(patchcam_recog_pub)
    patchcam_recog_pub.destroy_node()
    rclpy.shutdown()
    
    
    
               
if __name__ == "__main__":
    main()
