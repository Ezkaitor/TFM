## Common libraries
from pathlib import Path
import cv2 as cv
import numpy as np

## ROS libraries
import rospy
# ROS Messages
from std_msgs.msg import Header
from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBox,BoundingBoxes
# OpenVino Library
from openvino.inference_engine import IECore

## COCO Classes ##
labels_map = {0: '__background__', 1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus', 7: 'train', 8: 'truck', 9: 'boat',10: 'traffic light',11: 'fire hydrant',12: 'stop sign',13: 'parking meter',14: 'bench',15: 'bird',16: 'cat',17: 'dog',18: 'horse',19: 'sheep',20: 'cow',21: 'elephant',22: 'bear',23: 'zebra',24: 'giraffe',25: 'backpack',26: 'umbrella',27: 'handbag',28: 'tie',29: 'suitcase',30: 'frisbee',31: 'skis',32: 'snowboard',33: 'sports ball',34: 'kite',35: 'baseball bat',36: 'baseball glove',37: 'skateboard',38: 'surfboard',39: 'tennis racket',40: 'bottle',41: 'wine glass',42: 'cup',43: 'fork',44: 'knife',45: 'spoon',46: 'bowl',47: 'banana',48: 'apple',49: 'sandwich',50: 'orange',51: 'broccoli',52: 'carrot',53: 'hot dog',54: 'pizza',55: 'donut',56: 'cake',57: 'chair',58: 'couch',59: 'potted plant',60: 'bed',61: 'dining table',62: 'toilet',63: 'tv',64: 'laptop',65: 'mouse',66: 'remote',67: 'keyboard',68: 'cell phone',69: 'microwave',70: 'oven',71: 'toaster',72: 'sink',73: 'refrigerator',74: 'book',75: 'clock',76: 'vase',77: 'scissors',78: 'teddy bear',79: 'hair drier',80: 'toothbrush'}

#### INITIALIZE DEVICE ####
ie = IECore()

ncs_model_path = Path("/home/aitor/repos/TFM/drone-test/NCS_models/mobilenet_ssd")

network = ie.read_network(str(ncs_model_path/(ncs_model_path.name+".xml")), str(ncs_model_path/(ncs_model_path.name+".bin"))) # TODO: make it realtive path

img_info_input_blob = None
feed_dict = {}
for blob_name in network.inputs:
    print(blob_name)
    if len(network.inputs[blob_name].shape) == 4:
            input_blob = blob_name
    elif len(network.inputs[blob_name].shape) == 2:
        img_info_input_blob = blob_name
    else:
        raise RuntimeError("Unsupported {}D input layer '{}'. Only 2D and 4D input layers are supported"
                            .format(len(network.inputs[blob_name].shape), blob_name))

assert len(network.outputs) == 1, "Demo supports only single output topologies"
out_blob = next(iter(network.outputs))

rospy.loginfo("Checking device...")
if "MYRIAD" in ie.available_devices: device="MYRIAD"
else:
    device="CPU"
    rospy.logwarn("MYRIAD device is not plugged.")

rospy.loginfo("Loading IR to the plugin...")
exec_network = ie.load_network(network=network, num_requests=2, device_name=device)

n, c, h, w = network.inputs[input_blob].shape

if img_info_input_blob:
    feed_dict[img_info_input_blob] = [h, w, 1]


# TODO: add a label list to pick from, so rest of the labels will be discarded
#if args.labels:
#        with open(args.labels, 'r') as f:
#            labels_map = [x.strip() for x in f]
#    else:
#        labels_map = None

########################
### Start rospy node ###

image_recieved = False
camera_info_received = False
image = None
image_size = [0,0, 3]

def get_image(msg):
    global image
    global image_recieved
    global image_size
    image_size = (msg.width, msg.height, 3)
    image = np.frombuffer(msg.data, dtype=np.uint8).reshape(image_size)
    #image = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
    image_recieved = True

#def get_camera_info(msg):
#    image_size[0] = msg.width
#    image_size[1] = msg.height
#    camera_info_received = True

rospy.init_node("mobilenet_ssd")

image_tp = rospy.Subscriber('iris/camera/image_raw', Image, get_image)
#camera_info_tp = rospy.Subscriber('iris/camera/camera_info', CameraInfo, get_camera_info)

bb_pub = rospy.Publisher("mobilenet_ros/bounding_boxes", BoundingBoxes, queue_size=1)

rate = rospy.Rate(2)

while not rospy.is_shutdown():
    if image_recieved: # and camera_info_received:
        #image = image.reshape(image_size[:2]) # RGB Image
        image = cv.resize(image, (w,h))
        image = image.transpose((2,0,1)) # Change data layout from HWC to CHW
        image = image.reshape((n, c, h, w))
        feed_dict[input_blob] = image
        #exec_network.start_async(request_id=0, inputs=feed_dict)

        #if exec_network.requests[0].wait(-1) == 0:
        #inf_end = time.time()
        #det_time = inf_end - inf_start

        # Parse detection results of the current request
            #res = exec_network.requests[0].outputs[out_blob]
        res = exec_network.infer(inputs={input_blob: [image]})
        update_res = res[out_blob]
        out = update_res.reshape((100, 7)) # Maximum of 100 results, 7 values
        bb_list = []
        for obj in out:
            if obj[0] == -1: break # Reached las object
            # Draw only objects when probability more than specified threshold
            if obj[2]*100 > 50:
                bbox = BoundingBox()
                bbox.xmin = int(obj[3] * image_size[0])
                bbox.ymin = int(obj[4] * image_size[1])
                bbox.xmax = int(obj[5] * image_size[0])
                bbox.ymax = int(obj[6] * image_size[1])

                bbox.probability = obj[2]

                bbox.id = int(obj[1])
                bbox.Class = "" # TODO: to be determined
                
                bbox.Class = labels_map[int(obj[1])] if labels_map else str(int(obj[1]))
                
                bb_list.append(bbox)
        bounding_boxes = BoundingBoxes()
        bounding_boxes.header = Header()
        bounding_boxes.image_header = Header()

        bounding_boxes.bounding_boxes = bb_list
        bb_pub.publish(bounding_boxes)
    
    rate.sleep()

