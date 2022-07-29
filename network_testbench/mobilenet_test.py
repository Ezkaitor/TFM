## Common libraries
from os import times_result
from pathlib import Path
import cv2 as cv
from matplotlib import patches
import numpy as np
import time
from tqdm import tqdm
import json
from pathlib import Path

### Debugging ###
#import matplotlib.pyplot as plt
#import matplotlib.patches as patches
#################

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

print("Checking device...")
if "MYRIAD" in ie.available_devices: device="MYRIAD"
else:
    device="CPU"
    print("MYRIAD device is not plugged.")

print("Loading IR to the plugin...")
exec_network = ie.load_network(network=network, num_requests=2, device_name=device)

n, c, h, w = network.inputs[input_blob].shape

if img_info_input_blob:
    feed_dict[img_info_input_blob] = [h, w, 1]

########################
### Start rospy node ###

dataset_path = Path("/home/aitor/datasets/val2017")
#dataset_path = Path("/home/aitor/datasets/val2017_test")

results = []
time_elapsed = []
for image_path in tqdm(dataset_path.iterdir()):
    image_id = int(image_path.stem.strip('0'))
    image_raw = cv.imread(str(image_path))
    image_size = image_raw.shape

    start_time = time.time()
    image = cv.resize(image_raw, (w,h))
    image = image.transpose((2,0,1)) # Change data layout from HWC to CHW
    image = image.reshape((n, c, h, w))
    feed_dict[input_blob] = image
    
    # Infer
    res = exec_network.infer(inputs={input_blob: [image]})
    update_res = res[out_blob]
    out = update_res.reshape((100, 7)) # Maximum of 100 results, 7 values
    bb_list = []
    for obj in out:
        if obj[0] == -1: break # Reached las object
        # Draw only objects when probability more than specified threshold
            
        xmin = int(obj[3] * image_size[1])
        ymin = int(obj[4] * image_size[0])
        xmax = int(obj[5] * image_size[1])
        ymax = int(obj[6] * image_size[0])

        probability = obj[2]

        id = int(obj[1])
        
        result_dict = {'image_id': image_id}
        result_dict["category_id"]= id
        result_dict["bbox"] = [xmin, ymin, xmax-xmin, ymax-ymin]
        result_dict["score"] = float(probability)
        results.append(result_dict)

        '''
        fig, ax = plt.subplots(1)
        ax.imshow(image_raw)
        # Create a Rectangle patch
        rect = patches.Rectangle((xmin, ymin), xmax-xmin, ymax-ymin, linewidth=1, edgecolor='r', facecolor='none')
        plt.text(20, 20, 'Text')
        # Add the patch to the Axes
        ax.add_patch(rect)
        plt.show()
        '''

        time_elapsed.append(time.time()- start_time)
        #bclass = labels_map[int(obj[1])] if labels_map else str(int(obj[1]))
print(f"Infer mean time = {np.mean(time_elapsed)}")
results = {"Mean time": np.mean(time_elapsed), "Total time": np.sum(time_elapsed), "Results": results}

with open(Path(__file__).parent/f"mobilenet_results_{device}.json", 'w') as json_file:
    json.dump(results, json_file, indent=4)
                