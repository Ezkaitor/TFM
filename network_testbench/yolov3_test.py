#!/usr/bin/env python
import os
import sys
from pathlib import Path
from math import exp as exp
import numpy as np
import time

from tqdm import tqdm
import json

import cv2 as cv
from openvino.inference_engine import IECore

import warnings
warnings.filterwarnings("ignore") 


class YoloParams:
    # ------------------------------------------- Extracting layer parameters ------------------------------------------
    # Magic numbers are copied from yolo samples
    def __init__(self, param, side):
        self.num = 3 if 'num' not in param else int(param['num'])
        self.coords = 4 if 'coords' not in param else int(param['coords'])
        self.classes = 80 if 'classes' not in param else int(param['classes'])
        self.side = side
        self.anchors = [10.0, 13.0, 16.0, 30.0, 33.0, 23.0, 30.0, 61.0, 62.0, 45.0, 59.0, 119.0, 116.0, 90.0, 156.0,
                        198.0,
                        373.0, 326.0] if 'anchors' not in param else [float(a) for a in param['anchors'].split(',')]

        self.isYoloV3 = False

        if param.get('mask'):
            mask = [int(idx) for idx in param['mask'].split(',')]
            self.num = len(mask)

            maskedAnchors = []
            for idx in mask:
                maskedAnchors += [self.anchors[idx * 2], self.anchors[idx * 2 + 1]]
            self.anchors = maskedAnchors

            self.isYoloV3 = True # Weak way to determine but the only one.

def entry_index(side, coord, classes, location, entry):
    side_power_2 = side ** 2
    n = location // side_power_2
    loc = location % side_power_2
    return int(side_power_2 * (n * (coord + classes + 1) + entry) + loc)


def scale_bbox(x, y, h, w, class_id, confidence, h_scale, w_scale):
    xmin = int((x - w / 2) * w_scale)
    ymin = int((y - h / 2) * h_scale)
    xmax = int(xmin + w * w_scale)
    ymax = int(ymin + h * h_scale)
    return dict(xmin=xmin, xmax=xmax, ymin=ymin, ymax=ymax, class_id=class_id, confidence=confidence)


def parse_yolo_region(blob, resized_image_shape, original_im_shape, params, threshold):
    # ------------------------------------------ Validating output parameters ------------------------------------------
    _, _, out_blob_h, out_blob_w = blob.shape
    assert out_blob_w == out_blob_h, "Invalid size of output blob. It sould be in NCHW layout and height should " \
                                     "be equal to width. Current height = {}, current width = {}" \
                                     "".format(out_blob_h, out_blob_w)

    # ------------------------------------------ Extracting layer parameters -------------------------------------------
    orig_im_h, orig_im_w = original_im_shape
    resized_image_h, resized_image_w = resized_image_shape
    objects = list()
    predictions = blob.flatten()
    side_square = params.side * params.side

    # ------------------------------------------- Parsing YOLO Region output -------------------------------------------
    for i in range(side_square):
        row = i // params.side
        col = i % params.side
        for n in range(params.num):
            obj_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, params.coords)
            scale = predictions[obj_index]
            if scale < threshold:
                continue
            box_index = entry_index(params.side, params.coords, params.classes, n * side_square + i, 0)
            # Network produces location predictions in absolute coordinates of feature maps.
            # Scale it to relative coordinates.
            x = (col + predictions[box_index + 0 * side_square]) / params.side
            y = (row + predictions[box_index + 1 * side_square]) / params.side
            # Value for exp is very big number in some cases so following construction is using here
            try:
                w_exp = exp(predictions[box_index + 2 * side_square])
                h_exp = exp(predictions[box_index + 3 * side_square])
            except OverflowError:
                continue
            # Depends on topology we need to normalize sizes by feature maps (up to YOLOv3) or by input shape (YOLOv3)
            w = w_exp * params.anchors[2 * n] / (resized_image_w if params.isYoloV3 else params.side)
            h = h_exp * params.anchors[2 * n + 1] / (resized_image_h if params.isYoloV3 else params.side)
            for j in range(params.classes):
                class_index = entry_index(params.side, params.coords, params.classes, n * side_square + i,
                                          params.coords + 1 + j)
                confidence = scale * predictions[class_index]
                if confidence < threshold:
                    continue
                objects.append(scale_bbox(x=x, y=y, h=h, w=w, class_id=j, confidence=confidence,
                                          h_scale=orig_im_h, w_scale=orig_im_w))
    return objects


def intersection_over_union(box_1, box_2):
    width_of_overlap_area = min(box_1['xmax'], box_2['xmax']) - max(box_1['xmin'], box_2['xmin'])
    height_of_overlap_area = min(box_1['ymax'], box_2['ymax']) - max(box_1['ymin'], box_2['ymin'])
    if width_of_overlap_area < 0 or height_of_overlap_area < 0:
        area_of_overlap = 0
    else:
        area_of_overlap = width_of_overlap_area * height_of_overlap_area
    box_1_area = (box_1['ymax'] - box_1['ymin']) * (box_1['xmax'] - box_1['xmin'])
    box_2_area = (box_2['ymax'] - box_2['ymin']) * (box_2['xmax'] - box_2['xmin'])
    area_of_union = box_1_area + box_2_area - area_of_overlap
    if area_of_union == 0:
        return 0
    return area_of_overlap / area_of_union


def main():

    # ------------- 1. Plugin initialization for specified device and load extensions library if specified -------------
    print("Creating Inference Engine...")
    ie = IECore()

    # -------------------- 2. Reading the IR generated by the Model Optimizer (.xml and .bin files) --------------------
    print("Loading network")

    ncs_model_path = Path("/home/aitorezkerra/repos/TFM/drone-test/NCS_models/yolo-v3-tf")

    net = ie.read_network(str(ncs_model_path/(ncs_model_path.name+".xml")), str(ncs_model_path/(ncs_model_path.name+".bin"))) # TODO: make it realtive path

    if "MYRIAD" in ie.available_devices: device="MYRIAD"
    else:
        device="CPU"
        print("MYRIAD device is not plugged.")


    # ---------------------------------- 3. Load CPU extension for support specific layer ------------------------------
    if device == 'CPU':
        supported_layers = ie.query_network(net, "CPU")
        not_supported_layers = [l for l in net.layers.keys() if l not in supported_layers]
        if len(not_supported_layers) != 0:
            print("ERROR: Following layers are not supported by the plugin for specified device {}:\n {}".
                      format(args.device, ', '.join(not_supported_layers)))
            print("ERROR: Please try to specify cpu extensions library path in sample's command line parameters using -l "
                      "or --cpu_extension command line argument")
            sys.exit(1)

    assert len(net.inputs.keys()) == 1, "Sample supports only YOLO V3 based single input topologies"

    # ---------------------------------------------- 4. Preparing inputs -----------------------------------------------
    print("Preparing inputs")
    input_blob = next(iter(net.inputs))

    #  Defaulf batch_size is 1
    net.batch_size = 1

    # Read and pre-process input images
    n, c, h, w = net.inputs[input_blob].shape


    # ----------------------------------------- 5. Loading model to the plugin -----------------------------------------
    print("Loading model to the plugin")
    exec_net = ie.load_network(network=net, num_requests=2, device_name=device)

    # ----------------------------------------------- 6. Doing inference -----------------------------------------------
    print("Starting inference...")
    dataset_path = Path("/home/aitorezkerra/datasets/val2017")
    #dataset_path = Path("/home/aitorezkerra/datasets/val2017_test")

    results = []
    time_elapsed = []
    for image_path in tqdm(dataset_path.iterdir()):
        image_id = int(image_path.stem.lstrip('0'))
        image_raw = cv.imread(str(image_path))
        image_size = image_raw.shape

        # Start inference
        start_time = time.time()
        image = cv.resize(image_raw, (w,h))
        image = image.transpose((2,0,1)) # Change data layout from HWC to CHW
        image = image.reshape((n, c, h, w))
    
        # Infer
        output = exec_net.infer(inputs={input_blob: [image]})

        # Collecting object detection results
        objects = list()
    
            
        for layer_name, out_blob in output.items():
            out_blob = out_blob.reshape(net.layers[net.layers[layer_name].parents[0]].out_data[0].shape)
            layer_params = YoloParams(net.layers[layer_name].params, out_blob.shape[2])
            #print("Layer {} parameters: ".format(layer_name))
            objects += parse_yolo_region(out_blob, image.shape[2:],
                                            image_size[:-1], layer_params,
                                            0.1)

        # Filtering overlapping boxes with respect to the --iou_threshold CLI parameter
        objects = sorted(objects, key=lambda obj : obj['confidence'], reverse=True)

        '''
        for i in range(len(objects)):
            if objects[i]['confidence'] == 0:
                continue
            for j in range(i + 1, len(objects)):
                if intersection_over_union(objects[i], objects[j]) > 0.2:
                    objects[j]['confidence'] = 0
        '''

        for obj in objects:
            
            # Draw only objects when probability more than specified threshold
            
            result_dict = {'image_id': image_id}
            result_dict["category_id"]= obj['class_id']
            result_dict["bbox"] = [obj['xmin'], obj['ymin'], obj['xmax']-obj['xmin'], obj['ymax']-obj['ymin']]
            result_dict["score"] = obj['confidence']
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
        


if __name__ == '__main__':
    sys.exit(main() or 0)