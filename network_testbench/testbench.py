import json
from tqdm import tqdm
import cv2 as cv
from pathlib import Path

## COCO Classes ##
labels_map = {0: '__background__', 1: 'person', 2: 'bicycle', 3: 'car', 4: 'motorcycle', 5: 'airplane', 6: 'bus', 7: 'train', 8: 'truck', 9: 'boat',10: 'traffic light',11: 'fire hydrant',12: 'stop sign',13: 'parking meter',14: 'bench',15: 'bird',16: 'cat',17: 'dog',18: 'horse',19: 'sheep',20: 'cow',21: 'elephant',22: 'bear',23: 'zebra',24: 'giraffe',25: 'backpack',26: 'umbrella',27: 'handbag',28: 'tie',29: 'suitcase',30: 'frisbee',31: 'skis',32: 'snowboard',33: 'sports ball',34: 'kite',35: 'baseball bat',36: 'baseball glove',37: 'skateboard',38: 'surfboard',39: 'tennis racket',40: 'bottle',41: 'wine glass',42: 'cup',43: 'fork',44: 'knife',45: 'spoon',46: 'bowl',47: 'banana',48: 'apple',49: 'sandwich',50: 'orange',51: 'broccoli',52: 'carrot',53: 'hot dog',54: 'pizza',55: 'donut',56: 'cake',57: 'chair',58: 'couch',59: 'potted plant',60: 'bed',61: 'dining table',62: 'toilet',63: 'tv',64: 'laptop',65: 'mouse',66: 'remote',67: 'keyboard',68: 'cell phone',69: 'microwave',70: 'oven',71: 'toaster',72: 'sink',73: 'refrigerator',74: 'book',75: 'clock',76: 'vase',77: 'scissors',78: 'teddy bear',79: 'hair drier',80: 'toothbrush'}

home_path = Path.home()
results_file = home_path / "repos/TFM/network_testbench/mobilenet_results_MYRIAD.json"
reference_file = home_path / "datasets/annotations_trainval2017/annotations/instances_val2017.json"
images_dir = home_path / "datasets/val2017"

with open(results_file, 'r') as r:
    results = json.load(r)
    results = results['Results']
with open(reference_file, 'r') as r:
    reference = json.load(r)
categories = reference['categories']
reference = reference['annotations']


false_positive = []
positive_positive = []
not_sure = []
no_ground_truth = []
for idx, result in tqdm(enumerate(results)):
    for cat in categories:
        if result['category_id'] == cat['id']: tag = cat['name']
    for ref in reference:
        if result['image_id'] == ref['image_id']:
            if result['category_id'] == ref['category_id']:
                if result['bbox'][0] > ref['bbox'][0]:
                    int_width = ref['bbox'][0]+ref['bbox'][2] - result['bbox'][0]
                else:
                    int_width =  result['bbox'][0] + result['bbox'][2] - ref['bbox'][0]
                
                if result['bbox'][1] > ref['bbox'][1]:
                    int_height = ref['bbox'][1]+ref['bbox'][3] - result['bbox'][1]
                else:
                    int_height = result['bbox'][1] + result['bbox'][3] - ref['bbox'][1]
                
                if int_width <= 0 or int_height <= 0:
                    continue
                intersection = int_width*int_height
                

                union = result['bbox'][2]*result['bbox'][3] + ref['bbox'][2]*ref['bbox'][3] - intersection
                if union <=0: continue

                IoU = 100*intersection/union
                if result['score']*100 > 50:
                    if IoU > 60:
                        positive_positive.append({
                            'index': idx,
                            'category': result['category_id'],
                            'name': tag,
                            'probability': result['score'],
                            'IoU': IoU,
                        })
                    
                    else:
                        false_positive.append({
                            'index': idx,
                            'category': result['category_id'],
                            'name': tag,
                            'probability': result['score'],
                            'IoU': IoU,
                        })
                else:
                    not_sure.append({
                            'index': idx,
                            'category': result['category_id'],
                            'name': tag,
                            'probability': result['score'],
                            'IoU': IoU,
                        })

                    '''
                    ### Debugging ###
                    image = cv.imread(images_dir + "/" + "0"*(12-len(str(result['image_id'])))+str(result['image_id'])+".jpg")
                    start_point = (int(ref['bbox'][0]), int(ref['bbox'][1]))
                    end_point = (int(ref['bbox'][0]) + int(ref['bbox'][2]), int(ref['bbox'][1]) + int(ref['bbox'][3]))
                    color = (0,0,255)
                    image = cv.rectangle(image, start_point, end_point, color, 3)

                    start_point = (int(result['bbox'][0]), int(result['bbox'][1]))
                    end_point = (int(result['bbox'][0]) + int(result['bbox'][2]), int(result['bbox'][1]) + int(result['bbox'][3]))
                    color = (0,255,0)
                    image = cv.rectangle(image, start_point, end_point, color, 3)
                    
                    for cat in categories:
                        if ref['category_id'] == cat['id']: print(cat['name'])
                    cv.imshow("Image", image)
                    cv.waitKey(0)
                    cv.destroyAllWindows()
                    ###################
                    '''
            break
    no_ground_truth.append({
                        'index': idx,
                        'category': result['category_id'],
                        'name': tag,
                        'probability': result['score'],
                        'IoU': 0,
                        })
comparison = {'positive':positive_positive, 'false_positive':false_positive, 'not_sure':not_sure, 'no_ground_truth':no_ground_truth}
with open(results_file.strip('.json')+'_comp.json', 'w') as json_file:
    json.dump(comparison, json_file, indent=4)
