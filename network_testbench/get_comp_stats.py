from importlib.abc import Loader
import json
from unittest import loader

results_file_m = "/home/aitor/repos/TFM/network_testbench/mobilenet_results_MYRIAD.json"
results_file = "/home/aitor/repos/TFM/network_testbench/mobilenet_results_MYRIAD_comp.json"
reference_file = "/home/aitor/datasets/annotations_trainval2017/annotations/instances_val2017.json"

with open(results_file, 'r') as f:
    results = json.load(f)

with open(results_file_m, 'r') as f:
    results_m = json.load(f)

with open(reference_file, 'r') as r:
    reference = json.load(r)
categories = reference['categories']
reference = reference['annotations']

positives = results['positive']
false_negatice = results['false_negative']