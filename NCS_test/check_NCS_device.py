from openvino.inference_engine import IECore
ie=IECore()
devices = ie.available_devices
print("##############################")
if "MYRIAD" in devices:
    print("### NCS has been detected! ###")
else:
    print("## No NCS has ben detected. ##")
print("##############################")