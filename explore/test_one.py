import depthai as dai

pipeline = dai.Pipeline()

cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(300, 300)
cam.setInterleaved(False)

xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("preview")
cam.preview.link(xout.input)

print("Connecting to device...")
with dai.Device(pipeline) as device:
    print("Connected.")
    print("USB speed:", device.getUsbSpeed())
    print("Device name:", device.getDeviceName())
    print("MxID:", device.getMxId())

    q = device.getOutputQueue("preview", maxSize=1, blocking=False)

    for _ in range(10):
        frame = q.get().getCvFrame()
        print("Got frame:", frame.shape)
