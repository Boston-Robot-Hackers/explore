def main():
    import depthai as dai
    print("started...")
    print("DepthAI version:", dai.__version__)

    # depthai 2.x API for OAK-D-Lite (RVC2)
    pipeline = dai.Pipeline()

    mono = pipeline.createMonoCamera()    
    mono.setBoardSocket(dai.CameraBoardSocket.LEFT)
    xout = pipeline.createXLinkOut()
    xout.setStreamName("left")
    mono.out.link(xout.input)

    with dai.Device(pipeline) as device:
        print("Device connected:", device.getMxId())
        queue = device.getOutputQueue(name="left")
        frame = queue.get()
        imOut = frame.getCvFrame()

    # with dai.Device(pipeline) as device:
    #     print("Device connected:", device.getMxId())
    #     q = device.getOutputQueue("left", maxSize=1, blocking=False)
    #     frame = q.get()
    #     print("Got frame:", frame.getCvFrame().shape)

    print("exited...")
