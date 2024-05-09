# ready to run example: PythonClient/multirotor/hello_drone.py
import airsim
import os
import time

# connect to the AirSim simulator
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# # Async methods returns Future. Call join() to wait for task to complete.
# client.takeoffAsync().join()
# client.moveToPositionAsync(0, 0, 1.0, 1.0).join()

# take images
while (True):
    start_time = time.time()
    responses = client.simGetImages([
        airsim.ImageRequest("front_center_custom", airsim.ImageType.DepthPlanar, True)], vehicle_name="drone_1")
    print("time: %s" % (time.time() - start_time))



