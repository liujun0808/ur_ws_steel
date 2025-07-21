from jodellSdk.jodellSdkDemo import ClawEpgTool
import time

class jodell_gripper():
    def __init__(self) -> None:
        self.gripper = ClawEpgTool()
        self.gripper.serialOperation('/dev/ttyUSB0',115200,True)
        self.gripper.clawEnable(9,True)
        print("Activating gripper...")
        # self.close_gripper()
        # time.sleep(2)
        # self.open_gripper()

    def log_gripper_info(self):
        print(f"Pos: {str(self.gripper.getClawCurrentLocation(9))}")

    def getPos(self):
        pos = self.gripper.getClawCurrentLocation(9)
        return pos

    def close_gripper(self):
        self.gripper.runWithoutParam(9,2)
        print("gripper had closed!")
        time.sleep(3)
        self.log_gripper_info()

    def open_gripper(self):
        self.gripper.runWithoutParam(9,1)
        print("gripper had opened!")
        # self.log_gripper_info()

test = jodell_gripper()
# time.sleep(1)
time.sleep(10)
test.open_gripper()
# test.close_gripper()


# pos = test.getPos()
# distance = pos[0] * (-0.2021) + 51.53
# print(distance)
