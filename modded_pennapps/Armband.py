import myo
from myo.lowlevel import pose_t, stream_emg
from myo.six import print_
import random
import math

class Collector(myo.DeviceListener):
    def __init__(self):
        self.myo = None
        self.on_arm = False
        self.which_arm = None
        self.is_unlocked = False
        self.roll_w = 0
        self.pitch_w = 0
        self.yaw_w = 0
        self.current_pose = None
        self.one_shot = False
        self.all_shots = False

    def on_unpair(self, myo, timestamp):
        self.roll_w = 0
        self.pitch_w = 0
        self.yaw_w = 0.0

    def on_orientation_data(self, myo, timestamp, quat):
        roll =math.atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]),
                    1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (quat[0] * quat[2] - quat[3] * quat[1]))))
        yaw = math.atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]),
                          1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]))
        self.roll_w = ((roll + math.pi)/(2*math.pi) * 360)
        self.pitch_w = ((pitch + math.pi)/(2*math.pi) * 360)
        self.yaw_w = ((yaw + math.pi)/(2*math.pi) * 360)

    def on_pose(self, myo, timestamp, pose):
        print pose
        self.current_pose = pose
        pass

    def on_arm_sync(self, myo, timestamp, arm, x_direction):
        self.on_arm = True
        self.which_arm = arm

    def on_arm_unsync(self, myo, timestamp):
        self.on_arm = False

    def on_sync(self, myo, timestamp, arg4, arg5):
        self.myo = myo
        print "synced!"
        pass

    def on_unsync(self, myo, timestamp):
        #idk even more
        pass

    def on_unlock(self, myo, timestamp):
        self.is_unlocked = True

    def on_lock(self, myo, timestamp):
        self.is_unlocked = False

class Armband:
    def __init__(self):
        myo.init()
        self.collector = Collector()
        self.hub = myo.Hub()
        self.hub.set_locking_policy(myo.locking_policy.none)
        self.hub.run(1000/30, self.collector)

