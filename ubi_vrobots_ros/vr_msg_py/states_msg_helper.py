from .R000_states_generated import *
import json


class Vec4(Vec4MsgT):
    def __init__(self, instance: Vec4MsgT = None):
        super().__init__()
        if instance is None:
            self.x = 0
            self.y = 0
            self.z = 0
            self.w = 0
        else:
            self.x = instance.x
            self.y = instance.y
            self.z = instance.z
            self.w = instance.w

    def get_dict_data(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
            "w": self.w,
        }

    def __str__(self) -> str:
        return json.dumps(self.get_dict_data(), indent=4, sort_keys=True)


class Vec3(Vec3MsgT):
    def __init__(self, instance: Vec3MsgT = None):
        super().__init__()
        if instance is None:
            self.x = 0
            self.y = 0
            self.z = 0
        else:
            self.x = instance.x
            self.y = instance.y
            self.z = instance.z

    def get_dict_data(self):
        return {
            "x": self.x,
            "y": self.y,
            "z": self.z,
        }

    def __str__(self) -> str:
        return json.dumps(self.get_dict_data(), indent=4, sort_keys=True)

class Collision(CollisionT):
    def __init__(self, instance: CollisionT = None):
        super().__init__()
        if instance is None:
            self.timestamp = 0.0
            self.collision_type = 0
            self.objectName = ""
            self.pos = Vec3()
            self.vel = Vec3()
            self.eul = Vec3()
            self.angvel = Vec3()
        else:
            self.timestamp = instance.timestamp
            self.collision_type = instance.collision_type
            self.objectName = instance.objectName.decode("utf-8")
            self.pos = Vec3(instance.pos)
            self.vel = Vec3(instance.vel)
            self.eul = Vec3(instance.eul)
            self.angvel = Vec3(instance.angvel)

    def get_dict_data(self):
        return {
            "timestamp": self.timestamp,
            "collision_type": self.collision_type,
            "object_name": self.objectName,
            "pos": self.pos.get_dict_data(),
            "vel": self.vel.get_dict_data(),
            "eul": self.eul.get_dict_data(),
            "angvel": self.angvel.get_dict_data(),
        }

    def __str__(self) -> str:
        return json.dumps(self.get_dict_data(), indent=4, sort_keys=True)        
        

class VRobotState(StatesMsgT):
    def __init__(self, instance: StatesMsgT = None) -> None:
        if instance is None:
            self.name = ""
            self.sysId = 0
            self.timestamp = 0.0
            self.linAcc = Vec3()
            self.linVel = Vec3()
            self.linPos = Vec3()
            self.altitude = 0.0
            self.angAcc = Vec3()
            self.angVel = Vec3()
            self.euler = Vec3()
            self.eulerDot = Vec3()
            self.quaternion = Vec4()
            self.pwm = []
            self.actuators = []
            self.force = Vec3()
            self.torque = Vec3()
            self.accelerometer = Vec3()
            self.gyroscope = Vec3()
            self.magnetometer = Vec3()
            self.barometer = 0.0
            self.temperature = 0.0
            self.gpsPos = Vec3()
            self.gpsVel = Vec3()
            self.mass = 0.0
            self.cg = Vec3()
            self.momentArms = []
            self.moi3x1 = Vec3()
            self.moi3x3 = []
            self.extraProps = []
            self.collisions = []
            self.imageData0 = []
            self.imageData1 = []

        else:
            self.name = instance.name.decode("utf-8")
            self.sysId = instance.sysId
            self.timestamp = instance.timestamp
            self.linAcc = Vec3(instance.linAcc)
            self.linVel = Vec3(instance.linVel)
            self.linPos = Vec3(instance.linPos)
            self.altitude = instance.altitude
            self.angAcc = Vec3(instance.angAcc)
            self.angVel = Vec3(instance.angVel)
            self.euler = Vec3(instance.euler)
            self.eulerDot = Vec3(instance.eulerDot)
            self.quaternion = Vec4(instance.quaternion)
            self.pwm = instance.pwm
            self.actuators = instance.actuators
            self.force = Vec3(instance.force)
            self.torque = Vec3(instance.torque)
            self.accelerometer = Vec3(instance.accelerometer)
            self.gyroscope = Vec3(instance.gyroscope)
            self.magnetometer = Vec3(instance.magnetometer)
            self.barometer = instance.barometer
            self.temperature = instance.temperature
            self.gpsPos = Vec3(instance.gpsPos)
            self.gpsVel = Vec3(instance.gpsVel)
            self.mass = instance.mass
            self.cg = Vec3(instance.cg)
            self.momentArms = instance.momentArms
            self.moi3x1 = Vec3(instance.moi3x1)
            self.moi3x3 = instance.moi3x3
            self.extraProps = instance.extraProps
            self.collisions = instance.collisions
            self.imageData0 = instance.imageData0
            self.imageData1 = instance.imageData1


    def get_dict_data(self):
        return {
            "name": self.name,
            "sys_id": self.sysId,
            "timestamp": self.timestamp,
            "lin_acc": self.linAcc.get_dict_data(),
            "lin_vel": self.linVel.get_dict_data(),
            "lin_pos": self.linPos.get_dict_data(),
            "altitude": self.altitude,
            "ang_acc": self.angAcc.get_dict_data(),
            "ang_vel": self.angVel.get_dict_data(),
            "euler": self.euler.get_dict_data(),
            "euler_dot": self.eulerDot.get_dict_data(),
            "quaternion": self.quaternion.get_dict_data(),
            "pwm": self.pwm if self.pwm is not None else [],
            "actuators": self.actuators if self.actuators is not None else [],
            "force": self.force.get_dict_data(),
            "torque": self.torque.get_dict_data(),
            "accelerometer": self.accelerometer.get_dict_data(),
            "gyroscope": self.gyroscope.get_dict_data(),
            "magnetometer": self.magnetometer.get_dict_data(),
            "barometer": self.barometer,
            "temperature": self.temperature,
            "gps_pos": self.gpsPos.get_dict_data(),
            "gps_vel": self.gpsVel.get_dict_data(),
            "mass": self.mass,
            "cg": self.cg.get_dict_data(),
            "moment_arms": self.momentArms if self.momentArms is not None else [],
            "moi_3x1": self.moi3x1.get_dict_data(),
            "moi_3x3": self.moi3x3 if self.moi3x3 is not None else [],
            "extra_props": self.extraProps if self.extraProps is not None else [],
            "collisions": self.collisions if self.collisions is not None else [],
            "image_data_0": self.imageData0 if self.imageData0 is not None else [],
            "image_data_1": self.imageData1 if self.imageData1 is not None else [],

        }

    def __str__(self) -> str:
        return json.dumps(self.get_dict_data(), indent=4, sort_keys=True)
