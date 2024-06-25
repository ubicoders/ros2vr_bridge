from typing import List
from geometry_msgs.msg import Vector3, Quaternion
from .vr_msg_py.R000_states_generated import StatesMsg, StatesMsgT, Vec3MsgT, Vec4MsgT
from .vr_msg_py.collision_generated import CollisionT
from ros2vr_interface.msg import VRobotStates, VRobotCMD, Collision
from std_msgs.msg import Header

def create_vector4(vec4MsgT:Vec4MsgT):
    return Quaternion(x=vec4MsgT.x*1.0  or 0.0,
                y=vec4MsgT.y*1.0 or 0.0,
                z=vec4MsgT.z*1.0 or 0.0,
                w=vec4MsgT.w*1.0 or 0.0 )

def create_vector3(vec3MsgT:Vec3MsgT):
    return Vector3(x=vec3MsgT.x*1.0  or 0.0,
                y=vec3MsgT.y*1.0 or 0.0,
                z=vec3MsgT.z*1.0 or 0.0)

def ros2msg_packer(_statesMsgT:StatesMsgT):
    seconds = int(_statesMsgT.timestamp/1000)
    nanoseconds = int((_statesMsgT.timestamp - seconds*1000)*1e6) 
    
    msg = VRobotStates()
    msg.header = Header()
    msg.header.stamp.sec = seconds
    msg.header.stamp.nanosec = nanoseconds
    msg.header.frame_id = 'default'
    msg.name = _statesMsgT.name
    msg.sys_id = _statesMsgT.sysId
    msg.lin_acc = create_vector3(_statesMsgT.linAcc)
    msg.lin_vel = create_vector3(_statesMsgT.linVel)
    msg.lin_pos = create_vector3(_statesMsgT.linPos)
    msg.altitude = _statesMsgT.altitude*1.0 if _statesMsgT.altitude is not None else 0.0

    msg.ang_acc = create_vector3(_statesMsgT.angAcc)
    msg.ang_vel = create_vector3(_statesMsgT.angVel)
    msg.euler = create_vector3(_statesMsgT.euler)
    msg.euler_dot = create_vector3(_statesMsgT.eulerDot)
    msg.quaternion = create_vector4(_statesMsgT.quaternion)

    msg.pwm = [ int( pwm) for pwm in _statesMsgT.pwm ]
    msg.actuators = [ act*1.0 for act in _statesMsgT.actuators ]
    msg.force = create_vector3(_statesMsgT.force)
    msg.torque = create_vector3(_statesMsgT.torque)

    msg.accelerometer = create_vector3(_statesMsgT.accelerometer)
    msg.gyroscope = create_vector3(_statesMsgT.gyroscope)
    msg.magnetometer = create_vector3(_statesMsgT.magnetometer)
    msg.barometer = _statesMsgT.barometer
    msg.gps_pos = create_vector3(_statesMsgT.gpsPos)
    msg.gps_vel = create_vector3(_statesMsgT.gpsVel)

    msg.mass = _statesMsgT.mass*1.0
    msg.cg = create_vector3(_statesMsgT.cg)
    msg.moment_arms = [ arm*1.0 for arm in _statesMsgT.momentArms ]
    msg.moi_3x1 = Vector3(x=_statesMsgT.moi3x1.x, y=_statesMsgT.moi3x1.y, z=_statesMsgT.moi3x1.z)
    msg.moi_3x3 = [value*1.0 for value in _statesMsgT.moi3x3]
    msg.extra_props = [value *1.0 for value in _statesMsgT.extraProps]

    msg.collisions = []  
    orig_collisions:List[CollisionT] = _statesMsgT.collisions
    for collision in orig_collisions:
        msgc= Collision()
        msgc.header = Header()
        msgc.header.stamp.sec = int(collision.timestamp/1000)
        msgc.header.stamp.nanosec = int((collision.timestamp - msgc.header.stamp.sec*1000)*1e6)
        # change binary string to str
        msgc.object_name =  collision.objectName.decode('utf-8')
        msgc.collision_type = collision.collisionType

        msgc.pos =create_vector3(collision.pos)
        msgc.vel = create_vector3(collision.vel)
        msgc.eul = create_vector3(collision.eul)
        msgc.angvel = create_vector3(collision.angvel)
        msg.collisions.append(msgc)
    
    return msg