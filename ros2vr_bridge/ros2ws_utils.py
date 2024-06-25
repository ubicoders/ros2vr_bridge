import flatbuffers
from ros2vr_interface.msg import VRobotCMD
from ros2vr_bridge.vr_msg_py.C000_commands_generated import CommandMsgT, Vec3MsgT, Vec4MsgT
import time
CMDS_DICT = {}

def Tobj2bytes(msgT, fid) -> bytearray:
    builder = flatbuffers.Builder(512)
    os = msgT.Pack(builder)
    builder.Finish(os, bytes(fid, "utf-8"))
    return builder.Output()

def serialize_commands(cmdMsgT) -> bytearray:
    return Tobj2bytes(cmdMsgT, "CMD0")

async def send_cmd_msg(websocket):
    keys = CMDS_DICT.keys()
    for key in keys:
        pair = CMDS_DICT[key]
        updated = pair["updated"]
        ba = pair["msg"]
        if (updated == True):
            await websocket.send(ba)
            CMDS_DICT[key]["updated"] = False
    


def pack_rosmsg_to_fb_ba(ros_cmd_msg:VRobotCMD)->bytearray:
    fb_cmdMsgT = CommandMsgT()
    fb_cmdMsgT.timestamp = time.time()*1000
    fb_cmdMsgT.cmdId = ros_cmd_msg.cmd_id
    fb_cmdMsgT.sysId = ros_cmd_msg.sys_id
    fb_cmdMsgT.intVal =  int(ros_cmd_msg.int_val)
    fb_cmdMsgT.floatVal = ros_cmd_msg.float_val*1.0
    fb_cmdMsgT.intArr = ros_cmd_msg.int_arr
    fb_cmdMsgT.floatArr = ros_cmd_msg.float_arr
    
    fb_cmdMsgT.vec3 = Vec3MsgT()
    fb_cmdMsgT.vec3.x = ros_cmd_msg.vec3.x
    fb_cmdMsgT.vec3.y = ros_cmd_msg.vec3.y
    fb_cmdMsgT.vec3.z = ros_cmd_msg.vec3.z

    fb_cmdMsgT.vec4 = Vec4MsgT()
    fb_cmdMsgT.vec4.x = ros_cmd_msg.vec4.x
    fb_cmdMsgT.vec4.y = ros_cmd_msg.vec4.y
    fb_cmdMsgT.vec4.z = ros_cmd_msg.vec4.z
    fb_cmdMsgT.vec4.w = ros_cmd_msg.vec4.w

    # iterate vec3_arr
    for (vec3) in ros_cmd_msg.vec3_arr:
        fb_cmdMsgT.vec3Arr.append(Vec3MsgT(vec3.x, vec3.y, vec3.z))
    
    # iterate vec4_arr
    for (vec4) in ros_cmd_msg.vec4_arr:
        fb_cmdMsgT.vec4Arr.append(Vec4MsgT(vec4.x, vec4.y, vec4.z, vec4.w))

    # return
    sysId = ros_cmd_msg.sys_id
    ba = serialize_commands(fb_cmdMsgT)    
    return [sysId, ba]


#ros2 topic pub --rate 50 /vrobot_cmd_pub_0 ubi_vrobots_interface/msg/VRobotCMD   "{sys_id: 0, cmd_id: 303, int_arr:[1, 0, 0, 1], float_arr: [5.0, 0.0, 0.0, 2.0]}"
