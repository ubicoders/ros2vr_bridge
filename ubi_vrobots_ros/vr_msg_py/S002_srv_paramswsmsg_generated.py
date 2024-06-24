# automatically generated by the FlatBuffers compiler, do not modify

# namespace: 

import flatbuffers
from flatbuffers.compat import import_numpy
np = import_numpy()

class Vec3Msg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAs(cls, buf, offset=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = Vec3Msg()
        x.Init(buf, n + offset)
        return x

    @classmethod
    def GetRootAsVec3Msg(cls, buf, offset=0):
        """This method is deprecated. Please switch to GetRootAs."""
        return cls.GetRootAs(buf, offset)
    @classmethod
    def Vec3MsgBufferHasIdentifier(cls, buf, offset, size_prefixed=False):
        return flatbuffers.util.BufferHasIdentifier(buf, offset, b"\x53\x30\x30\x32", size_prefixed=size_prefixed)

    # Vec3Msg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Vec3Msg
    def X(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

    # Vec3Msg
    def Y(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

    # Vec3Msg
    def Z(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

def Vec3MsgStart(builder):
    builder.StartObject(3)

def Vec3MsgAddX(builder, x):
    builder.PrependFloat32Slot(0, x, 0.0)

def Vec3MsgAddY(builder, y):
    builder.PrependFloat32Slot(1, y, 0.0)

def Vec3MsgAddZ(builder, z):
    builder.PrependFloat32Slot(2, z, 0.0)

def Vec3MsgEnd(builder):
    return builder.EndObject()



class Vec3MsgT(object):

    # Vec3MsgT
    def __init__(self):
        self.x = 0.0  # type: float
        self.y = 0.0  # type: float
        self.z = 0.0  # type: float

    @classmethod
    def InitFromBuf(cls, buf, pos):
        vec3Msg = Vec3Msg()
        vec3Msg.Init(buf, pos)
        return cls.InitFromObj(vec3Msg)

    @classmethod
    def InitFromPackedBuf(cls, buf, pos=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, pos)
        return cls.InitFromBuf(buf, pos+n)

    @classmethod
    def InitFromObj(cls, vec3Msg):
        x = Vec3MsgT()
        x._UnPack(vec3Msg)
        return x

    # Vec3MsgT
    def _UnPack(self, vec3Msg):
        if vec3Msg is None:
            return
        self.x = vec3Msg.X()
        self.y = vec3Msg.Y()
        self.z = vec3Msg.Z()

    # Vec3MsgT
    def Pack(self, builder):
        Vec3MsgStart(builder)
        Vec3MsgAddX(builder, self.x)
        Vec3MsgAddY(builder, self.y)
        Vec3MsgAddZ(builder, self.z)
        vec3Msg = Vec3MsgEnd(builder)
        return vec3Msg


class Vec4Msg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAs(cls, buf, offset=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = Vec4Msg()
        x.Init(buf, n + offset)
        return x

    @classmethod
    def GetRootAsVec4Msg(cls, buf, offset=0):
        """This method is deprecated. Please switch to GetRootAs."""
        return cls.GetRootAs(buf, offset)
    @classmethod
    def Vec4MsgBufferHasIdentifier(cls, buf, offset, size_prefixed=False):
        return flatbuffers.util.BufferHasIdentifier(buf, offset, b"\x53\x30\x30\x32", size_prefixed=size_prefixed)

    # Vec4Msg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # Vec4Msg
    def X(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

    # Vec4Msg
    def Y(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

    # Vec4Msg
    def Z(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

    # Vec4Msg
    def W(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float32Flags, o + self._tab.Pos)
        return 0.0

def Vec4MsgStart(builder):
    builder.StartObject(4)

def Vec4MsgAddX(builder, x):
    builder.PrependFloat32Slot(0, x, 0.0)

def Vec4MsgAddY(builder, y):
    builder.PrependFloat32Slot(1, y, 0.0)

def Vec4MsgAddZ(builder, z):
    builder.PrependFloat32Slot(2, z, 0.0)

def Vec4MsgAddW(builder, w):
    builder.PrependFloat32Slot(3, w, 0.0)

def Vec4MsgEnd(builder):
    return builder.EndObject()



class Vec4MsgT(object):

    # Vec4MsgT
    def __init__(self):
        self.x = 0.0  # type: float
        self.y = 0.0  # type: float
        self.z = 0.0  # type: float
        self.w = 0.0  # type: float

    @classmethod
    def InitFromBuf(cls, buf, pos):
        vec4Msg = Vec4Msg()
        vec4Msg.Init(buf, pos)
        return cls.InitFromObj(vec4Msg)

    @classmethod
    def InitFromPackedBuf(cls, buf, pos=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, pos)
        return cls.InitFromBuf(buf, pos+n)

    @classmethod
    def InitFromObj(cls, vec4Msg):
        x = Vec4MsgT()
        x._UnPack(vec4Msg)
        return x

    # Vec4MsgT
    def _UnPack(self, vec4Msg):
        if vec4Msg is None:
            return
        self.x = vec4Msg.X()
        self.y = vec4Msg.Y()
        self.z = vec4Msg.Z()
        self.w = vec4Msg.W()

    # Vec4MsgT
    def Pack(self, builder):
        Vec4MsgStart(builder)
        Vec4MsgAddX(builder, self.x)
        Vec4MsgAddY(builder, self.y)
        Vec4MsgAddZ(builder, self.z)
        Vec4MsgAddW(builder, self.w)
        vec4Msg = Vec4MsgEnd(builder)
        return vec4Msg


class SrvParamsWSMsg(object):
    __slots__ = ['_tab']

    @classmethod
    def GetRootAs(cls, buf, offset=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, offset)
        x = SrvParamsWSMsg()
        x.Init(buf, n + offset)
        return x

    @classmethod
    def GetRootAsSrvParamsWSMsg(cls, buf, offset=0):
        """This method is deprecated. Please switch to GetRootAs."""
        return cls.GetRootAs(buf, offset)
    @classmethod
    def SrvParamsWSMsgBufferHasIdentifier(cls, buf, offset, size_prefixed=False):
        return flatbuffers.util.BufferHasIdentifier(buf, offset, b"\x53\x30\x30\x32", size_prefixed=size_prefixed)

    # SrvParamsWSMsg
    def Init(self, buf, pos):
        self._tab = flatbuffers.table.Table(buf, pos)

    # SrvParamsWSMsg
    def Timestamp(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(4))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Float64Flags, o + self._tab.Pos)
        return 0.0

    # SrvParamsWSMsg
    def Name(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(6))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def Id(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(8))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def ReqSrcId(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(10))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def CmdReportOnce(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(12))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def CmdConnect(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(14))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def CmdDisconnect(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(16))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def IsConnected(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(18))
        if o != 0:
            return self._tab.Get(flatbuffers.number_types.Uint32Flags, o + self._tab.Pos)
        return 0

    # SrvParamsWSMsg
    def CurrentUrl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(20))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def ProdUrl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(22))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def ProdPort(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(24))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def DevUrl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(26))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def DevPort(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(28))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def SetProdUrl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(30))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def SetProdPort(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(32))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def SetDevUrl(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(34))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

    # SrvParamsWSMsg
    def SetDevPort(self):
        o = flatbuffers.number_types.UOffsetTFlags.py_type(self._tab.Offset(36))
        if o != 0:
            return self._tab.String(o + self._tab.Pos)
        return None

def SrvParamsWSMsgStart(builder):
    builder.StartObject(17)

def SrvParamsWSMsgAddTimestamp(builder, timestamp):
    builder.PrependFloat64Slot(0, timestamp, 0.0)

def SrvParamsWSMsgAddName(builder, name):
    builder.PrependUOffsetTRelativeSlot(1, flatbuffers.number_types.UOffsetTFlags.py_type(name), 0)

def SrvParamsWSMsgAddId(builder, id):
    builder.PrependUint32Slot(2, id, 0)

def SrvParamsWSMsgAddReqSrcId(builder, reqSrcId):
    builder.PrependUint32Slot(3, reqSrcId, 0)

def SrvParamsWSMsgAddCmdReportOnce(builder, cmdReportOnce):
    builder.PrependUint32Slot(4, cmdReportOnce, 0)

def SrvParamsWSMsgAddCmdConnect(builder, cmdConnect):
    builder.PrependUint32Slot(5, cmdConnect, 0)

def SrvParamsWSMsgAddCmdDisconnect(builder, cmdDisconnect):
    builder.PrependUint32Slot(6, cmdDisconnect, 0)

def SrvParamsWSMsgAddIsConnected(builder, isConnected):
    builder.PrependUint32Slot(7, isConnected, 0)

def SrvParamsWSMsgAddCurrentUrl(builder, currentUrl):
    builder.PrependUOffsetTRelativeSlot(8, flatbuffers.number_types.UOffsetTFlags.py_type(currentUrl), 0)

def SrvParamsWSMsgAddProdUrl(builder, prodUrl):
    builder.PrependUOffsetTRelativeSlot(9, flatbuffers.number_types.UOffsetTFlags.py_type(prodUrl), 0)

def SrvParamsWSMsgAddProdPort(builder, prodPort):
    builder.PrependUOffsetTRelativeSlot(10, flatbuffers.number_types.UOffsetTFlags.py_type(prodPort), 0)

def SrvParamsWSMsgAddDevUrl(builder, devUrl):
    builder.PrependUOffsetTRelativeSlot(11, flatbuffers.number_types.UOffsetTFlags.py_type(devUrl), 0)

def SrvParamsWSMsgAddDevPort(builder, devPort):
    builder.PrependUOffsetTRelativeSlot(12, flatbuffers.number_types.UOffsetTFlags.py_type(devPort), 0)

def SrvParamsWSMsgAddSetProdUrl(builder, setProdUrl):
    builder.PrependUOffsetTRelativeSlot(13, flatbuffers.number_types.UOffsetTFlags.py_type(setProdUrl), 0)

def SrvParamsWSMsgAddSetProdPort(builder, setProdPort):
    builder.PrependUOffsetTRelativeSlot(14, flatbuffers.number_types.UOffsetTFlags.py_type(setProdPort), 0)

def SrvParamsWSMsgAddSetDevUrl(builder, setDevUrl):
    builder.PrependUOffsetTRelativeSlot(15, flatbuffers.number_types.UOffsetTFlags.py_type(setDevUrl), 0)

def SrvParamsWSMsgAddSetDevPort(builder, setDevPort):
    builder.PrependUOffsetTRelativeSlot(16, flatbuffers.number_types.UOffsetTFlags.py_type(setDevPort), 0)

def SrvParamsWSMsgEnd(builder):
    return builder.EndObject()



class SrvParamsWSMsgT(object):

    # SrvParamsWSMsgT
    def __init__(self):
        self.timestamp = 0.0  # type: float
        self.name = None  # type: str
        self.id = 0  # type: int
        self.reqSrcId = 0  # type: int
        self.cmdReportOnce = 0  # type: int
        self.cmdConnect = 0  # type: int
        self.cmdDisconnect = 0  # type: int
        self.isConnected = 0  # type: int
        self.currentUrl = None  # type: str
        self.prodUrl = None  # type: str
        self.prodPort = None  # type: str
        self.devUrl = None  # type: str
        self.devPort = None  # type: str
        self.setProdUrl = None  # type: str
        self.setProdPort = None  # type: str
        self.setDevUrl = None  # type: str
        self.setDevPort = None  # type: str

    @classmethod
    def InitFromBuf(cls, buf, pos):
        srvParamsWsmsg = SrvParamsWSMsg()
        srvParamsWsmsg.Init(buf, pos)
        return cls.InitFromObj(srvParamsWsmsg)

    @classmethod
    def InitFromPackedBuf(cls, buf, pos=0):
        n = flatbuffers.encode.Get(flatbuffers.packer.uoffset, buf, pos)
        return cls.InitFromBuf(buf, pos+n)

    @classmethod
    def InitFromObj(cls, srvParamsWsmsg):
        x = SrvParamsWSMsgT()
        x._UnPack(srvParamsWsmsg)
        return x

    # SrvParamsWSMsgT
    def _UnPack(self, srvParamsWsmsg):
        if srvParamsWsmsg is None:
            return
        self.timestamp = srvParamsWsmsg.Timestamp()
        self.name = srvParamsWsmsg.Name()
        self.id = srvParamsWsmsg.Id()
        self.reqSrcId = srvParamsWsmsg.ReqSrcId()
        self.cmdReportOnce = srvParamsWsmsg.CmdReportOnce()
        self.cmdConnect = srvParamsWsmsg.CmdConnect()
        self.cmdDisconnect = srvParamsWsmsg.CmdDisconnect()
        self.isConnected = srvParamsWsmsg.IsConnected()
        self.currentUrl = srvParamsWsmsg.CurrentUrl()
        self.prodUrl = srvParamsWsmsg.ProdUrl()
        self.prodPort = srvParamsWsmsg.ProdPort()
        self.devUrl = srvParamsWsmsg.DevUrl()
        self.devPort = srvParamsWsmsg.DevPort()
        self.setProdUrl = srvParamsWsmsg.SetProdUrl()
        self.setProdPort = srvParamsWsmsg.SetProdPort()
        self.setDevUrl = srvParamsWsmsg.SetDevUrl()
        self.setDevPort = srvParamsWsmsg.SetDevPort()

    # SrvParamsWSMsgT
    def Pack(self, builder):
        if self.name is not None:
            name = builder.CreateString(self.name)
        if self.currentUrl is not None:
            currentUrl = builder.CreateString(self.currentUrl)
        if self.prodUrl is not None:
            prodUrl = builder.CreateString(self.prodUrl)
        if self.prodPort is not None:
            prodPort = builder.CreateString(self.prodPort)
        if self.devUrl is not None:
            devUrl = builder.CreateString(self.devUrl)
        if self.devPort is not None:
            devPort = builder.CreateString(self.devPort)
        if self.setProdUrl is not None:
            setProdUrl = builder.CreateString(self.setProdUrl)
        if self.setProdPort is not None:
            setProdPort = builder.CreateString(self.setProdPort)
        if self.setDevUrl is not None:
            setDevUrl = builder.CreateString(self.setDevUrl)
        if self.setDevPort is not None:
            setDevPort = builder.CreateString(self.setDevPort)
        SrvParamsWSMsgStart(builder)
        SrvParamsWSMsgAddTimestamp(builder, self.timestamp)
        if self.name is not None:
            SrvParamsWSMsgAddName(builder, name)
        SrvParamsWSMsgAddId(builder, self.id)
        SrvParamsWSMsgAddReqSrcId(builder, self.reqSrcId)
        SrvParamsWSMsgAddCmdReportOnce(builder, self.cmdReportOnce)
        SrvParamsWSMsgAddCmdConnect(builder, self.cmdConnect)
        SrvParamsWSMsgAddCmdDisconnect(builder, self.cmdDisconnect)
        SrvParamsWSMsgAddIsConnected(builder, self.isConnected)
        if self.currentUrl is not None:
            SrvParamsWSMsgAddCurrentUrl(builder, currentUrl)
        if self.prodUrl is not None:
            SrvParamsWSMsgAddProdUrl(builder, prodUrl)
        if self.prodPort is not None:
            SrvParamsWSMsgAddProdPort(builder, prodPort)
        if self.devUrl is not None:
            SrvParamsWSMsgAddDevUrl(builder, devUrl)
        if self.devPort is not None:
            SrvParamsWSMsgAddDevPort(builder, devPort)
        if self.setProdUrl is not None:
            SrvParamsWSMsgAddSetProdUrl(builder, setProdUrl)
        if self.setProdPort is not None:
            SrvParamsWSMsgAddSetProdPort(builder, setProdPort)
        if self.setDevUrl is not None:
            SrvParamsWSMsgAddSetDevUrl(builder, setDevUrl)
        if self.setDevPort is not None:
            SrvParamsWSMsgAddSetDevPort(builder, setDevPort)
        srvParamsWsmsg = SrvParamsWSMsgEnd(builder)
        return srvParamsWsmsg

