#! /usr/bin/env python

from jodellSdk.jodellSdkDemo import ClawEpgTool

# 创建夹爪对象
clawTool =  ClawEpgTool()
#查询可用串口
comlist = clawTool.searchCom()
#通讯连接
flag_connect = clawTool.serialOperation('/dev/ttyUSB0',115200,True)
#使能
flag_enable = clawTool.clawEnable(9,True)
# 夹爪关闭并返回标志
flag_close = clawTool.runWithoutParam(9,1)
    