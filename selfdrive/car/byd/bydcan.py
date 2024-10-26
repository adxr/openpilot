from openpilot.selfdrive.car.byd.values import  BydFlags  # 导入按钮和byd标志


def byd_checksum(byte_key, dat):
  second_bytes = [byte & 0xf for byte in dat]
  remainder = sum(second_bytes) >> 4
  second_bytes.append(byte_key >> 4)

  first_bytes = [byte >> 4 for byte in dat]
  first_bytes.append(byte_key & 0xf)

  return (((((-1 * sum(first_bytes) + 0x9) & 0xf) + (-1*remainder + 5)) << 4) + ((-1 * sum(second_bytes) + 0x9) & 0xf)) & 0xff


def create_steering_control(packer, CP, cam_msg: dict, angle, enabled,LKAS_Prepare,Counter):
  # 创建转向控制消息
  angle = max(min(angle, 400), -400)

  values = {}
  values = {s: cam_msg[s] for s in [
      "AutoFullBeam_OnOff",#自动远光灯
      "AutoFullBeamState",#自动远光灯状态
      "LKAS_Config",#LKAS Config
      "SETME_0x1",
      "ReqHandsOnSteeringWheel",#要求将手放在方向盘上
      "TrafficSignRecognition_OnOff",#交通标志识别
      "SETME4_0x1",
      "SETME_0x0",
      "TrafficSignRecognition_Result",#交通标志识别结果
      "LKAS_AlarmType",
      "SETME_0x3",
  ]}
  values.update({
      "LKAS_Output" : angle,#LKAS控制
      "LKAS_Prepare" : LKAS_Prepare,#LKAS准备
      "LKAS_ACTIVE" : enabled,#LKAS# 转向控制类型（启用或禁用）
      "LKAS_OnOff" : 1,# 开/关
      "LeftLaneState" : enabled,#左车道状态
      "RightLaneState" : enabled,#右车道状态
      "Counter" : Counter,# 控制计数器
  })
    # 生成转向控制消息并计算校验和
  data = packer.make_can_msg("MPC_LKAS_CTRL_AND_STATUS",0, values)[1]
  values["CheckSum"] = byd_checksum(0xaf, data)  # 计算校验和
  return packer.make_can_msg("MPC_LKAS_CTRL_AND_STATUS", 0, values)  # 生成CAN消息


def acc_command(packer, CP, cam_msg: dict, speed,enabled):
  # 创建ACC_CMD
  speed = max(min(speed * 16.67, 30), -50)
  values = {}

  values = {s: cam_msg[s] for s in [
      "ComfortBandUpper",#最大加速度
      "ComfortBandLower",#最小加速度
      "JerkUpperLimit",#最大加速度变化率
      "SET_ME_3",
      "JerkLowerLimit",#最小加速度变化率
      "ResumeFromStandstill", #从静止状态恢复
      "StandstillState",#静止状态
      "BrakeBehaviour",#制动行为
      "AccOverrideOrStandstill",#加速覆盖或静止
      "EspBehaviour",#Esp 行为
      "Counter",#计数器
      "SET_ME_0xF",#设置 ME 0xF
      "AccelCmd",#加速命令
      "AccReqNotStandstill",
      "AccControlActive",
  ]}
  # values.update({
  #     "AccelCmd" : speed,#加速命令
  #     "AccReqNotStandstill" : 1 if enabled else 0,#加速请求非静止 1
  #     "AccControlActive" : 1 if enabled else 0,#加速控制激活 1

      # })
    # 生成转向控制消息并计算校验和
  data = packer.make_can_msg("ACC_CMD",0, values)[1]
  values["CheckSum"] = byd_checksum(0xaf, data)  # 计算校验和
  return packer.make_can_msg("ACC_CMD", 0, values)  # 生成CAN消息