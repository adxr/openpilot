import copy
from cereal import car  # 导入汽车相关数据结构
from openpilot.common.conversions import Conversions as CV  # 导入单位转换工具
from opendbc.can.can_define import CANDefine  # 导入CAN定义
from opendbc.can.parser import CANParser  # 导入CAN解析器
from openpilot.selfdrive.car.interfaces import CarStateBase  # 导入汽车状态基类
from openpilot.selfdrive.car.byd.values import DBC, LKAS_LIMITS,  BUTTONS  # 导入byd相关常量和按钮

class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)  # 初始化基类

    # 初始化CAN定义和相关变量
    can_define = CANDefine(DBC[CP.carFingerprint]["pt"])
    self.shifter_values = can_define.dv["DRIVE_STATE"]["Gear"]

    self.crz_btns_counter = 0  # 自适应巡航按钮计数器
    self.acc_active_last = False  # 上一次ACC激活状态
    self.low_speed_alert = False  # 低速警报状态
    self.lkas_allowed_speed = False  # LKAS允许的速度状态
    self.lkas_disabled = False  # LKAS禁用状态

    self.prev_distance_button = 0  # 上一次距离按钮状态
    self.distance_button = 0  # 当前距离按钮状态

    self.lkas_enabled = False  # LKAS激活状态
    self.prev_lkas_enabled = False  # 上一次LKAS状态

    self.cam_lkas = False
    self.cam_acc = False

    # 按钮状态初始化
    self.button_states = {button.event_type: False for button in BUTTONS}

  def update(self, cp, cp_cam):
    ret = car.CarState.new_message()  # 创建新的汽车状态消息

    # 更新按钮状态
    self.prev_distance_button = self.distance_button
    self.distance_button = cp.vl["PCM_BUTTONS"]["BTN_AccDistanceIncrease"]#Acc距离

    self.prev_mads_enabled = self.mads_enabled
    self.prev_lkas_enabled = self.lkas_enabled

    # 获取轮速并计算车速
    ret.wheelSpeeds = self.get_wheel_speeds(
      cp.vl["IPB1"]["WheelSpeed_FL"],
      cp.vl["IPB1"]["WheelSpeed_FR"],
      cp.vl["IPB1"]["WheelSpeed_RL"],
      cp.vl["IPB1"]["WheelSpeed_RR"],
    )
    speed_kph = (ret.wheelSpeeds.fl + ret.wheelSpeeds.fr + ret.wheelSpeeds.rl + ret.wheelSpeeds.rr) / 4.
    ret.vEgoRaw = speed_kph
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # # 匹配速度读取
    # #speed_kph = cp.vl["ENGINE_DATA"]["SPEED"]
    ret.standstill = speed_kph <= .1  # 判断是否静止

    self.lkas_enabled = not self.lkas_disabled  # 更新LKAS激活状态

    # 解析换挡状态
    can_gear = int(cp.vl["DRIVE_STATE"]["Gear"])
    ret.gearShifter = self.parse_gear_shifter(self.shifter_values.get(can_gear, None))

    #更新按钮状态
    for button in BUTTONS:
      state = (cp.vl[button.can_addr][button.can_msg] in button.values)
      if self.button_states[button.event_type] != state:
        event = car.CarState.ButtonEvent.new_message()
        event.type = button.event_type
        event.pressed = state
        self.button_events.append(event)  # 添加按钮事件
      self.button_states[button.event_type] = state  # 更新按钮状态

    # 更新其他状态
    #远光灯
    ret.genericToggle = bool(cp.vl["STALKS"]["HeadLight"])
    #左、右盲区监测状态
    ret.leftBlindspot = cp.vl["BSD_RADAR"]["LEFT_ROACH"] != 0
    ret.rightBlindspot = cp.vl["BSD_RADAR"]["RIGHT_ROACH"] != 0
    # 转向灯状态
    ret.leftBlinker, ret.rightBlinker = ret.leftBlinkerOn, ret.rightBlinkerOn = self.update_blinker_from_lamp(40, cp.vl["STALKS"]["LeftIndicator"] == 1,
                                                                                                              cp.vl["STALKS"]["RightIndicator"] == 1)

    # 获取转向角和转向力
    ret.steeringAngleDeg = cp.vl["EPS"]["SteeringAngle"]  # 方向盘角度
    ret.steeringTorque = cp.vl["STEERING_TORQUE"]["SteerDriverTorque"]  # 人给车的方向盘转矩
    # ret.steeringPressed = (cp.vl["STEERING_TORQUE"]["ReportHandsNotOnSteeringWheel"] < 1)  # 如果手握方向盘的级别小于 1，则认为方向盘被握住

    ret.steeringTorqueEps = cp.vl["STEERING_TORQUE"]["MainTorque"] #获取电动助力转向的扭矩。
    ret.steeringRateDeg = cp.vl["EPS"]["EpsTorque"]  # 方向盘角速度

    ret.steeringPressed = True


    # 获取刹车和油门状态
    ret.brake =  cp.vl["PEDAL"]["BrakePedal"]  # 刹车位置百分比
    #判断刹车是否被踩下
    ret.brakePressed = (ret.brake  > 0)  # 如果刹车位置大于 0  则刹车是否被踩下
    #检查安全带是否未系
    ret.seatbeltUnlatched = (cp.vl["BCM"]["DriverSeatBeltFasten"] != 1)
    #检查任一车门是否打开
    ret.doorOpen = any([cp.vl["BCM"]["DOOR_STATE_FL"], cp.vl["BCM"]["DOOR_STATE_FR"],
                        cp.vl["BCM"]["DOOR_STATE_RL"], cp.vl["BCM"]["DOOR_STATE_RR"]])
    #获取油门踏板的状态。
    ret.gas = cp.vl["PEDAL"]["AcceleratorPedal"]  # 油门位置百分比
    ret.gasPressed = ret.gas > 0  # 判断油门是否踩下

    # 检查LKAS是否被禁用
    # lkas_blocked = 0

    # if self.CP.minSteerSpeed > 0:
    #     if speed_kph > LKAS_LIMITS.ENABLE_SPEED and not lkas_blocked:
    #         self.lkas_allowed_speed = True  # LKAS允许的速度
    #     elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
    #         self.lkas_allowed_speed = False  # LKAS不允许的速度
    # else:
    #     self.lkas_allowed_speed = True

    if self.CP.minSteerSpeed > 0:
      if speed_kph > LKAS_LIMITS.ENABLE_SPEED:
        self.lkas_allowed_speed = True  # LKAS允许的速度
      elif speed_kph < LKAS_LIMITS.DISABLE_SPEED:
        self.lkas_allowed_speed = False  # LKAS不允许的速度
    else:
      self.lkas_allowed_speed = True

    # 更新巡航状态
    #检查巡航控制是否可用
    ret.cruiseState.available = cp_cam.vl["ACC_HUD_ADAS"]["AccOn1"] == 1
    #检查巡航控制是否激活
    ret.cruiseState.enabled = cp_cam.vl["ACC_HUD_ADAS"]["AccOn2"] == 1
    #判断车辆是否静止
    ret.cruiseState.standstill = False
    #获取当前巡航速度
    ret.cruiseState.speed = cp_cam.vl["ACC_HUD_ADAS"]["SetSpeed"] * CV.KPH_TO_MS

    if ret.cruiseState.enabled:
      if not self.lkas_allowed_speed and self.acc_active_last:
        self.low_speed_alert = True  # 低速警报
      else:
        self.low_speed_alert = False

    # 检查LKAS临时故障
    ret.steerFaultTemporary = False if self.lkas_allowed_speed else True
    # ret.steerFaultTemporary = False

    self.acc_active_last = ret.cruiseState.enabled  # 更新ACC激活状态

    self.crz_btns_counter = cp_cam.vl["ACC_HUD_ADAS"]["Counter"]  # 更新自适应巡航按钮计数器

    self.LKAS_Counter = cp_cam.vl["MPC_LKAS_CTRL_AND_STATUS"]["Counter"]  # 上LKAS_Counter
    self.ACC_CMD_Counter = cp_cam.vl["ACC_CMD"]["Counter"]  # ACC_CMD_Counter

    self.cam_adas = copy.copy(cp_cam.vl["ACC_HUD_ADAS"])
    # 处理摄像头信号
    self.lkas_disabled =  0 if cp_cam.vl["MPC_LKAS_CTRL_AND_STATUS"]["LKAS_Config"] > 0 else 1

    self.cam_lkas = copy.copy(cp_cam.vl["MPC_LKAS_CTRL_AND_STATUS"])
    self.cam_acc = copy.copy(cp_cam.vl["ACC_CMD"])

    # 永久转向故障
    # ret.steerFaultPermanent = cp.vl["STEERING_TORQUE"]["SteerError_1"]  # 永久转向故障
    # AEB（自动紧急制动）事件
    # ret.stockAeb = (cp_cam.vl["ACC_HUD_ADAS"]["AEB"] == 1)

    return ret  # 返回更新后的状态

  @staticmethod
  def get_can_parser(CP):
    messages = [
        # 信号地址, 频率
        ("EPS", 80),
        ("IPB1", 40),
        ("PEDAL", 20),
        ("ESP", 20),
        ("STEERING_TORQUE", 20),
        ("DRIVE_STATE", 20),
        ("STALKS", 10),
        ("BCM", 10),
        ("PCM_BUTTONS", 10),
        ("BSD_RADAR", 20),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 0)  # 返回CAN解析器

  @staticmethod
  def get_cam_can_parser(CP):
    messages = [
        # 信号地址, 频率
        ("ACC_CMD", 180),
        ("MPC_LKAS_CTRL_AND_STATUS", 180),
        ("ACC_HUD_ADAS", 180),
    ]

    return CANParser(DBC[CP.carFingerprint]["pt"], messages, 2)  # 返回摄像头CAN解析器
