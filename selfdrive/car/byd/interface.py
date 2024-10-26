#!/usr/bin/env python3
from cereal import car  # 导入汽车相关数据结构
from openpilot.common.conversions import Conversions as CV  # 导入单位转换工具
from openpilot.selfdrive.car.byd.values import CAR, LKAS_LIMITS  # 导入Byd相关的常量
from openpilot.selfdrive.car import create_button_events, get_safety_config  # 导入按钮事件和安全配置
from openpilot.selfdrive.car.interfaces import CarInterfaceBase  # 导入基类

ButtonType = car.CarState.ButtonEvent.Type  # 按钮类型
EventName = car.CarEvent.EventName  # 事件名称
GearShifter = car.CarState.GearShifter  # 换挡状态

class CarInterface(CarInterfaceBase):
  def __init__(self, CP, CarController, CarState):
    super().__init__(CP, CarController, CarState)  # 初始化基类

  @staticmethod
  def _get_params(ret, candidate, fingerprint, car_fw, experimental_long, docs):
    # 获取汽车参数
    ret.carName = "byd"  # 设置汽车名称
    ret.safetyConfigs = [get_safety_config(car.CarParams.SafetyModel.byd)]  # 设置安全配置
    ret.radarUnavailable = True  # 雷达不可用
    ret.customStockLongAvailable = True  # 自定义长效巡航可用

    # 判断是否为Dashcam-only模式
    #ret.dashcamOnly = candidate not in (CAR.BYD_HAN_EV_21)
    ret.dashcamOnly = False

    ret.steerActuatorDelay = 0.1  # 转向执行器延迟
    ret.steerLimitTimer = 0.8  # 转向限制计时器

    CarInterfaceBase.configure_torque_tune(candidate, ret.lateralTuning)  # 配置扭矩调整

    ret.minSteerSpeed = LKAS_LIMITS.DISABLE_SPEED * CV.KPH_TO_MS  # 设置最小转向速度

    ret.centerToFront = ret.wheelbase * 0.41  # 计算前轴中心到前轮轴的距离

    return ret  # 返回设置后的参数

  # 更新汽车状态
  def _update(self, c):
    ret = self.CS.update(self.cp, self.cp_cam)

    # TODO: add button types for inc and dec
    self.CS.button_events = [
      *self.CS.button_events,
      *create_button_events(self.CS.distance_button, self.CS.prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.CS.lkas_enabled, self.CS.prev_lkas_enabled, {1: ButtonType.altButton1}),
    ]

    self.CS.mads_enabled = self.get_sp_cruise_main_state(ret)

    self.CS.accEnabled = self.get_sp_v_cruise_non_pcm_state(ret, c.vCruise, self.CS.accEnabled)

    if ret.cruiseState.available:
      if self.enable_mads:
        if not self.CS.prev_mads_enabled and self.CS.mads_enabled:
          self.CS.madsEnabled = True
        if any(b.type == ButtonType.altButton1 and b.pressed for b in self.CS.button_events):
          self.CS.madsEnabled = not self.CS.madsEnabled
        self.CS.madsEnabled = self.get_acc_mads(ret, self.CS.madsEnabled)
    else:
      self.CS.madsEnabled = False

    if not self.CP.pcmCruise or (self.CP.pcmCruise and self.CP.minEnableSpeed > 0) or not self.CP.pcmCruiseSpeed:
      if any(b.type == ButtonType.cancel for b in self.CS.button_events):
        self.get_sp_cancel_cruise_state()
    if self.get_sp_pedal_disengage(ret):
      self.get_sp_cancel_cruise_state()
      ret.cruiseState.enabled = ret.cruiseState.enabled if not self.enable_mads else False if self.CP.pcmCruise else self.CS.accEnabled

    if self.CP.pcmCruise and self.CP.minEnableSpeed > 0 and self.CP.pcmCruiseSpeed:
      if ret.gasPressed and not ret.cruiseState.enabled:
        self.CS.accEnabled = False
      self.CS.accEnabled = ret.cruiseState.enabled or self.CS.accEnabled

    ret = self.get_sp_common_state(ret)

    ret.buttonEvents = [
      *self.CS.button_events,
      *self.button_events.create_mads_event(self.CS.madsEnabled, self.CS.out.madsEnabled)  # MADS BUTTON
    ]

    # events
    events = self.create_common_events(ret, c, extra_gears=[GearShifter.sport, GearShifter.low, GearShifter.brake],
                                       pcm_enable=False)

    events, ret = self.create_sp_events(ret, events)

    #if self.CS.lkas_disabled:
    #  events.add(EventName.lkasDisabled)
    if self.CS.low_speed_alert:
      events.add(EventName.belowSteerSpeed)

    ret.customStockLong = self.update_custom_stock_long()

    ret.events = events.to_msg()

    return ret
