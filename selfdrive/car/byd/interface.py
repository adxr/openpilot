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
    
    ret.events = events.to_msg()
    return ret
