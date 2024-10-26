from collections import namedtuple  # 导入命名元组
from dataclasses import dataclass, field  # 导入数据类和字段
from enum import IntFlag  # 导入整型标志枚举

from cereal import car  # 导入car模块
from openpilot.common.conversions import Conversions as CV  # 导入单位转换模块
from openpilot.selfdrive.car import CarSpecs, DbcDict, PlatformConfig, Platforms, dbc_dict  # 导入车辆规格、DBC字典和平台配置
from openpilot.selfdrive.car.docs_definitions import CarHarness, CarDocs, CarParts  # 导入车辆文档定义
from openpilot.selfdrive.car.fw_query_definitions import FwQueryConfig, Request, StdQueries  # 导入固件查询定义

Ecu = car.CarParams.Ecu
Button = namedtuple('Button', ['event_type', 'can_addr', 'can_msg', 'values'])  # 定义按钮事件的命名元组


# 转向扭矩限制参数

class CarControllerParams:
  STEER_MAX = 300  # 理论最大转向扭矩
  STEER_DELTA_UP = 14  # 每次刷新增加的扭矩
  STEER_DELTA_DOWN = 14  # 每次刷新减少的扭矩
  STEER_DRIVER_ALLOWANCE = 65  # 驾驶员允许的扭矩
  STEER_DRIVER_MULTIPLIER = 4  # 驾驶员扭矩权重
  STEER_DRIVER_FACTOR = 1  # 来自DBC的因子
  STEER_ERROR_MAX = 350  # 扭矩命令和电机扭矩之间的最大差异
  STEER_STEP = 1  # 100 Hz

  ACCEL_MAX = 20               # m/s^2 最大加速度
  ACCEL_MIN = -30              # m/s^2 最小加速度
  def __init__(self, CP):
    pass


@dataclass
class MazdaCarDocs(CarDocs):
  package: str = "All"
  car_parts: CarParts = field(default_factory=CarParts.common([CarHarness.byd]))  # 车辆部件初始化


@dataclass(frozen=True, kw_only=True)
class MazdaCarSpecs(CarSpecs):
  tireStiffnessFactor: float = 0.7  # 轮胎刚度因子


class BydFlags(IntFlag):
  # 静态标志
  # HAN硬件：相同的CAN消息和相同的相机
  GEN1 = 1  # HAN


@dataclass
class BydPlatformConfig(PlatformConfig):
  dbc_dict: DbcDict = field(default_factory=lambda: dbc_dict('byd_han_21', None))  # 初始化DBC字典
  flags: int = BydFlags.GEN1  # 设置标志


class CAR(Platforms):
  # 定义不同车型的配置
  BYD_HAN_EV_21 = BydPlatformConfig(
      [BydCarDocs("Byd Han EV20-21")],
      BydCarSpecs(mass=4217 * CV.LB_TO_KG, wheelbase=3.1, steerRatio=17.6)
  )


class LKAS_LIMITS:
  STEER_THRESHOLD = 15  # 转向阈值
  DISABLE_SPEED = 5  # 禁用速度（公里/小时）
  ENABLE_SPEED = 20  # 启用速度（公里/小时）


class Buttons:
  NONE = 0  # 无事件
  SET_PLUS = 1  # 增加设置
  SET_MINUS = 2  # 减少设置
  RESUME = 3  # 恢复
  CANCEL = 4  # 取消


BUTTONS = [
  Button(car.CarState.ButtonEvent.Type.leftBlinker, "STALKS", "LeftIndicator", [0x01]),  # 左转向灯
  Button(car.CarState.ButtonEvent.Type.rightBlinker, "STALKS", "RightIndicator", [0x01]),  # 右转向灯
  Button(car.CarState.ButtonEvent.Type.accelCruise, "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x03]),  # 加速巡航
  Button(car.CarState.ButtonEvent.Type.decelCruise, "PCM_BUTTONS", "BTN_AccUpDown_Cmd", [0x01]),  # 减速巡航
  Button(car.CarState.ButtonEvent.Type.cancel, "PCM_BUTTONS", "BTN_AccCancel", [0x01]),  # 取消巡航
  Button(car.CarState.ButtonEvent.Type.resumeCruise, "PCM_BUTTONS", "BTN_TOGGLE_ACC_OnOff", [0x01]),  # 恢复巡航
]


FW_QUERY_CONFIG = FwQueryConfig(
  requests=[
    # TODO: check data to ensure ABS does not skip ISO-TP frames on bus 0
    Request(
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_REQUEST],
      [StdQueries.MANUFACTURER_SOFTWARE_VERSION_RESPONSE],
      bus=0,
    ),
  ],
)

DBC = CAR.create_dbc_map()
