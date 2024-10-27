from cereal import car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_meas_steer_torque_limits,apply_driver_steer_torque_limits
from openpilot.selfdrive.car.interfaces import CarControllerBase
from openpilot.selfdrive.car.byd import bydcan
from openpilot.selfdrive.car.byd.values import CarControllerParams
from openpilot.selfdrive.controls.lib.drive_helpers import BYD_V_CRUISE_MIN

# HUD控制的可视警告类型
VisualAlert = car.CarControl.HUDControl.VisualAlert
# 按钮事件类型
ButtonType = car.CarState.ButtonEvent.Type


class CarController(CarControllerBase):
  def __init__(self, dbc_name, CP, VM):
    super().__init__(dbc_name, CP, VM)
    #不用

    #常用
    self.ACC_CMD_Counter_last = 0  # 上一帧的ACC_CMD_Counter
    self.params = CarControllerParams(self.CP)  # 获取控制参数
    self.apply_steer_last = 0  # 上一帧的转向值
    self.packer = CANPacker(dbc_name)  # CAN打包器

    self.lkas_counter_tx = 0 #lkas 发送计数器
    self.last_steer_frame = 0  # 上次转向帧

    self.lkas_Prepare_num = 0 #lkas准备计数
    self.lkas_Prepare = 0
    self.lkas_enabled = 0


  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    can_sends = []  # 初始化发送的CAN消息列表

    # 转向控制（激活时 50Hz，非激活时 20Hz）
    steer_step = self.params.STEER_STEP if CC.latActive else self.params.INACTIVE_STEER_STEP
    if (self.frame - self.last_steer_frame) >= steer_step :

        apply_steer = 0  # 初始化转向输出
        if CC.latActive:
            # 计算转向并根据驾驶员施加的扭矩设置限制
            new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))

            # apply_torque: 当前计算的转向扭矩。
            # apply_torque_last: 上一个应用的转向扭矩。
            # driver_torque: 驾驶员施加的转向扭矩。
            # LIMITS: 包含转向相关限制的参数对象。
            # apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
            #                                                 CS.out.steeringTorque, CarControllerParams)
            # *** steer torque ***
            apply_steer = apply_meas_steer_torque_limits(new_steer, self.apply_steer_last,
                                                            CS.out.steeringTorqueEps, self.params)

        self.apply_steer_last = apply_steer  # 保存上次应用的转向

        if CC.latActive and self.lkas_Prepare_num <10:
            apply_steer = 0  # 转向输出0
            self.lkas_Prepare = 1
            self.lkas_enabled = 0
            self.lkas_Prepare_num +=1
        elif CC.latActive and self.lkas_Prepare_num ==10:
            self.lkas_Prepare = 0
            self.lkas_enabled = 1
        else:
            self.lkas_Prepare = 0
            self.lkas_enabled = 0
            self.lkas_Prepare_num =0
            apply_steer = 0  # 转向输出0

        can_sends.append(bydcan.create_steering_control(self.packer, self.CP,CS.cam_lkas, apply_steer,
                                                        self.lkas_enabled,self.lkas_Prepare,self.lkas_counter_tx ))
        #更新计数器
        self.lkas_counter_tx = (self.lkas_counter_tx + 1) % 16  # lkas 发送计数器 每次循环将计数器递增，超过15后返回到0
        self.last_steer_frame = self.frame  # 更新上次转向帧

    # 发送ACC控制命令
    if CS.ACC_CMD_Counter != self.ACC_CMD_Counter_last:
        accel = clip(actuators.accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX)
        can_sends.append(bydcan.acc_command(self.packer, self.CP,CS.cam_acc, accel, CC.enabled ))
        self.ACC_CMD_Counter_last = CS.ACC_CMD_Counter


    new_actuators = CC.actuators.as_builder()  # 创建新的执行器状态
    new_actuators.steer = self.apply_steer_last / CarControllerParams.STEER_MAX  # 归一化转向值
    new_actuators.steerOutputCan = self.apply_steer_last  # 设置输出的转向值


    self.frame += 1  # 增加帧计数
    return new_actuators, can_sends  # 返回新的执行器状态和要发送的CAN消息
