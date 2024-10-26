from cereal import car
import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.params import Params
from openpilot.common.numpy_fast import clip
from openpilot.common.realtime import DT_CTRL
from opendbc.can.packer import CANPacker
from openpilot.selfdrive.car import apply_driver_steer_torque_limits
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
    self.LKAS_Counter_last = 0  # 上一帧的转向值LKAS_Counter
    self.ACC_CMD_Counter_last = 0  # 上一帧的ACC_CMD_Counter

    self.apply_steer_last = 0  # 上一帧的转向值
    self.packer = CANPacker(dbc_name)  # CAN打包器
    self.brake_counter = 0  # 刹车计数器

    self.sm = messaging.SubMaster(['longitudinalPlanSP'])  # 订阅纵向计划状态
    self.param_s = Params()  # 参数存储
    self.last_speed_limit_sign_tap_prev = False  # 上一次速度限制标志
    self.speed_limit = 0.  # 当前速度限制
    self.speed_limit_offset = 0  # 速度限制偏移量
    self.timer = 0  # 计时器
    self.final_speed_kph = 0  # 最终速度（公里每小时）
    self.init_speed = 0  # 初始速度
    self.current_speed = 0  # 当前速度
    self.v_set_dis = 0  # 设置的速度
    self.v_cruise_min = 0  # 最小巡航速度
    self.button_type = 0  # 按钮类型
    self.button_select = 0  # 按钮选择
    self.button_count = 0  # 按钮计数
    self.target_speed = 0  # 目标速度
    self.t_interval = 7  # 时间间隔
    self.slc_active_stock = False  # 速度限制控制状态
    self.sl_force_active_timer = 0  # 强制激活计时器
    self.v_tsc_state = 0  # 视觉转向控制状态
    self.slc_state = 0  # 速度限制控制状态
    self.m_tsc_state = 0  # 地图转向控制状态
    self.cruise_button = None  # 巡航按钮
    self.speed_diff = 0  # 速度差
    self.v_tsc = 0  # 视觉转向速度
    self.m_tsc = 0  # 地图转向速度
    self.steady_speed = 0  # 稳定速度

    self.lkas_Prepare_num = 0 #lkas准备数
    self.lkas_Prepare = 0
    self.lkas_enabled = 0

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    # 如果车辆没有设置巡航速度
    if not self.CP.pcmCruiseSpeed:
        self.sm.update(0)  # 更新状态机，0代表没有数据更新的时间戳

        # 如果 longitudinalPlanSP 数据已更新
        if self.sm.updated['longitudinalPlanSP']:
            # 从状态机获取各类控制状态和速度限制
            self.v_tsc_state = self.sm['longitudinalPlanSP'].visionTurnControllerState
            self.slc_state = self.sm['longitudinalPlanSP'].speedLimitControlState
            self.m_tsc_state = self.sm['longitudinalPlanSP'].turnSpeedControlState
            self.speed_limit = self.sm['longitudinalPlanSP'].speedLimit
            self.speed_limit_offset = self.sm['longitudinalPlanSP'].speedLimitOffset
            self.v_tsc = self.sm['longitudinalPlanSP'].visionTurnSpeed
            self.m_tsc = self.sm['longitudinalPlanSP'].turnSpeed

        # 设置最小巡航速度
        self.v_cruise_min = BYD_V_CRUISE_MIN[CS.params_list.is_metric] * (CV.KPH_TO_MPH if not CS.params_list.is_metric else 1)

    can_sends = []  # 初始化发送的CAN消息列表

    if not self.CP.pcmCruiseSpeed:
        # 检查上次速度限制标志是否被触发
        if not self.last_speed_limit_sign_tap_prev and CS.params_list.last_speed_limit_sign_tap:
            self.sl_force_active_timer = self.frame
            self.param_s.put_bool_nonblocking("LastSpeedLimitSignTap", False)
        self.last_speed_limit_sign_tap_prev = CS.params_list.last_speed_limit_sign_tap

        # 判断速度限制是否有效
        sl_force_active = CS.params_list.speed_limit_control_enabled and (self.frame < (self.sl_force_active_timer * DT_CTRL + 2.0))
        sl_inactive = not sl_force_active and (not CS.params_list.speed_limit_control_enabled or (True if self.slc_state == 0 else False))
        sl_temp_inactive = not sl_force_active and (CS.params_list.speed_limit_control_enabled and (True if self.slc_state == 1 else False))
        slc_active = not sl_inactive and not sl_temp_inactive

        self.slc_active_stock = slc_active  # 保存当前速度限制控制状态



    # 临时禁用转向控制，当驾驶员握方向盘并触发故障时
    # hands_on_fault = CS.hands_on_level >= 3
    # lkas_enabled = CC.latActive and not hands_on_fault  # 确定是否启用车道保持辅助系统
    # 发送转向控制命令
    if CS.LKAS_Counter != self.LKAS_Counter_last:
        apply_steer = 0  # 初始化转向输出
        if CC.latActive:
            # 计算转向并根据驾驶员施加的扭矩设置限制
            new_steer = int(round(CC.actuators.steer * CarControllerParams.STEER_MAX))

            # apply_torque: 当前计算的转向扭矩。
            # apply_torque_last: 上一个应用的转向扭矩。
            # driver_torque: 驾驶员施加的转向扭矩。
            # LIMITS: 包含转向相关限制的参数对象。
            apply_steer = apply_driver_steer_torque_limits(new_steer, self.apply_steer_last,
                                                            CS.out.steeringTorque, CarControllerParams)
        self.apply_steer_last = apply_steer  # 保存上次应用的转向

        if CC.latActive and self.lkas_Prepare_num <10:
            apply_steer = 0  # 初始化转向输出
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
            apply_steer = 0  # 初始化转向输出


        can_sends.append(bydcan.create_steering_control(self.packer, self.CP,CS.cam_lkas, apply_steer, self.lkas_enabled,self.lkas_Prepare ))
        self.LKAS_Counter_last=CS.LKAS_Counter


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
