#!/usr/bin/env python3
# encoding: utf-8
"""
Конвертеры между форматами данных для модуля стабилизации.
Преобразует данные между внутренними форматами gait_manager и интерфейсами.
"""

from typing import Dict, Any, List
from walking.interfaces import (
    WalkingParams,
    WalkingPeriodParams,
    RobotPoseParams,
    GaitBaseParams,
    StabilizationResult
)


def gait_param_to_walking_params(gait_param: Dict[str, Any], period_time: List, 
                                  move_amplitudes: Dict[str, float]) -> WalkingParams:
    """
    Конвертирует gait_param и period_time в WalkingParams.
    
    Args:
        gait_param: Словарь параметров походки из gait_manager
        period_time: Список [period_ms, dsp_ratio, y_swap_amplitude]
        move_amplitudes: Словарь {'x': float, 'y': float, 'angle': float}
    
    Returns:
        WalkingParams: Структурированные параметры ходьбы
    """
    period = WalkingPeriodParams(
        period_ms=int(period_time[0]),
        dsp_ratio=float(period_time[1]),
        y_swap_amplitude=float(period_time[2])
    )
    
    pose = RobotPoseParams(
        init_x_offset=float(gait_param.get('init_x_offset', 0.0)),
        init_y_offset=float(gait_param.get('init_y_offset', 0.0)),
        init_roll_offset=float(gait_param.get('init_roll_offset', 0.0)),
        init_pitch_offset=float(gait_param.get('init_pitch_offset', 0.0))
    )
    
    gait_base = GaitBaseParams(
        body_height=float(gait_param.get('body_height', 0.025)),
        step_fb_ratio=float(gait_param.get('step_fb_ratio', 0.028)),
        z_swap_amplitude=float(gait_param.get('z_swap_amplitude', 0.006)),
        hip_pitch_offset=float(gait_param.get('hip_pitch_offset', 15.0)),
        pelvis_offset=float(gait_param.get('pelvis_offset', 5.0))
    )
    
    extra_params = {k: v for k, v in gait_param.items() 
                    if k not in ['init_x_offset', 'init_y_offset', 'init_roll_offset', 
                                'init_pitch_offset', 'body_height', 'step_fb_ratio',
                                'z_swap_amplitude', 'hip_pitch_offset', 'pelvis_offset']}
    
    return WalkingParams(
        period=period,
        pose=pose,
        gait_base=gait_base,
        move_amplitudes=move_amplitudes.copy(),
        extra_params=extra_params
    )


def walking_params_to_gait_param(walking_params: WalkingParams) -> Dict[str, Any]:
    """
    Конвертирует WalkingParams обратно в gait_param.
    
    Args:
        walking_params: Структурированные параметры ходьбы
    
    Returns:
        Dict[str, Any]: Словарь параметров походки для gait_manager
    """
    gait_param = {}
    
    gait_param['init_x_offset'] = walking_params.pose.init_x_offset
    gait_param['init_y_offset'] = walking_params.pose.init_y_offset
    gait_param['init_roll_offset'] = walking_params.pose.init_roll_offset
    gait_param['init_pitch_offset'] = walking_params.pose.init_pitch_offset
    
    gait_param['body_height'] = walking_params.gait_base.body_height
    gait_param['step_fb_ratio'] = walking_params.gait_base.step_fb_ratio
    gait_param['z_swap_amplitude'] = walking_params.gait_base.z_swap_amplitude
    gait_param['hip_pitch_offset'] = walking_params.gait_base.hip_pitch_offset
    gait_param['pelvis_offset'] = walking_params.gait_base.pelvis_offset
    
    gait_param.update(walking_params.extra_params)
    
    return gait_param


def walking_params_to_period_time(walking_params: WalkingParams) -> List:
    """
    Конвертирует WalkingParams в period_time список.
    
    Args:
        walking_params: Структурированные параметры ходьбы
    
    Returns:
        List: [period_ms, dsp_ratio, y_swap_amplitude]
    """
    return [
        walking_params.period.period_ms,
        walking_params.period.dsp_ratio,
        walking_params.period.y_swap_amplitude
    ]


def apply_stabilization_result(
    base_params: WalkingParams,
    stabilization_result: StabilizationResult
) -> WalkingParams:
    """
    Применяет результат стабилизации к базовым параметрам.
    Если override не указан - используется значение из base_params.
    
    Args:
        base_params: Базовые параметры из конфига
        stabilization_result: Результат стабилизации
    
    Returns:
        WalkingParams: Параметры с примененными переопределениями
    """
    if not stabilization_result.modified:
        return base_params
    
    period = stabilization_result.period_override if stabilization_result.period_override else base_params.period
    pose = stabilization_result.pose_override if stabilization_result.pose_override else base_params.pose
    gait_base = stabilization_result.gait_base_override if stabilization_result.gait_base_override else base_params.gait_base
    
    return WalkingParams(
        period=period,
        pose=pose,
        gait_base=gait_base,
        move_amplitudes=base_params.move_amplitudes.copy(),
        extra_params=base_params.extra_params.copy()
    )

