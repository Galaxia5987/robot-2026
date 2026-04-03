package frc.robot.lib.universal_motor

import com.ctre.phoenix6.signals.ControlModeValue

data class MotorLogConfig(
    val position: Boolean = true,
    val statorCurrent: Boolean = true,
    val current: Boolean = true,
    val velocity: Boolean = true,
    val absoluteEncoder: Boolean = true,
    val voltage: Boolean = true,
    val controlRequest: Boolean = false // TODO: Change after mock comp
)
