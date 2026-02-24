package frc.robot.lib.universal_motor

data class MotorLogConfig(
    val position: Boolean = true,
    val statorCurrent: Boolean = true,
    val current: Boolean = true,
    val velocity: Boolean = true,
    val absoluteEncoder: Boolean = true,
    val voltage: Boolean = true
)
