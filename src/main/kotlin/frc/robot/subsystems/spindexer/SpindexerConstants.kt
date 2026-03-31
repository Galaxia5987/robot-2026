package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum

val GAINS = Gains(kP = 0.4, kV = 1.1)

val SIM_GAINS = Gains(1.0)

const val MAIN_MOTOR_ID = 12

const val GEAR_RATIO = 9.0

val SETPOINT_TOLERANCE = 1.0.rps

val REGULAR_CURRENT_LIMITS =
    createCurrentLimits(
        supplyCurrentLimit = 40.amps,
        supplyCurrentPeakDifference = 20.amps
    )

val LOW_CURRENT_LIMITS =
    createCurrentLimits(
        supplyCurrentLimit = 10.amps,
        supplyCurrentPeakDifference = 5.amps
    )

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = GAINS.toSlotConfig()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        CurrentLimits = REGULAR_CURRENT_LIMITS
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }

@CommandEnum
enum class SpindexerVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    START(8.rps),
    SLOW(4.rps),
    REVERSE((-10).rps),
    REVERSE_SLOW(-SLOW.velocity)
}
