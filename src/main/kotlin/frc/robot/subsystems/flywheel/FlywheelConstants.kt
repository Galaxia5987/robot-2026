package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.MotorAlignmentValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.rps
import org.team5987.annotation.command_enum.CommandEnum

const val GEAR_RATIO = 1
val REAL_GAINS = Gains(kP = 0.0, kS = 0.0, kV = 0.0)
val SIM_GAINS = Gains(kP = 1.0, kS = 0.0)

val MAIN_MOTOR_PORT = 0

val AUXILIARY_MOTORS_PORTS =
    mapOf<Int, MotorAlignmentValue>(
        0 to MotorAlignmentValue.Aligned,
        1 to MotorAlignmentValue.Aligned
    )

@CommandEnum
enum class FlywheelVelocities(val velocity: AngularVelocity) {
    NEAR(30.0.rps),
    MEDIUM(40.0.rps),
    FAR(50.0.rps),
    ZERO(0.rps)
}

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { GEAR_RATIO }
        Slot0 = REAL_GAINS.toSlotConfig()

        CurrentLimits =
            createCurrentLimits(
                supplyCurrentLimit = 30.amps,
                supplyCurrentPeakDifference = 10.amps
            )
    }
