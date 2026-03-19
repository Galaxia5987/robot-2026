package frc.robot.subsystems.shooter.flywheel

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
import frc.robot.lib.switchable
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import org.team5987.annotation.command_enum.CommandEnum

const val GEAR_RATIO = 1.33
val REAL_GAINS = Gains(kP = 0.3, kS = 0.3, kV = 0.115)
val SIM_GAINS = Gains(kP = 1.0, kV = 0.12)

const val MAIN_MOTOR_PORT = 16

val FLYWHEEL_TOLERANCE
    get() = isShootingOnMove.asBoolean.switchable(4.rps, 0.55.rps)

val AUXILIARY_MOTORS_PORTS =
    mapOf<Int, MotorAlignmentValue>(
        17 to MotorAlignmentValue.Opposed,
        18 to MotorAlignmentValue.Aligned
    )

@CommandEnum
enum class FlywheelVelocities(val velocity: AngularVelocity) {
    NEAR(30.0.rps),
    MEDIUM(40.0.rps),
    FAR(50.0.rps),
    ZERO(0.rps)
}

val REGULAR_CURRENT_LIMITS =
    createCurrentLimits(
        supplyCurrentLimit = 30.amps,
        supplyCurrentPeakDifference = 5.amps
    )

val LOW_CURRENT_LIMITS =
    createCurrentLimits(
        supplyCurrentLimit = 20.amps,
        supplyCurrentPeakDifference = 5.amps
    )

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { GEAR_RATIO }
        Slot0 = REAL_GAINS.toSlotConfig()

        CurrentLimits = REGULAR_CURRENT_LIMITS
    }
