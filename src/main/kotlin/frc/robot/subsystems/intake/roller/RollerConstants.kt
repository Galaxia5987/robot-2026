package frc.robot.subsystems.intake.roller

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

@CommandEnum
enum class RollerPositions(val velocity: AngularVelocity) {
    SLOW(5.rps),
    OUTTAKE((-10).rps),
    STOP(0.rps)
}

val REAL_GAINS = Gains(kP = 0.2, kV = 0.5)
val SIM_GAINS = Gains(kP = 1.0)

val INTAKE_BASE_SPEED = 25.rps

const val ROBOT_VELOCITY_MULTIPLIER = 10.0

val NORMAL_CURRENT_LIMITS = createCurrentLimits(30.amps, 5.amps)
val HIGH_CURRENT_LIMITS = createCurrentLimits(60.amps, 5.amps)

const val PORT = 10
const val GEAR_RATIO = 2.5

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = REAL_GAINS.toSlotConfig()
        CurrentLimits = NORMAL_CURRENT_LIMITS
        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = GEAR_RATIO
            }
    }
