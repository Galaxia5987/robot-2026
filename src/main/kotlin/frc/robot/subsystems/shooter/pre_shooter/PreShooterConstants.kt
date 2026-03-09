package frc.robot.subsystems.shooter.pre_shooter

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.measure.AngularVelocity
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.rps
import frc.robot.lib.switchable
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import org.team5987.annotation.command_enum.CommandEnum

val REAL_GAINS = Gains(kP = 1.0, kV = 0.2, kS = 2.0)

val SIM_GAINS = Gains(0.6, kV = 0.1)

const val MAIN_MOTOR_ID = 13

const val GEAR_RATIO = 2.5

val SETPOINT_TOLERANCE
    get() = isShootingOnMove.asBoolean.switchable(4.rps, 2.rps)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = REAL_GAINS.toSlotConfig()
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
                NeutralMode = NeutralModeValue.Brake
            }
        CurrentLimits = createCurrentLimits(30.amps, 10.amps)
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }

@CommandEnum
enum class PreShooterVelocity(val velocity: AngularVelocity) {
    STOP(0.rps),
    SHOOTING(25.rps),
    REVERSE(((-10).rps)),
}
