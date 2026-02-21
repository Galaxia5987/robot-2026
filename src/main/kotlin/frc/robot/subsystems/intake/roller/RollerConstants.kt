package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.volts
import org.team5987.annotation.command_enum.CommandEnum

@CommandEnum
enum class RollerPositions(val velocity: AngularVelocity) {
    INTAKE(25.rps),
    OUTTAKE((-10).rps),
    STOP(0.rps)
}

val REAL_GAINS = Gains(kP = 10.0, kV = 0.12)
val SIM_GAINS = Gains(kP = 1.0)

const val PORT = 10
const val GEAR_RATIO = 2.5

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
        Slot0 = REAL_GAINS.toSlotConfig()

        CurrentLimits = createCurrentLimits(30.amps, 5.amps)
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }
    }
