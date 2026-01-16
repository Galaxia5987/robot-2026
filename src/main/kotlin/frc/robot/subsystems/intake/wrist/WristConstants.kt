package frc.robot.subsystems.intake.wrist

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.rot
import org.team5987.annotation.command_enum.CommandEnum

val PORT = 0
// TODO: actual port

val GEAR_RATIO = 1.0
// TODO: actual value

val SIM_GAINS = Gains(kP = 1.33, kD = 0.24)

val REAL_GAINS = Gains(kP = 1.0, kD = 0.0)
// TODO: actual values

val ENCODER_ID = 10
//TODO: actual value

val ABSOLUTE_ENCODER_OFFSET = 0.rad
//TODO: actual value

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rot]
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.Clockwise_Positive
                // TODO: check motor direction
            }

        CurrentLimits = createCurrentLimits()

        Slot0 =
            Slot0Configs().apply {
                REAL_GAINS
            }

        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = GEAR_RATIO
            }
    }



@CommandEnum
enum class WristPositions(var angle: Angle) {
    INTAKE(0.deg),
    UP(30.deg)
    // TODO: actual values
}
