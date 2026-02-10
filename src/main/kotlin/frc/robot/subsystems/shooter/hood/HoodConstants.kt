package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.*
import org.team5987.annotation.command_enum.CommandEnum

const val PORT = 15
// TODO: check port

const val ENCODER_ID = 4
const val ENCODER_GEAR_RATIO = 0.5 // TODO: Might be 2

val TOLERANCE = 0.5.deg

val SIM_GAINS = Gains(kP = 1.7, kD = 0.3)
val REAL_GAINS = Gains(kP = 1.7)

const val GEAR_RATIO = 156.83

val ABSOLUTE_ENCODER_OFFSET = 0.rad

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

        Slot0 = REAL_GAINS.toSlotConfig()
        Feedback =
            FeedbackConfigs().apply {
                SensorToMechanismRatio = GEAR_RATIO
                RotorToSensorRatio = ENCODER_GEAR_RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }

        CurrentLimits = createCurrentLimits(15.amps, 5.amps)
    }

@CommandEnum
enum class HoodPositions(var angle: Angle) {
    UP(10.deg),
    DOWN(0.deg)
}
// TODO: real values
