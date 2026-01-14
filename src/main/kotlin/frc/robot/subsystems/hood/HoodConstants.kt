package frc.robot.subsystems.hood

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
import frc.robot.lib.extensions.volts

val PORT = 0
// TODO: check port

val ENCODER_ID = 10

val ANGLE_UP_VOLTAGE = 0.013.volts
val ANGLE_DOWN_VOLTAGE = -0.013.volts
// TODO: real values


val SIM_GAINS = Gains(kP = 1.7, kD = 0.32)

val GEAR_RATIO = 1.0

val ABSOLUTE_ENCODER_OFFSET = 0.rad

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rot]
    }

val CONFIG = TalonFXConfiguration().apply {
    MotorOutput =
        MotorOutputConfigs().apply {
            NeutralMode = NeutralModeValue.Brake
            Inverted = InvertedValue.Clockwise_Positive
            // TODO: check motor direction
    }

    CurrentLimits = createCurrentLimits()

    Slot0 =
        Slot0Configs().apply {
            kP = 1.0
            kD = 0.0
            // TODO: actual values
        }
    Feedback =
        FeedbackConfigs().apply {
            SensorToMechanismRatio = GEAR_RATIO
        }
}

enum class HoodPositions(var angle: Angle){
    UP(10.deg),
    DOWN(0.deg)
}
// TODO: real values