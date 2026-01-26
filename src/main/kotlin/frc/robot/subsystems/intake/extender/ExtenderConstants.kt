package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.configs.CANcoderConfiguration
import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.mm
import frc.robot.lib.extensions.rad
import frc.robot.lib.extensions.rot
import org.team5987.annotation.command_enum.CommandEnum

val DIAMETER = 12.7.mm

val PORT = 0
// TODO: actual port

val GEAR_RATIO = 1.0
// TODO: actual value

val SIM_GAINS = Gains(kP = 1.4, kD = 0.3)

val REAL_GAINS = Gains(kP = 1.4, kD = 0.3)
// TODO: actual values

val ENCODER_ID = 10
// TODO: actual value

val ABSOLUTE_ENCODER_OFFSET = 0.rad
// TODO: actual value

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

        Slot0 = Slot0Configs().apply { REAL_GAINS }
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = GEAR_RATIO }

        CurrentLimits = createCurrentLimits(30.amps, 5.amps)
    }

@CommandEnum
enum class ExtenderPositions(val distance: Distance) {
    OPEN(0.3.meters),
    CLOSE(0.meters)
    // TODO: actual values
}
