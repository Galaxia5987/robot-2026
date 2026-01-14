package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import com.ctre.phoenix6.signals.SensorDirectionValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rps

const val PORT = 0
const val RATIO = 1.0
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 0.5, kD = 0.075)
val SETPOINT_TOLERANCE = 1.deg
val ENCODER_ID = 0
val ABSOLUTE_ENCODER_OFFSET = 0.rps

val ENCODER_CONFIG =
    CANcoderConfiguration().apply {
        MagnetSensor.SensorDirection =
            SensorDirectionValue.CounterClockwise_Positive
        MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.9 // ?
        MagnetSensor.MagnetOffset = ABSOLUTE_ENCODER_OFFSET[rps] //?
    }

val CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 =
            Slot0Configs().apply { //
                Slot0 = REAL_GAINS.toSlotConfig() // ?
            }
        Feedback =
            FeedbackConfigs().apply { // ?
                SensorToMechanismRatio = RATIO
                FeedbackRemoteSensorID = ENCODER_ID
                FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder
            }
    }
