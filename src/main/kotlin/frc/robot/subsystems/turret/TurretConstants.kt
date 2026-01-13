package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.degrees

const val PORT = 0
const val RATIO = 0.0
val SIM_GAINS = Gains(kP = 0.0, kD = 0.0)
val LIMIT_EXCEEDED = 1.degrees

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 =
            Slot0Configs().apply {
                kP = 0.0
                kD = 0.0
            }
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = 0.0 / 0.0 }
    }