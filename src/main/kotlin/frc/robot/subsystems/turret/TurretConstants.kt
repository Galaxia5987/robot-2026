package frc.robot.subsystems.turret

import com.ctre.phoenix6.configs.*
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.degrees

const val PORT = 0
const val RATIO = 1.0
val SIM_GAINS = Gains(kP = 0.5, kD = 0.075)
val REAL_GAINS = Gains(kP = 1.0, kD = 0.0)
val SETPOINT_TOLERANCE = 1.degrees

val config =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits()
        Slot0 =
            Slot0Configs().apply { //
                Slot0 = REAL_GAINS.toSlotConfig()
            }
        Feedback =
            FeedbackConfigs().apply { SensorToMechanismRatio = RATIO }
    }