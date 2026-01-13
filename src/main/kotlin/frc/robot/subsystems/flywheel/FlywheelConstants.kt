package frc.robot.subsystems.flywheel

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.sec

val GAINS = Gains(kP = 0.0, kS = 0.0, kV = 0.0)

val MOTOR_PORT = 0
val MOTOT_PORT2 = 0
val MOTOR_PORT3 = 0

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Coast
                Inverted = InvertedValue.CounterClockwise_Positive
            }
        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 0.0 }
        Slot0 = GAINS.toSlotConfig()

        CurrentLimits =
            createCurrentLimits(
                supplyCurrentLimit = 30.amps,
                supplyCurrentPeakDifference = 10.amps,
                supplyCurrentLowerTime = 0.4.sec
            )
    }
