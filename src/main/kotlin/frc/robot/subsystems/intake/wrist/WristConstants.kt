package frc.robot.subsystems.intake.wrist

import com.ctre.phoenix6.configs.FeedbackConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits

val PORT = 0
//TODO: actual port

val GEAR_RATIO = 1.0
//TODO: actual value

val SIM_GAINS = Gains(kP = 1.0, kD = 0.0)
//TODO: actual values

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
}
