package frc.robot.subsystems.hood

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.Slot0Configs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import com.ctre.phoenix6.signals.NeutralModeValue
import edu.wpi.first.units.AngleUnit
import edu.wpi.first.units.measure.Angle
import frc.robot.lib.Gains
import frc.robot.lib.createCurrentLimits
import kotlin.invoke

val PORT = 0;
// TODO: check port

val SIM_GAINS = Gains(kP = 1.5, kD = 0.22)

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



