package frc.robot.subsystems.conveyor.netConveyor

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import frc.robot.lib.LoggedNetworkGains
import frc.robot.lib.createCurrentLimits

val GAINS = LoggedNetworkGains("NetConveyorGains", 1.0)
val SIM_GAINS = LoggedNetworkGains("NetConveyorGains", 1.0)
val MOTOR_CONFIG = TalonFXConfiguration().apply {
    Slot0 = GAINS.toSlotConfig()
    CurrentLimits = createCurrentLimits()
    MotorOutput = MotorOutputConfigs().apply {
        Inverted = InvertedValue.Clockwise_Positive
    }
}