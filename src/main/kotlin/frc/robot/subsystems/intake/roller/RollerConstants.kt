package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Voltage
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts
import org.team5987.annotation.command_enum.CommandEnum

@CommandEnum
enum class RollerPositions(val voltage: Voltage) {
    INTAKE(6.volts),
    OUTTAKE((-6.0).volts),
    STOP(0.volts)
}

val MAIN_MOTOR_PORT = 0
val AUX_MOTOR_PORT = 0

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits(30.amps, 5.amps)
    }
