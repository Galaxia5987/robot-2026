package frc.robot.subsystems.roller

import com.ctre.phoenix6.configs.MotorOutputConfigs
import com.ctre.phoenix6.configs.TalonFXConfiguration
import com.ctre.phoenix6.signals.InvertedValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.createCurrentLimits
import frc.robot.lib.extensions.amps
import frc.robot.lib.extensions.volts
import org.team5987.annotation.command_enum.CommandEnum

@CommandEnum
enum class positions(val voltage: Voltage) {
    INTAKE(0.volts),
    OUTTAKE(0.volts),
    STOP(0.volts)
}

interface positionActions {
    fun setVoltage(voltage: Voltage): Command
}

val MAIN_MOTOR_PORT = 0

val AUXILIARY_MOTORS_PORTS = listOf(0, 0)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        MotorOutput =
            MotorOutputConfigs().apply {
                Inverted = InvertedValue.Clockwise_Positive
            }
        CurrentLimits = createCurrentLimits(supplyCurrentLimit = 00.amps)//defult number
    }

