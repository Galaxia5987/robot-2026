package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Climb : SubsystemBase(), ClimbLevelsCommandFactory {
    private val motor =
        UniversalTalonFX(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            logConfig =
                MotorLogConfig(
                    position = true,
                    statorCurrent = false,
                    current = false,
                    velocity = false,
                    absoluteEncoder = false,
                    voltage = true
                )
        )

    private val voltageOut = VoltageOut(0.0)

    private val positionTorque = PositionTorqueCurrentFOC(0.0)
    private var setpoint = ClimbLevels.RETRACTED

    val atSetpoint = Trigger {
        setpoint.height.isNear(motor.inputs.distance, TOLERANCE)
    }

    override fun setTarget(value: ClimbLevels): Command = runOnce {
        setpoint = value
        motor.setControl(
            positionTorque.withPosition(
                value.height.toAngle(SPROCKET_DIAMETER, GEAR_RATIO)
            )
        )
    }

    private fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    fun stop(): Command = setVoltage(0.volts)
    fun up(): Command = setVoltage(3.volts)
    fun down(): Command = setVoltage((-3).volts)

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Climb/setpoint", setpoint)
        Logger.recordOutput("Subsystems/Climb/atSetpoint", setpoint)
    }
}
