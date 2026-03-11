package frc.robot.subsystems.intake.funnel

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX

object Funnel : SubsystemBase() {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = MOTOR_CONFIG,
            subsystem = name,
            logConfig =
                MotorLogConfig(
                    position = false,
                    statorCurrent = false,
                    current = false,
                    velocity = false,
                    absoluteEncoder = false,
                    voltage = true
                )
        )

    private val voltageOut = VoltageOut(0.0)

    private fun setControl(voltage: Voltage): Command = runOnce { motor.setControl(voltageOut.withOutput(voltage)) }


    fun start(): Command = setControl(START_VOLTAGE)
    fun stop(): Command = setControl(0.volts)

    override fun periodic() {
        motor.periodic()
    }
}
