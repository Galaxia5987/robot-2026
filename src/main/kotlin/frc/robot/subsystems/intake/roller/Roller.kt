package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.VoltageOut
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX

object Roller : SubsystemBase(), RollerPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(port = PORT, config = MOTOR_CONFIG, subsystem = name)

    private val voltageRequest = VoltageOut(0.0)

    val inputs
        get() = motor.inputs

    override fun setTarget(value: RollerPositions): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(value.voltage))
    }

    override fun periodic() {
        motor.periodic()
    }
}
