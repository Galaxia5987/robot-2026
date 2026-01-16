package frc.robot.subsystems.roller

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.universal_motor.UniversalTalonFX

class Roller : SubsystemBase(), positionActions {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name
        )
    @AutoLogOutput private var setpoint = 0.rps

    private val velocity
        get() = motor.inputs.velocity
    init {
        AUXILIARY_MOTORS_PORTS.forEach {
            UniversalTalonFX(port = it, config = MOTOR_CONFIG, subsystem = name)
                .setControl(
                    Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
                )
        }
    }
    private val voltageRequest = VoltageOut(0.0)

    override fun setVoltage(voltage: Voltage): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
        setpoint=velocity
    }



    override fun periodic() {
        motor.periodic()
    }
}
