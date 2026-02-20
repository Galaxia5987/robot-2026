package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.roller.*

object Roller : SubsystemBase(), RollerPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name
        )

    private val auxMotor =
        UniversalTalonFX(
                port = AUX_MOTOR_PORT,
                config = MOTOR_CONFIG,
                subsystem = name
            )
            .apply {
                setControl(
                    Follower(MAIN_MOTOR_PORT, MotorAlignmentValue.Aligned)
                )
            }

    private val voltageRequest = VoltageOut(0.0)

    val inputs
        get() = motor.inputs

    override fun setTarget(value: RollerPositions): Command = runOnce {
        motor.setControl(voltageRequest.withOutput(value.voltage))
    }

    // --- Alerts ---
    private val disconnectedAlert =
        Alert("$name's motor is disconnected", AlertType.kError)

    private val connectedAlert =
        Alert("$name's motor is connected", AlertType.kInfo)

    fun isConnected(condition: Boolean) : Command = runOnce {
        if (condition) {
            disconnectedAlert.set(false)
            connectedAlert.set(true)
        }
        else {
            disconnectedAlert.set(true)
            connectedAlert.set(false)
        }
    }

    val setStatus = Trigger { motor.inputs.connected }
        .onChange(isConnected(motor.inputs.connected))
    
    override fun periodic() {
        motor.periodic()
    }
}
