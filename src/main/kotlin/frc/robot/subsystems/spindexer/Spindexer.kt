package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import org.littletonrobotics.junction.Logger

object Spindexer : SubsystemBase(), SpindexerVelocityCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS
        )

    val inputs
        get() = mainMotor.inputs
    private val velocityVoltage = VelocityVoltage(0.0)

    private var setpoint: SpindexerVelocity = SpindexerVelocity.STOP

    val isAtSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint.velocity, SETPOINT_TOLERANCE)
    }

    override fun setTarget(value: SpindexerVelocity): Command = runOnce {
        setpoint = value
        mainMotor.setControl(velocityVoltage.withVelocity(value.velocity))
    }

    // --- Alerts ---
    private val disconnectedAlert =
        Alert("$name's motor is disconnected", AlertType.kWarning)

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

    val setStatus = Trigger { inputs.connected }
        .onChange(isConnected(inputs.connected))

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint.velocity[rad_ps],
            rad_ps
        )
        Logger.recordOutput("Subsystems/$name/atSetpoint", isAtSetpoint)
    }
}
