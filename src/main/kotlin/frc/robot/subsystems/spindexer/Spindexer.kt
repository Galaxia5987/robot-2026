package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.subsystems.shooter.turret.ConveyorVelocity
import org.littletonrobotics.junction.Logger

object Spindexer : SubsystemBase(), SpindexerVelocityCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS
        )

    private val velocityVoltage = VelocityVoltage(0.0)

    private var setpoint: SpindexerVelocity = SpindexerVelocity.STOP

    val isAtSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint.velocity, SETPOINT_TOLERANCE)
    }

    fun setTarget(value: ConveyorVelocity): Command = runOnce {
        setpoint = value.velocity.
        mainMotor.setControl(velocityVoltage.withVelocity(value.velocity))
    }

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint.velocity)
        Logger.recordOutput("Subsystems/$name/atSetpoint", isAtSetpoint)
    }
}
