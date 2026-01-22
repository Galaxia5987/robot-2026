package frc.robot.subsystems.shooter.pre_shooter

import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object PreShooter : SubsystemBase(), PreShooterVelocityCommandFactory {

    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS
        )
    private val velocityVoltage = VelocityVoltage(0.0)

    private var setpoint = PreShooterVelocity.STOP

    val isAtSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint.velocity, SETPOINT_TOLERANCE)
    }

    override fun setTarget(value: PreShooterVelocity): Command = runOnce {
        setpoint = value
        mainMotor.setControl(velocityVoltage.withVelocity(value.velocity))
    }

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint.velocity)
        Logger.recordOutput("Subsystems/$name/atSetpoint", isAtSetpoint)
    }
}
