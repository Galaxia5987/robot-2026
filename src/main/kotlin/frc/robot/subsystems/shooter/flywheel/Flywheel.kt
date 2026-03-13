package frc.robot.subsystems.shooter.flywheel

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.extensions.radiansPerSecond
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

object Flywheel : SubsystemBase(), FlywheelVelocitiesCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = MAIN_MOTOR_PORT,
            config = MOTOR_CONFIG,
            subsystem = name,
            simGains = SIM_GAINS,
            logConfig =
                MotorLogConfig(
                    position = false,
                    statorCurrent = false,
                    absoluteEncoder = false
                )
        )

    private val velocityOut = VelocityVoltage(0.0)
    private var setpoint = 0.radiansPerSecond

    val isActive = Trigger { setpoint > 0.rps }

    val inputs
        get() = motor.inputs

    init {
        AUXILIARY_MOTORS_PORTS.forEach {
            UniversalTalonFX(
                    port = it.key,
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
                .setControl(Follower(MAIN_MOTOR_PORT, it.value))
        }
    }

    val calibrationVelocity =
        LoggedNetworkNumber(
            "/Tuning/Flywheel/calibrationFlywheelVelocity",
            40.0
        )

    val atSetpoint =
        Trigger { motor.inputs.velocity.isNear(setpoint, FLYWHEEL_TOLERANCE) }
            .debounce(0.13, Debouncer.DebounceType.kFalling)

    fun setVelocity(velocity: () -> AngularVelocity): Command = run {
        setpoint = velocity.invoke()
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    override fun setTarget(value: FlywheelVelocities): Command = runOnce {
        setpoint = value.velocity
        motor.setControl(velocityOut.withVelocity(setpoint))
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/Flywheel/atSetpoint", atSetpoint)
        Logger.recordOutput(
            "Subsystems/Flywheel/setpoint",
            setpoint[rad_ps],
            rad_ps
        )
    }
}
