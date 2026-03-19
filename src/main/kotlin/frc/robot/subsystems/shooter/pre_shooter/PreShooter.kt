package frc.robot.subsystems.shooter.pre_shooter

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.kg2m
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.extensions.radiansPerSecond
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object PreShooter : SubsystemBase(), PreShooterVelocityCommandFactory {

    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
            momentOfInertia = 0.03.kg2m,
            logConfig =
                MotorLogConfig(
                    position = false,
                    statorCurrent = false,
                    current = false,
                    velocity = true,
                    absoluteEncoder = false,
                    voltage = true
                )
        )
    private val velocityVoltage = VelocityVoltage(0.0)

    private var setpoint = 0.0.radiansPerSecond

    val atSetpoint = Trigger {
        mainMotor.inputs.velocity.isNear(setpoint, SETPOINT_TOLERANCE)
    }
    val inputs
        get() = mainMotor.inputs

    private fun setVelocityControl(velocity: AngularVelocity) {
        setpoint = velocity
        mainMotor.setControl(velocityVoltage.withVelocity(velocity))
    }

    fun setVelocity(velocity: AngularVelocity): Command = runOnce {
        setVelocityControl(velocity)
    }

    fun setVelocity(velocity: () -> AngularVelocity): Command = run {
        setVelocityControl(velocity())
    }

    override fun setTarget(value: PreShooterVelocity): Command =
        setVelocity(value.velocity)

    fun setCurrentLimits(limits: CurrentLimitsConfigs): Command =
        Commands.runOnce({
            mainMotor.applyConfiguration(MOTOR_CONFIG.withCurrentLimits(limits))
        })

    fun setRegularCurrentLimits() = setCurrentLimits(REGULAR_CURRENT_LIMITS)
    fun setLowCurrentLimits() = setCurrentLimits(LOW_CURRENT_LIMITS)

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput(
            "Subsystems/PreShooter/setpoint",
            setpoint[rad_ps],
            rad_ps
        )
        Logger.recordOutput("Subsystems/PreShooter/atSetpoint", atSetpoint)
    }
}
