package frc.robot.subsystems.intake.roller

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.VelocityVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.rps
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Roller : SubsystemBase(), RollerPositionsCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            port = MAIN_PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            subsystem = name,
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

    private val auxMotor = UniversalTalonFX(
        port = AUX_PORT,
        config = MOTOR_CONFIG,
        simGains = SIM_GAINS,
        subsystem = name,
        logConfig =
            MotorLogConfig(
                position = false,
                statorCurrent = false,
                current = false,
                velocity = true,
                absoluteEncoder = false,
                voltage = true
            )
    ).apply {
        setControl(Follower(MAIN_PORT, MotorAlignmentValue.Opposed))
    }

    private val velocityVoltage = VelocityVoltage(0.0)
    private var setpoint: AngularVelocity = 0.rps

    val isActive = Trigger { setpoint > 0.rps }

    val inputs
        get() = mainMotor.inputs

    override fun setTarget(value: RollerPositions): Command = runOnce {
        setpoint = value.velocity
        mainMotor.setControl(velocityVoltage.withVelocity(value.velocity))
    }

    private fun setControl(velocity: AngularVelocity) {
        setpoint = velocity
        mainMotor.setControl(velocityVoltage.withVelocity(velocity))
    }

    fun intake(): Command = defer { run { setControl(INTAKE_BASE_SPEED) } }

    private fun setCurrentLimits(
        currentLimitConfig: CurrentLimitsConfigs
    ): Command = runOnce {
        mainMotor.applyConfiguration(
            MOTOR_CONFIG.withCurrentLimits(currentLimitConfig)
        )
        auxMotor.applyConfiguration(
            MOTOR_CONFIG.withCurrentLimits(currentLimitConfig)
        )
    }

    fun setHighCurrentLimits(): Command = setCurrentLimits(HIGH_CURRENT_LIMITS)
    fun setNormalCurrentLimits(): Command =
        setCurrentLimits(NORMAL_CURRENT_LIMITS)

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput("Subsystems/Roller/setpoint", setpoint)
    }
}
