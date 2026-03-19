package frc.robot.subsystems.spindexer

import com.ctre.phoenix6.configs.CurrentLimitsConfigs
import com.ctre.phoenix6.controls.VelocityVoltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.rad_ps
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Spindexer : SubsystemBase(), SpindexerVelocityCommandFactory {
    private val mainMotor =
        UniversalTalonFX(
            MAIN_MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO,
            simGains = SIM_GAINS,
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

    fun setCurrentLimits(limits: CurrentLimitsConfigs): Command =
        Commands.runOnce({
            mainMotor.applyConfiguration(MOTOR_CONFIG.withCurrentLimits(limits))
        })

    fun setRegularCurrentLimits() = setCurrentLimits(REGULAR_CURRENT_LIMITS)
    fun setLowCurrentLimits() = setCurrentLimits(LOW_CURRENT_LIMITS)

    override fun periodic() {
        mainMotor.periodic()
        Logger.recordOutput(
            "Subsystems/Spindexer/setpoint",
            setpoint.velocity[rad_ps],
            rad_ps
        )
        Logger.recordOutput("Subsystems/Spindexer/atSetpoint", isAtSetpoint)
    }
}
