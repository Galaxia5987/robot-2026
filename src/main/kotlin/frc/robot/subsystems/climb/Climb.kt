package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object Climb : SubsystemBase(), ClimbLevelsCommandFactory {

    val mainMotor =
        UniversalTalonFX(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATION
        )
    val auxMotor =
        UniversalTalonFX(
                AUX_PORT,
                config = MOTOR_CONFIG,
                simGains = SIM_GAINS,
                gearRatio = GEAR_RATION
            )
            .apply {
                setControl(Follower(MAIN_PORT, MotorAlignmentValue.Aligned))
            }

    val lock =
        UniversalTalonFX(
            LOCK_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATION
        ) // TODO swap later

    val positionVoltage = PositionVoltage(0.0)

    var setpoint = ClimbLevels.DOWN

    @LoggedOutput(LogLevel.COMP)
    val isAtSetpoint = Trigger {
        setpoint.angle.isNear(mainMotor.inputs.position, TOLERANCE)
    }

    var isLocked = false

    fun lock(): Command = runOnce {
        lock.setControl(positionVoltage.withPosition(LOCK))
        isLocked = true
    }

    fun unlock(): Command = runOnce {
        lock.setControl(positionVoltage.withPosition(UNLOCK))
        isLocked = false
    }

    override fun setTarget(value: ClimbLevels): Command = runOnce {
        mainMotor.setControl(positionVoltage.withPosition(value.angle))
        setpoint = value
    }

    override fun periodic() {
        listOf(mainMotor, lock).forEach { it.periodic() }
        Logger.recordOutput("Subsystems/setpoint", setpoint)
        Logger.recordOutput("Subsystems/isLocked", isLocked)
    }
}
