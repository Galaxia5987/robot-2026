package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.Follower
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC
import com.ctre.phoenix6.signals.MotorAlignmentValue
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.namedRunOnce
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

    val lockMotor =
        UniversalTalonFX(
            LOCK_PORT,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATION
        ) // TODO swap later

    val positionVoltage = MotionMagicTorqueCurrentFOC(0.0)

    var setpoint = ClimbLevels.GROUND

    @LoggedOutput(LogLevel.COMP)
    val isAtSetpoint = Trigger {
        setpoint.angle.isNear(mainMotor.inputs.position, TOLERANCE)
    }

    @LoggedOutput(LogLevel.COMP)
    var isLocked = Trigger { lockSetPoint.isNear(lockMotor.inputs.position, TOLERANCE) }

    private var lockSetPoint = 0.deg

    fun lock(): Command = namedRunOnce {
        lockMotor.setControl(positionVoltage.withPosition(LOCK))
        lockSetPoint = LOCK
    }

    fun unlock(): Command = namedRunOnce {
        lockMotor.setControl(positionVoltage.withPosition(UNLOCK))
        lockSetPoint = UNLOCK
    }

    override fun setTarget(value: ClimbLevels): Command = runOnce {
        mainMotor.setControl(positionVoltage.withPosition(value.angle))
        setpoint = value
    }

    override fun periodic() {
        mainMotor.periodic()
        lockMotor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/lockSetPoint", lockSetPoint)
    }
}
