package frc.robot.subsystems.climb

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object Climb : SubsystemBase(), ClimbLevelsCommandFactory {

    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot(name, 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("ClimbLigament", 1.0, 0.0))

    private val motor =
        UniversalTalonFX(
            MAIN_PORT,
            config = MOTOR_CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATION
        )

    private val positionVoltage = PositionTorqueCurrentFOC(0.0)

    private var setpoint = ClimbLevels.RETRACTED

    @LoggedOutput(LogLevel.COMP)
    val atSetpoint = Trigger {
        setpoint.angle.isNear(motor.inputs.position, TOLERANCE)
    }

    override fun setTarget(value: ClimbLevels): Command = runOnce {
        setpoint = value
        motor.setControl(positionVoltage.withPosition(value.angle))
    }

    override fun periodic() {
        ligament.angle = motor.inputs.position[deg]
        motor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
    }
}
