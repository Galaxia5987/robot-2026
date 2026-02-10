package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Extender : SubsystemBase(), ExtenderPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO,
            linearSystemWheelDiameter = DIAMETER
        )

    private val positionRequest = PositionVoltage(0.0)

    private var setpoint = 0.meters

    val atSetpoint = Trigger {
        setpoint.isNear(motor.inputs.distance, TOLERANCE)
    }
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot(name, 2.5, 2.5)

    private val ligament =
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 1.0, 0.0))

    val inputs
        get() = motor.inputs

    init {
        motor.reset()
    }

    override fun setTarget(value: ExtenderPositions): Command = runOnce {
        setpoint = value.distance
        Logger.recordOutput("Test", value.distance.toAngle(DIAMETER, GEAR_RATIO))
        motor.setControl(
            positionRequest.withPosition(
                value.distance.toAngle(DIAMETER, GEAR_RATIO)
            )
        )
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint[meters],
            meters
        )
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
    }
}
