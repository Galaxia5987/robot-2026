package frc.robot.subsystems.intake.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Wrist : SubsystemBase(), WristPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            gearRatio = GEAR_RATIO
        )

    private var setPoint = 0.deg

    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))

    private val positionRequest = PositionVoltage(0.deg)
    override fun setTarget(value: WristPositions): Command {
        return runOnce {
            setPoint = value.angle
            motor.setControl(positionRequest.withPosition(value.angle))
        }
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(motor.inputs.position)
    }
}
