package frc.robot.subsystems.intake.wrist

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Wrist : SubsystemBase(), WristPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            gearRatio = GEAR_RATIO
        )

    @AutoLogOutput
    private var setPoint = 0.deg

    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    @AutoLogOutput(key = "Wrist/mechanism")
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Wrist", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.0, 0.0))

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
