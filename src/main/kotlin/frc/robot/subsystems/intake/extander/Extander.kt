package frc.robot.subsystems.intake.extander

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.meters
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Extander : SubsystemBase(), SysIdable, PositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            simGains = SIM_GAINS,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            gearRatio = GEAR_RATIO
        )

    private var setpoint = 0.meters

    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot(name, 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("WristLigament", 1.0, 0.0))

    private val positionRequest = PositionTorqueCurrentFOC(0.deg)
    private val voltageOut = VoltageOut(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    override fun setTarget(value: ExtanderPositions): Command = runOnce {
        setpoint = value.
        motor.setControl(positionRequest.withPosition(value.angle))
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun periodic() {
        ligament.setAngle(motor.inputs.position)
        motor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
    }
}
