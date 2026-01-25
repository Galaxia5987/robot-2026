package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.volts
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase(), SysIdable, HoodPositionsCommandFactory {
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))

    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            simGains = SIM_GAINS
        )
    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint = 0.deg
    var atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    private val positionRequest = PositionTorqueCurrentFOC(0.deg)
    private val voltageRequest = VoltageOut(0.volts)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setControlAngle(angle: Angle) {
        setpoint = angle
        motor.setControl(positionRequest.withPosition(angle))
    }

    fun setAngle(angle: Angle): Command = runOnce { setControlAngle(angle) }

    fun setAngle(angle: () -> Angle): Command = run { setControlAngle(angle()) }

    override fun setTarget(value: HoodPositions): Command =
        setAngle(value.angle)

    override fun periodic() {
        ligament.setAngle(setpoint)
        motor.periodic()
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
        Logger.recordOutput("Subsystems/$name/atSetpoint", atSetpoint)
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
    }
}
