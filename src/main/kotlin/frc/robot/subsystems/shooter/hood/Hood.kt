package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.volts
import frc.robot.lib.named
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase(), HoodPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            simGains = SIM_GAINS
        )
    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }
    @AutoLogOutput private var setpoint = 0.deg

    var atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    @AutoLogOutput(key = "Hood/mechanism")
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))

    private val positionRequest = PositionVoltage(0.deg)
    private val voltageRequest = VoltageOut(0.volts)

    fun setVoltage(voltage: Voltage): Command {
        return run{ motor.setControl(voltageRequest.withOutput(voltage)) }
            .named()
    }

    fun angleUpByController(): Command {
        return setVoltage(ANGLE_UP_VOLTAGE).named()
    }

    fun angleDownByController(): Command {
        return setVoltage(ANGLE_DOWN_VOLTAGE).named()
    }

    fun stop(): Command {
        return setVoltage(0.volts).named()
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(setpoint)
    }

    override fun setTarget(value: HoodPositions): Command {
        return runOnce {
            motor.setControl(positionRequest.withPosition(value.angle))
        }
    }
}
