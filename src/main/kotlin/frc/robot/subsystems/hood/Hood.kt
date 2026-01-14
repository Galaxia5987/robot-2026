package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase(){
    private val motor = UniversalTalonFX(
        port = PORT,
        config = CONFIG,
        absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
        simGains = SIM_GAINS
    )
    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }
    @AutoLogOutput
    private var setpoint = 0.deg

    var atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, 0.5.deg)
    }

    @AutoLogOutput(key = "Hood/mechanism")
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))

    private val positionRequest = PositionVoltage(0.deg)
    private val voltageRequest = VoltageOut(0.volts)

    fun getToPosition(angle: HoodPositions) : Command{
        return runOnce({
            setpoint = angle.angle
            motor.setControl(positionRequest.withPosition(angle.angle))
        })
    }

    fun getUp() : Command{
        return getToPosition(HoodPositions.UP)
    }

    fun getDown() : Command{
        return getToPosition(HoodPositions.DOWN)
    }

    fun setAngle(voltage: Voltage) : Command{
        return run ({ motor.setControl(voltageRequest.withOutput(voltage))})
    }

    fun angleUpByController() : Command{
        return setAngle(ANGLE_UP_VOLTAGE)
    }

    fun angleDownByController() : Command{
        return setAngle(ANGLE_DOWN_VOLTAGE)
    }

    fun stop() : Command{
        return setAngle(0.volts)
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(setpoint)
    }
}