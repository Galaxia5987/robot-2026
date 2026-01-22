package frc.robot.subsystems.intake.extender

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.meters
import frc.robot.lib.extensions.toAngle
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import javax.swing.text.Position

object Extender : SubsystemBase(), ExtenderPositionsCommandFactory {
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
        root.append(LoggedMechanismLigament2d("ExtenderLigament", 1.0, 0.0))

    private val positionRequest = PositionTorqueCurrentFOC(0.0)
    private val voltageOut = VoltageOut(0.0)

    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
    }

    override fun setTarget(value: ExtenderPositions): Command =runOnce{
       setpoint = value.distance
        motor.setControl(positionRequest.withPosition(value.distance.toAngle(DIAMETER,GEAR_RATIO)))
    }

     fun setVoltage(voltage: Voltage) {
       return motor.setControl(voltageOut.withOutput(voltage))
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
    }
}
