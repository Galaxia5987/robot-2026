package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase(){
    private val hoodMotor = UniversalTalonFX(
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
        hoodMotor.inputs.position.isNear(setpoint, 0.5.deg)
    }

    @AutoLogOutput(key = "Hood/mechanism")
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))
    private val positionRequest = PositionVoltage(0.deg)

    fun getToPosition(angle: Angle) : Command{
        return runOnce({
            setpoint = angle
            hoodMotor.setControl(positionRequest.withPosition(angle))
        })
    }

    fun setPosition(angle: Angle): Command{
        return run ({ hoodMotor.setControl(positionRequest.withPosition(1.deg))})
    }


    override fun periodic() {
        hoodMotor.periodic()
        ligament.setAngle(setpoint)
    }
}