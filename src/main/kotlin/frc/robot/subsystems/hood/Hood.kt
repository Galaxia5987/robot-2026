package frc.robot.subsystems.hood

import com.ctre.phoenix6.controls.PositionVoltage
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.volts
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

object Hood : SubsystemBase(){
    val hoodMotor = UniversalTalonFX(
        port = PORT,
        config = CONFIG,
        simGains = SIM_GAINS
    )
    @AutoLogOutput
    var setpoint = 0.deg

    @AutoLogOutput(key = "Hood/mechanism")
    private var mechanism = LoggedMechanism2d(5.0, 5.0)
    private var root = mechanism.getRoot("Hood", 2.5, 2.5)
    private val ligament =
        root.append(LoggedMechanismLigament2d("HoodLigament", 1.0, 0.0))
    val positionRequest = PositionVoltage(0.deg)

    fun setPosition(angle: Angle) : Command{
        return Commands.runOnce({
            setpoint = angle
            hoodMotor.setControl(positionRequest.withPosition(angle))
        })
    }


    override fun periodic() {
        hoodMotor.periodic()
        ligament.setAngle(setpoint)
    }
}