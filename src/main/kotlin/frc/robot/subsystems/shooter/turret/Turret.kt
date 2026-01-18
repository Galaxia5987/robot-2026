package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.deg
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d

@AutoLogOutput(key = "Turret/mechanism")
private var mechanism = LoggedMechanism2d(5.0, 5.0)
private var root = mechanism.getRoot("Turret", 2.5, 2.5)
private val ligament =
    root.append(LoggedMechanismLigament2d("TurretLigament", 1.0, 0.0))

object Turret : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS
        )
    private val absoluteEncoder = CANcoder(ENCODER_ID)

    private val positionTorqueCurrentFOC: PositionTorqueCurrentFOC = PositionTorqueCurrentFOC(0.0)
    private var setpoint = 0.deg
    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
    }

    fun setAngle(angle: Angle) = runOnce {
        setpoint = angle
        motor.setControl(positionTorqueCurrentFOC.withPosition(angle))
    }

    fun setAngle(angleSupplier: () -> Angle) = run {
        setpoint = angleSupplier()
        motor.setControl(positionTorqueCurrentFOC.withPosition(setpoint))
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(motor.inputs.position)
        Logger.recordOutput("Subsystems/$name/mechanism", mechanism)
        Logger.recordOutput("Subsystems/$name/setpoint", setpoint)
        Logger.recordOutput("Subsystems/$name/atSetpoint", isAtSetpoint)
    }
}