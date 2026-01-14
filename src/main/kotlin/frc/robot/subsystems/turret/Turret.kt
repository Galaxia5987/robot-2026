package frc.robot.subsystems.turret

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.degrees
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

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
            simGains = SIM_GAINS,
        )
    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    @LoggedOutput(LogLevel.DEV) var setpoint = 0.degrees

    @LoggedOutput(LogLevel.DEV)
    val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    fun setAngle(angle: Angle) = runOnce {
        setpoint = angle
        motor.setControl(positionVoltageRequest.withPosition(angle))
    }

    fun setAngle(angleSupplier: () -> Angle) = run { // ?
        val angle = angleSupplier()
        setpoint = angleSupplier()
        motor.setControl(positionVoltageRequest.withPosition(setpoint))
    }

    private val encoder = CANcoder(ENCODER_ID)

    init {
        encoder.configurator.apply(ENCODER_CONFIG) // ?
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(motor.inputs.position)
        Logger.recordOutput("Turret", mechanism)
    }
}