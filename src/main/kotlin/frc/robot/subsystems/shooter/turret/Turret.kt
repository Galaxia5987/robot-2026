package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.radians
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.math.atan2

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

    private val positionVoltageRequest: PositionVoltage = PositionVoltage(0.0)
    @LoggedOutput(LogLevel.DEV) var setpoint = 0.deg
    @LoggedOutput(LogLevel.DEV) val isAtSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    fun setAngle(angle: Angle) = runOnce {
        setpoint = angle
        motor.setControl(positionVoltageRequest.withPosition(angle))
    }

    fun setAngle(angleSupplier: () -> Angle) = run {
        setpoint = angleSupplier()
        motor.setControl(positionVoltageRequest.withPosition(setpoint))
    }

    fun resetAngle() = runOnce {
        setpoint = 0.deg
        motor.setControl(positionVoltageRequest.withPosition(0.0))
    }

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG) // What does it do?
    }

    override fun periodic() {
        motor.periodic()
        ligament.setAngle(motor.inputs.position)
        Logger.recordOutput("Turret", mechanism)
    }
}