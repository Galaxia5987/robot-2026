package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.createDisableTriggerForCoast
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.radians
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Turret : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS
        )
    private val absoluteEncoder = CANcoder(ENCODER_ID)

    private val positionVoltage: PositionVoltage = PositionVoltage(0.0)
    private var setpoint = 0.radians

    val wrappedPosition: Angle
        get() {
            val raw = motor.inputs.position
            return if (raw > 180.0.deg) raw - 360.0.deg
            else if (raw < (-180.0).deg) raw + 360.0.deg else raw
        }
    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, SETPOINT_TOLERANCE)
    }

    init {
        absoluteEncoder.configurator.apply(ENCODER_CONFIG)
        motor.reset(absoluteEncoder.absolutePosition.value)
        createDisableTriggerForCoast(motor)
    }

    fun setAngle(angle: Angle) = runOnce {
        setpoint = angle
        motor.setControl(positionVoltage.withPosition(angle))
    }

    fun setAngle(angleSupplier: () -> Angle) = run {
        setpoint = angleSupplier()
        motor.setControl(positionVoltage.withPosition(setpoint))
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint[radians],
            radians
        )
        Logger.recordOutput("Subsystems/$name/wrappedPosition", wrappedPosition)
        Logger.recordOutput("Subsystems/$name/atSetpoint", atSetpoint)
    }
}
