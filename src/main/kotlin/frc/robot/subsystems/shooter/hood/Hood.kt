package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.Voltage
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.TRENCH_AREAS
import frc.robot.lib.createDisableTriggerForCoast
import frc.robot.lib.estimateAt
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.radians
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Hood : SubsystemBase(), SysIdable, HoodPositionsCommandFactory {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            simGains = SIM_GAINS
        )
    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint = 0.radians

    val shouldCrouch: Trigger = Trigger {
        TRENCH_AREAS.any { it.contains(drive.pose.translation) }
            .or(
                TRENCH_AREAS.any {
                    val speeds = drive.chassisSpeeds
                    it.contains(drive.pose.estimateAt(0.2.sec, speeds))
                }
            )
    }

    val atSetpoint = Trigger {
        motor.inputs.position.isNear(setpoint, TOLERANCE)
    }

    private val positionRequest = PositionVoltage(0.deg)
    private val voltageRequest = VoltageOut(0.volts)
    val inputs
        get() = motor.inputs

    init {
        encoder.configurator.apply(ENCODER_CONFIG)
        createDisableTriggerForCoast(motor)
    }

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setControlAngle(angle: Angle) {
        setpoint =
            if (shouldCrouch.asBoolean) HoodPositions.DOWN.angle
            else angle - HOOD_STARTING_ANGLE
        motor.setControl(positionRequest.withPosition(setpoint))
    }

    fun setAngle(angle: Angle): Command = runOnce { setControlAngle(angle) }

    fun setAngle(angle: () -> Angle): Command = run { setControlAngle(angle()) }

    override fun setTarget(value: HoodPositions): Command =
        setAngle(value.angle)

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput("Subsystems/$name/atSetpoint", atSetpoint)
        Logger.recordOutput("Subsystems/$name/shouldCrouch", shouldCrouch)
        Logger.recordOutput(
            "Subsystems/$name/setpoint",
            setpoint[radians],
            radians
        )
    }
}
