package frc.robot.subsystems.shooter.hood

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.controls.VoltageOut
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Units.Degrees
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
import frc.robot.lib.extensions.mps_ps
import frc.robot.lib.extensions.radians
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.volts
import frc.robot.lib.sysid.SysIdable
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import frc.robot.states.setpoints_manager.SetpointsManager.turretTranslationFieldOriented
import org.littletonrobotics.junction.Logger

object Hood : SubsystemBase(), SysIdable {
    private val motor =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            absoluteEncoderOffset = ABSOLUTE_ENCODER_OFFSET,
            simGains = SIM_GAINS,
            logConfig =
                MotorLogConfig(
                    statorCurrent = false,
                    current = false,
                    velocity = false
                )
        )
    private val encoder = CANcoder(ENCODER_ID)

    private var setpoint = 0.radians

    private var shouldCrouchCached = false

    val shouldCrouch: Trigger = Trigger { shouldCrouchCached }

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
        var newAngle =
            if (shouldCrouch.asBoolean) Degrees.zero()
            else angle - HOOD_STARTING_ANGLE

        if (newAngle < 0.deg) {
            newAngle = 0.deg
        }

        setpoint = newAngle
        motor.setControl(positionRequest.withPosition(newAngle))
    }

    fun setAngle(angle: () -> Angle): Command = run { setControlAngle(angle()) }

    private fun computeShouldCrouch(): Boolean {
        if (TRENCH_AREAS.any { it.contains(turretTranslationFieldOriented) }) {
            return true
        }

        val dt = 0.3
        val fieldOrientedSpeeds = drive.fieldOrientedSpeeds

        val fieldOrientedAcceleration =
            ChassisSpeeds.fromRobotRelativeSpeeds(
                ChassisSpeeds(
                    drive.accelerationX[mps_ps],
                    drive.accelerationY[mps_ps],
                    0.0
                ),
                drive.rotation
            )

        val lookAheadTranslation =
            turretTranslationFieldOriented
                .toPose()
                .estimateAt(dt, fieldOrientedSpeeds, fieldOrientedAcceleration)

        Logger.recordOutput(
            "Odometry/LookAheadTranslation",
            lookAheadTranslation.toPose()
        )

        if (TRENCH_AREAS.any { it.contains(lookAheadTranslation) }) {
            return true
        }

        val rectangle =
            Rectangle2d(turretTranslationFieldOriented, lookAheadTranslation)

        for (area in TRENCH_AREAS) {
            if (
                rectangle.contains(
                    Translation2d(
                        area.center.x,
                        (turretTranslationFieldOriented.y +
                            lookAheadTranslation.y) / 2.0
                    )
                )
            ) {
                return true
            }
        }

        return false
    }

    override fun periodic() {
        motor.periodic()

        shouldCrouchCached = computeShouldCrouch()

        Logger.recordOutput("Subsystems/Hood/atSetpoint", atSetpoint)
        Logger.recordOutput("Subsystems/Hood/shouldCrouch", shouldCrouch)
        Logger.recordOutput(
            "Subsystems/Hood/setpoint",
            setpoint[radians],
            radians
        )
    }
}
