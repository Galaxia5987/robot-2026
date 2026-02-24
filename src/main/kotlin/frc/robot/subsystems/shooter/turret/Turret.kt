package frc.robot.subsystems.shooter.turret

import com.ctre.phoenix6.controls.PositionVoltage
import com.ctre.phoenix6.hardware.CANcoder
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.lib.createDisableTriggerForCoast
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.radians
import frc.robot.lib.extensions.rotations
import frc.robot.lib.universal_motor.MotorLogConfig
import frc.robot.lib.universal_motor.UniversalTalonFX
import org.littletonrobotics.junction.Logger

object Turret : SubsystemBase() {
    private val motor: UniversalTalonFX =
        UniversalTalonFX(
            port = PORT,
            config = CONFIG,
            gearRatio = RATIO,
            simGains = SIM_GAINS,
            logConfig =
                MotorLogConfig(
                    statorCurrent = false,
                    current = false,
                )
        )
    private val absoluteEncoder = CANcoder(ENCODER_ID)

    private val positionVoltage: PositionVoltage = PositionVoltage(0.0)
    private var setpoint = 0.radians
    private var lastSetpoint = 0.radians
    private var lastTime = Timer.getFPGATimestamp()
    private var setpointVelocityRps = 0.0

    val wrappedPosition: Angle
        get() = motor.inputs.position

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
        val newSetpoint = angleSupplier()
        val currentTime = Timer.getFPGATimestamp()
        val dt = currentTime - lastTime

        // Calculate discrete derivative to find setpoint velocity
        if (dt > 0.0) {
            val deltaRotations =
                newSetpoint[rotations] - lastSetpoint[rotations]
            setpointVelocityRps = deltaRotations / dt
        }

        setpoint = newSetpoint
        lastSetpoint = newSetpoint
        lastTime = currentTime

        motor.setControl(
            positionVoltage
                .withPosition(setpoint)
                .withVelocity(setpointVelocityRps)
            //                .withFeedForward(7 * setpointVelocityRps)
            )
    }

    override fun periodic() {
        motor.periodic()
        Logger.recordOutput(
            "Subsystems/Turret/setpoint",
            setpoint[radians],
            radians
        )
        Logger.recordOutput(
            "Subsystems/Turret/wrappedPosition",
            wrappedPosition
        )
        Logger.recordOutput("Subsystems/Turret/turretPose", turretPose)
        Logger.recordOutput(
            "Subsystems/Turret/angleFromRobotToHub",
            angleFromRobotToHub
        )
        Logger.recordOutput(
            "Subsystems/Turret/turretAngleToHub",
            turretAngleToHub
        )
        Logger.recordOutput(
            "Subsystems/Turret/isTurretAligned",
            isTurretAligned
        )
    }
}
