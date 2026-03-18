package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.BuiltInAccelerometer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.LOOP_TIME
import frc.robot.ShotCalculator.*
import frc.robot.drive
import frc.robot.lib.AccelerationFilter
import frc.robot.lib.AccelerationKalmanFusion
import frc.robot.lib.extensions.*
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.constraintTurretLimits
import frc.robot.subsystems.shooter.turret.getTurretTangentialVelocityFieldRelative
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.math.tanh

private const val LATENCY_FACTOR = 0.2
private val RIO_TO_ORIGIN = Transform2d()

private val accelFilter: AccelerationFilter = AccelerationKalmanFusion(RIO_TO_ORIGIN)

private var lastVelocityX: Double = 0.0
private var lastVelocityY: Double = 0.0
private var lastOmega: Double = 0.0
private val rioAccel = BuiltInAccelerometer()

private fun ChassisSpeeds.to2dVector(): Translation2d =
    Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)

@LoggedOutput(path = "Odometry", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d
    get() {
        val speeds = drive.chassisSpeeds
        val omega = drive.gyroOmega[rad_ps]
        val alpha = (omega - lastOmega) / LOOP_TIME // TODO: put this in a low pass filter
        accelFilter.update(
            (speeds.vxMetersPerSecond - lastVelocityX) / LOOP_TIME,
            (speeds.vyMetersPerSecond - lastVelocityY) / LOOP_TIME,
            rioAccel.x,
            rioAccel.y,
            drive.accelerationX[mps_ps],
            drive.accelerationY[mps_ps],
            omega,
            alpha
        )
        lastVelocityX = speeds.vxMetersPerSecond
        lastVelocityY = speeds.vyMetersPerSecond
        lastOmega = omega
        return speeds
            .to2dVector().plus(getTurretTangentialVelocityFieldRelative(omega + alpha * LATENCY_FACTOR))
            .plus(
                Translation2d(accelFilter.estimatedAccelerationX, accelFilter.estimatedAccelerationY).times(
                    LATENCY_FACTOR
                )
            )
            .rotateBy(Turret.position.toRotation2d())
    }


private fun getTurretSetpoint(): Angle {
    val speeds = turretOrientedChassisSpeeds
    return constraintTurretLimits(
        turretAngleToHub -
                calculateYaw(
                    compensatedTurretDistanceFromGoal[m],
                    speeds.x,
                    speeds.y
                )
                    .deg
    )
}

private fun getHoodSetpoint(): Angle {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return (90.deg -
            calculatePitch(
                compensatedTurretDistanceFromGoal[m],
                turretOrientedChassisSpeeds.x,
                turretOrientedChassisSpeeds.y
            )
                .deg)
}

private fun getFlywheelSetpoint(): AngularVelocity {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    var output = ((0.97 - (0.05 * tanh(turretOrientedChassisSpeeds.norm))) *
            calculateAngularVelocity(
                calculateVelocity(
                    compensatedTurretDistanceFromGoal[m],
                    turretOrientedChassisSpeeds.x,
                    turretOrientedChassisSpeeds.y
                )
            ))
    return output.rps
}

private fun getPreShooterSetpoint(): AngularVelocity =
    PreShooterVelocity.SHOOTING.velocity

val shootOnMoveMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
