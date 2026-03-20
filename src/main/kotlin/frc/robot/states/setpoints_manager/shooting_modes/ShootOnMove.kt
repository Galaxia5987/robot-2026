package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.math.filter.LinearFilter
import edu.wpi.first.math.geometry.Rotation2d
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
import frc.robot.field.HUB_TRANSLATION
import frc.robot.lib.AccelerationComplementaryFilter
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
import frc.robot.subsystems.shooter.turret.turretTranslationFieldOriented
import org.dyn4j.geometry.Rotation
import org.littletonrobotics.junction.Logger
import kotlin.math.abs
import kotlin.math.tanh
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.math.tanh
import kotlin.math.withSign

private const val LATENCY_FACTOR = 0.2
private val RIO_TO_ORIGIN = Transform2d((-93.39010).mm, 243.92419.mm, Rotation2d())

private val accelFilter: AccelerationFilter = AccelerationComplementaryFilter(RIO_TO_ORIGIN)

private var lastVelocityX: Double = 0.0
private var lastVelocityY: Double = 0.0
private var lastOmega: Double = 0.0
private val rioAccel = BuiltInAccelerometer()

private val rioFilterX = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val rioFilterY = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val pigeonFilterX = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val pigeonFilterY = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val swerveFilterX = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val swerveFilterY = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)
private val alphaFilter = LinearFilter.singlePoleIIR(1 / 5.0, LOOP_TIME)

private fun ChassisSpeeds.to2dVector(): Translation2d =
    Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)

@LoggedOutput(path = "Odometry", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d
    get() {
        val speeds = drive.chassisSpeeds
        return speeds
            .to2dVector()
            .plus(
                getTurretTangentialVelocityFieldRelative(
                    speeds.omegaRadiansPerSecond
                )
            )
            .rotateBy(Turret.position.toRotation2d())
    }

private fun getTurretSetpoint(): Angle {
    val speeds = turretOrientedChassisSpeeds
    val constrainedWithCompensation =
        constraintTurretLimits(
            turretAngleToHub -
                    calculateYaw(
                        compensatedTurretDistanceFromGoal[m],
                        speeds.x,
                        speeds.y
                    )
                        .deg
        )

    val constrainedStaticShooting = constraintTurretLimits(turretAngleToHub)
    if (
        abs(constrainedWithCompensation[deg] - constrainedStaticShooting[deg]) >
        180
    ) {
        return constrainedStaticShooting
    }
    return constrainedWithCompensation
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
    var output =
        ((0.97 + (0.05 * tanh(turretOrientedChassisSpeeds.x))) *
                calculateAngularVelocity(
                    calculateVelocity(
                        compensatedTurretDistanceFromGoal[m],
                        turretOrientedChassisSpeeds.x,
                        turretOrientedChassisSpeeds.y
                    )
                ))

    output *= 1 + 3.0 / 89.0 * Math.exp(1.3 * (compensatedTurretDistanceFromGoal[m] - 4.6))

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
