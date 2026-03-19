package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.FeedingShotCalculator.calculateFeedingPitch
import frc.robot.FeedingShotCalculator.calculateFeedingVelocity
import frc.robot.FeedingShotCalculator.calculateFeedingYaw
import frc.robot.ShotCalculator.calculateAngularVelocity
import frc.robot.ShotCalculator.calculateYaw
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.constraintTurretLimits
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import kotlin.math.abs

private fun getTurretSetpoint(): Angle {
    val speeds = turretOrientedChassisSpeeds
    val constrainedWithCompensation = constraintTurretLimits(
        turretAngleToHub -
                calculateFeedingYaw(
                    compensatedTurretDistanceFromGoal[m],
                    speeds.x,
                    speeds.y
                ).deg)

    val constrainedStaticShooting = constraintTurretLimits(turretAngleToHub)
    if(abs(constrainedWithCompensation[deg] - constrainedStaticShooting[deg]) > 180){
        return constrainedStaticShooting
    }
    return constrainedWithCompensation
}

private fun getHoodSetpoint(): Angle {
    return (90.deg - calculateFeedingPitch().deg)
}

private fun getFlywheelSetpoint(): AngularVelocity {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return (1 *
            calculateAngularVelocity(
                calculateFeedingVelocity(
                    turretDistanceFromGoal[m],
                    turretOrientedChassisSpeeds.x,
                    turretOrientedChassisSpeeds.y
                )
            ))
        .rps
}

private fun getPreShooterSetpoint(): AngularVelocity =
    PreShooterVelocity.SHOOTING.velocity

val feedShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
