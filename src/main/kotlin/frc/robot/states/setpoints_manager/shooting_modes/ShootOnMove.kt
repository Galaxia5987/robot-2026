package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.lib.extensions.toRotation2d
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

fun ChassisSpeeds.to2dVector(): Translation2d =
    Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)

@LoggedOutput(path = "Odometry", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d
    get() =
        drive.chassisSpeedsSetpoint
            .to2dVector()
            .rotateBy(Turret.wrappedPosition.toRotation2d())

private fun getTurretSetpoint(): Angle {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return turretAngleToHub -
        calculateYaw( // TODO: Might not work in blue
                turretDistanceFromGoal[m],
                turretOrientedChassisSpeeds.x,
                turretOrientedChassisSpeeds.y
            )
            .deg
}

private fun getHoodSetpoint(): Angle {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return (90.deg -
        calculatePitch(
                turretDistanceFromGoal[m],
                turretOrientedChassisSpeeds.x,
                turretOrientedChassisSpeeds.y
            )
            .deg)
}

private fun getFlywheelSetpoint(): AngularVelocity {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return calculateAngularVelocity(
            calculateVelocity(
                turretDistanceFromGoal[m],
                turretOrientedChassisSpeeds.x,
                turretOrientedChassisSpeeds.y
            )
        )
        .rps
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
