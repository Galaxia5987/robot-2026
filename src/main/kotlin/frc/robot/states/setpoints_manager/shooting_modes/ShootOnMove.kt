package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.*
import frc.robot.ShotCalculator.calculateAngularVelocity
import frc.robot.ShotCalculator.calculatePitch
import frc.robot.ShotCalculator.calculateVelocity
import frc.robot.ShotCalculator.calculateYaw
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
import frc.robot.subsystems.shooter.turret.constraintTurretLimits
import frc.robot.subsystems.shooter.turret.getTurretTangentialVelocityFieldRelative
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import kotlin.math.tanh
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput
import kotlin.math.abs

private fun ChassisSpeeds.to2dVector(): Translation2d =
    Translation2d(this.vxMetersPerSecond, this.vyMetersPerSecond)

@LoggedOutput(path = "Odometry", level = LogLevel.COMP)
val turretOrientedChassisSpeeds: Translation2d
    get() {
        val speeds = drive.chassisSpeeds
        return speeds
            .to2dVector().plus(getTurretTangentialVelocityFieldRelative(speeds.omegaRadiansPerSecond))
            .rotateBy(Turret.position.toRotation2d())
    }


private fun getTurretSetpoint(): Angle {
    val speeds = turretOrientedChassisSpeeds
    val constrainedWithCompensation = constraintTurretLimits(
        turretAngleToHub -
                calculateYaw(
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
