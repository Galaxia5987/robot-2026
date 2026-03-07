package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.calculateAngularVelocity
import frc.robot.calculatePitch
import frc.robot.calculateVelocity
import frc.robot.drive
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import frc.robot.lib.shooting.ShootingTableReader
import frc.robot.states.setpoints_manager.SetpointsManager.currentGoal
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.turretAimingSetpoint
import frc.robot.subsystems.shooter.turret.turretTranslationFieldOriented
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(LogLevel.COMP)
val turretDistanceFromGoal: Distance
    get() =
        turretTranslationFieldOriented.getDistance(currentGoal.translation).m

private val SHOOTER_VELOCITY_BY_DISTANCE: InterpolatingDoubleMap =
    ShootingTableReader.parse(
        Filesystem.getDeployDirectory().path +
            "/shootData/distanceToVelocity.csv"
    )

private fun getTurretSetpoint(): Angle {
    return turretAimingSetpoint
}

private fun getHoodSetpoint(): Angle =
    (90.deg -
        calculatePitch(
                turretDistanceFromGoal[m],
                drive.chassisSpeeds.vxMetersPerSecond,
                drive.chassisSpeeds.vyMetersPerSecond
            )
            .deg)

private fun getFlywheelSetpoint(): AngularVelocity {
    val turretOrientedChassisSpeeds = turretOrientedChassisSpeeds
    return (1 * calculateAngularVelocity(
        calculateVelocity(
            turretDistanceFromGoal[m],
            turretOrientedChassisSpeeds.x,
            turretOrientedChassisSpeeds.y
        )
    ))
        .rps
//    val flywheelKey = InterpolatingDouble(turretDistanceFromGoal[m])
//    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(flywheelKey).value.rps
}

private fun getPreShooterSetpoint(): AngularVelocity {
    return PreShooterVelocity.SHOOTING.velocity

    // Makes the preshooter velocity match the flywheel velocity
    //    val preShooterKey = InterpolatingDouble(distanceFromGoal[m])
    //    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(preShooterKey).value.rps
}

val interpolationShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
