package frc.robot.states.setpoints_manager.setpoints_state

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.drive
import frc.robot.lib.extensions.degrees
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import frc.robot.lib.shooting.ShootingTableReader
import frc.robot.states.setpoints_manager.currentGoal
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(LogLevel.COMP)
val distanceFromGoal: Distance
    get() = drive.pose.distanceFromPoint(currentGoal)

private val SHOOTER_VELOCITY_BY_DISTANCE: InterpolatingDoubleMap =
    ShootingTableReader.parse(
        Filesystem.getDeployDirectory().path +
            "/shootData/distanceToVelocity.csv"
    )

private val SHOOTER_ANGLE_BY_DISTANCE: InterpolatingDoubleMap =
    ShootingTableReader.parse(
        Filesystem.getDeployDirectory().path + "/shootData/distanceToAngle.csv"
    )

private fun getTurretSetpoint(): Angle {
    return turretAngleToHub
}

private fun getHoodSetpoint(): Angle {
    val hoodKey = InterpolatingDouble(distanceFromGoal[m])
    return SHOOTER_ANGLE_BY_DISTANCE.getInterpolated(hoodKey).value.degrees
}

private fun getFlywheelSetpoint(): AngularVelocity {
    val flywheelKey = InterpolatingDouble(distanceFromGoal[m])
    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(flywheelKey).value.rps
}

private fun getPreShooterSetpoint(): AngularVelocity {
    val preShooterKey = InterpolatingDouble(distanceFromGoal[m])
    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(preShooterKey).value.rps
}

val interpolationShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Turret to ::getTurretSetpoint,
        Hood to ::getHoodSetpoint,
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
