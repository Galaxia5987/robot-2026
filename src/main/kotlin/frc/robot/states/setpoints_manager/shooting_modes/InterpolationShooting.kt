package frc.robot.states.setpoints_manager.shooting_modes

import edu.wpi.first.units.Measure
import edu.wpi.first.units.Unit
import edu.wpi.first.units.measure.Angle
import edu.wpi.first.units.measure.AngularVelocity
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.drive
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.rps
import frc.robot.lib.math.interpolation.InterpolatingDouble
import frc.robot.lib.math.interpolation.InterpolatingDoubleMap
import frc.robot.lib.shooting.ShootingTableReader
import frc.robot.states.setpoints_manager.SetpointsManager.currentGoal
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.pre_shooter.PreShooterVelocity
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(LogLevel.COMP)
val turretDistanceFromGoal: Distance
    get() = drive.pose.translation.getDistance(currentGoal.translation).m

private val SHOOTER_VELOCITY_BY_DISTANCE: InterpolatingDoubleMap =
    ShootingTableReader.parse(
        Filesystem.getDeployDirectory().path +
            "/shootData/distanceToVelocity.csv"
    )


private fun getFlywheelSetpoint(): AngularVelocity {
    val flywheelKey = InterpolatingDouble(turretDistanceFromGoal[m])
    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(flywheelKey).value.rps
}

private fun getPreShooterSetpoint(): AngularVelocity {
    return PreShooterVelocity.SHOOTING.velocity

    // Makes the preshooter velocity match the flywheel velocity
    //    val preShooterKey = InterpolatingDouble(distanceFromGoal[m])
    //    return SHOOTER_VELOCITY_BY_DISTANCE.getInterpolated(preShooterKey).value.rps
}

val interpolationShootingMap: Map<SubsystemBase, () -> Measure<out Unit>> =
    mapOf(
        Flywheel to ::getFlywheelSetpoint,
        PreShooter to ::getPreShooterSetpoint,
    )
