package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field_constants.ALLIANCE_ZONE
import frc.robot.isOurHubActive
import frc.robot.lib.LoggedNetworkTrigger
import frc.robot.states.sensors.hasFuel
import frc.robot.states.sensors.isPreshooterUnloaded
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret

var dontShoot = false

val isInTeamZone
    get() = ALLIANCE_ZONE.contains(drive.pose.translation)

val canShoot = Trigger { isOurHubActive && isInTeamZone && !dontShoot }

val atGoal =
    Hood.atSetpoint
        .and(Turret.atSetpoint)
        .and(Flywheel.atSetpoint)
        .and(PreShooter.atSetpoint)

val robotEmpty = LoggedNetworkTrigger("/Tuning/robotEmpty", hasFuel.negate())

fun bindShootingTriggers() {
    canShoot.negate().onTrue(setIdle())

    isIdle.and(canShoot).onTrue(setPriming())
    isPriming.and(atGoal).onTrue(setShooting())
    isShooting.and(atGoal.negate()).onTrue(setBackfeeding())
    isBackfeeding.and(isPreshooterUnloaded).onTrue(setPriming())
    isShooting.and(robotEmpty).onTrue(setIdle())
}
