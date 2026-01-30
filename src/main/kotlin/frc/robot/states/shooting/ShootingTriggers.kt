package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.inAllianceZone
import frc.robot.field.isHubActive
import frc.robot.lib.extensions.logTrigger
import frc.robot.states.setpoints_manager.isShootingOnMove
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.Turret
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val isIdle = ShootingState.IDLE.trigger.onTrue(idle())
private val isPriming = ShootingState.PRIMING.trigger.onTrue(priming())
private val isBackfeeding =
    ShootingState.BACKFEEDING.trigger.onTrue(backfeeding())
private val isShooting = ShootingState.SHOOTING.trigger.onTrue(shooting())

@LoggedOutput(LogLevel.COMP)
val shooterAtSetpoint =
    Hood.atSetpoint
        .and(Turret.atSetpoint)
        .and(Flywheel.atSetpoint)
        .and(PreShooter.atSetpoint)

class Shooting(dontShootTrigger: Trigger) {
    private val canShoot =
        isHubActive
            .and(dontShootTrigger.negate())
            .and(inAllianceZone)
            .logTrigger("StateMachines/Shooting/canShoot")

    private val cantShoot = canShoot.negate().onTrue(ShootingState.IDLE.set())

    private val idleAndCanShoot =
        ShootingState.IDLE.trigger
            .and(canShoot)
            .onTrue(ShootingState.PRIMING.set())

    private val lockIfNeeded =
        ShootingState.PRIMING.trigger.onTrue(
            drive.lock().onlyIf(isShootingOnMove)
        )

    private val setShootingIfPrimed =
        ShootingState.PRIMING.trigger
            .and(shooterAtSetpoint)
            .onTrue(ShootingState.SHOOTING.set())

    private val setBackfeedingIfNotAtSetpoint =
        ShootingState.SHOOTING.trigger
            .and(shooterAtSetpoint.negate())
            .onTrue(ShootingState.BACKFEEDING.set())

    private val setPrimingIfHasFuel =
        ShootingState.BACKFEEDING.trigger
            .and(Sensors.hasFuel)
            .onTrue(ShootingState.PRIMING.set())

    private val setIdleIfHasNoFuel =
        ShootingState.SHOOTING.trigger
            .and(Sensors.hasFuel.negate())
            .onTrue(ShootingState.IDLE.set())
}
