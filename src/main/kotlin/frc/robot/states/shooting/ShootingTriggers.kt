package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.inAllianceZone
import frc.robot.field.isHubActive
import frc.robot.isEnabled
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.logTrigger
import frc.robot.lib.extensions.sec
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.subsystems.sensors.Sensors

private const val LOGGING_PATH = "StateMachines/Shooting"
private val allSubsystemsAtSetpointDebounce = 0.2.sec

private val isIdle = ShootingState.IDLE.trigger.onTrue(idle())
private val isPriming = ShootingState.PRIMING.trigger.onTrue(priming())
private val isBackfeeding =
    ShootingState.BACKFEEDING.trigger.onTrue(backfeeding())
private val isShooting = ShootingState.SHOOTING.trigger.onTrue(shooting())

class Shooting(dontShootTrigger: Trigger, canFeedTrigger: Trigger) {
    private val canShootToHub =
        isHubActive
            .and(dontShootTrigger.negate())
            .and(inAllianceZone)
            .and(Sensors.hasFuel)
            .and(isEnabled)
            .logTrigger("$LOGGING_PATH/canShootToHub")

    private val cantShootToHub =
        canShootToHub.negate().onTrue(ShootingState.IDLE.set())

    private val canFeed =
        canFeedTrigger.and(inAllianceZone.negate()).and(isEnabled)

    private val idleAndCanShootToHub =
        ShootingState.IDLE.trigger
            .and(canShootToHub)
            .onTrue(ShootingState.PRIMING.set())
            .logTrigger("$LOGGING_PATH/idleAndCanShootToHub")

    private val shootingStatePrimingOrShooting =
        ShootingState.PRIMING.trigger.or(ShootingState.SHOOTING.trigger)

    private val lockIfNeeded =
        shootingStatePrimingOrShooting
            .and(isShootingOnMove.negate())
            .and(inAllianceZone)
            .whileTrue(drive.continousLock())
            .logTrigger("$LOGGING_PATH/lockIfNeeded")

    private val setShootingIfCanFeed =
        ShootingState.IDLE.trigger
            .and(canFeedTrigger)
            .onTrue(ShootingState.SHOOTING.set())
            .logTrigger("$LOGGING_PATH/setShootingIfCanFeed")

    private val setShootingIfPrimed =
        ShootingState.PRIMING.trigger
            .and(canShootToHub)
            .onTrue(ShootingState.SHOOTING.set())
            .logTrigger("$LOGGING_PATH/setShootingIfPrimed")

    //    private val setBackfeedingIfNotAtSetpoint =
    //        ShootingState.SHOOTING.trigger
    //            .and(shooterAtSetpoint.negate())
    //            .onTrue(ShootingState.BACKFEEDING.set())
    //            .logTrigger("$LOGGING_PATH/setBackfeedingIfNotAtSetpoint")

    //    private val setPrimingIfHasFuel =
    //        ShootingState.BACKFEEDING.trigger
    //            .and(Sensors.hasFuel)
    //            .onTrue(ShootingState.PRIMING.set())
    //            .logTrigger("$LOGGING_PATH/setPrimingIfHasFuel")

    private val shouldShootingToHubStop =
        Sensors.hasFuel.negate().and(inAllianceZone)

    private val shouldStopFeeding =
        canFeedTrigger.negate().and(inAllianceZone.negate())

    private val setIdleIfShouldStopShooting =
        ShootingState.SHOOTING.trigger
            .and(shouldShootingToHubStop.or(shouldStopFeeding))
            .onTrue(ShootingState.IDLE.set())
            .logTrigger("$LOGGING_PATH/setIdleIfShouldStopShooting")
}
