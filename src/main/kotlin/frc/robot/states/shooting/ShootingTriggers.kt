package frc.robot.states.shooting

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.inAllianceZone
import frc.robot.field.isHubActive
import frc.robot.isAuto
import frc.robot.isEnabled
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.logTrigger
import frc.robot.lib.extensions.not
import frc.robot.lib.extensions.sec
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.subsystems.intake.funnel.Funnel
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private const val LOGGING_PATH = "StateMachines/Shooting"
private val allSubsystemsAtSetpointDebounce = 0.1.sec

private val isIdle = ShootingState.IDLE.trigger.onTrue(idle())
private val isPriming = ShootingState.PRIMING.trigger.onTrue(priming())
private val isBackfeeding =
    ShootingState.BACKFEEDING.trigger.onTrue(backfeeding())
private val isShooting = ShootingState.SHOOTING.trigger.onTrue(shooting())

@LoggedOutput(path = LOGGING_PATH, level = LogLevel.COMP)
val allSubsystemsAtSetpoint: Trigger =
    Hood.atSetpoint
        .and(Turret.atSetpoint)
        .and(Flywheel.atSetpoint)
        //        .and(PreShooter.atSetpoint)
        .debounce(
            allSubsystemsAtSetpointDebounce[sec],
            Debouncer.DebounceType.kRising
        )
        .logTrigger("$LOGGING_PATH/allSubsystemsAtSetpoint")

var commandShootOnAuto = false

fun setShootInAuto(): Command = runOnce({ commandShootOnAuto = true })

fun stopShootInAuto(): Command = runOnce({ commandShootOnAuto = false })

class Shooting(val dontShootTrigger: Trigger, canFeedTrigger: Trigger) {

    val isAutoCurrentlyShooting: Trigger =
        Trigger { commandShootOnAuto }.or(isAuto.negate())

    private val canShootToHub =
        isHubActive
            .and(dontShootTrigger.negate())
            .and(inAllianceZone)
            .and(Sensors.hasFuel)
            .and(isEnabled)
            .and(IntakingStates.INTAKING.trigger.negate().or(isShootingOnMove))
            .and(isAutoCurrentlyShooting)
            .logTrigger("$LOGGING_PATH/canShootToHub")

    val cantShootToHub = canShootToHub.negate().onTrue(ShootingState.IDLE.set())

    private val canFeed =
        canFeedTrigger
            .and(inAllianceZone.negate())
            .and(Turret.atSetpoint)
            .and(isEnabled)

    private val idleAndCanShootToHub =
        ShootingState.IDLE.trigger
            .and(canShootToHub)
            .onTrue(ShootingState.PRIMING.set())
            .logTrigger("$LOGGING_PATH/idleAndCanShootToHub")

    private val shootingStatePrimingOrShooting =
        ShootingState.PRIMING.trigger.or(ShootingState.SHOOTING.trigger)

    private val lockIfNeeded =
        shootingStatePrimingOrShooting
            .and(Turret.atSetpoint)
            .and(isShootingOnMove.negate())
            .and(inAllianceZone)
            .and(isAuto.negate())
            .whileTrue(drive.continousLock())
            .logTrigger("$LOGGING_PATH/lockIfNeeded")

    private val setShootingIfCanFeed =
        ShootingState.IDLE.trigger
            .and(canFeed)
            .onTrue(ShootingState.SHOOTING.set())
            .logTrigger("$LOGGING_PATH/setShootingIfCanFeed")

    private val setShootingIfPrimed =
        ShootingState.PRIMING.trigger
            .and(allSubsystemsAtSetpoint)
            .and(canShootToHub)
            .onTrue(ShootingState.SHOOTING.set())
            .logTrigger("$LOGGING_PATH/setShootingIfPrimed")

    //    private val setBackfeedingIfNotAtSetpoint =
    //        ShootingState.SHOOTING.trigger]\[

    //            .and(shooterAtSetpoint.negate())
    //            .onTrue(ShootingState.BACKFEEDING.set())
    //            .logTrigger("$LOGGING_PATH/setBackfeedingIfNotAtSetpoint")

    //    private val setPrimingIfHasFuel =
    //        ShootingState.BACKFEEDING.trigger
    //            .and(Sensors.hasFuel)
    //            .onTrue(ShootingState.PRIMING.set())
    //            .logTrigger("$LOGGING_PATH/setPrimingIfHasFuel")

    private val shouldShootingToHubStop =
        (Sensors.hasFuel
                .negate()
                .or(
                    IntakingStates.INTAKING.trigger.and(
                        isShootingOnMove.negate()
                    )
                )
                .or(Turret.atSetpoint.negate()))
            .and(inAllianceZone)

    private val shouldStopFeeding =
        (canFeedTrigger.negate().or(Turret.atSetpoint.negate())).and(
            !inAllianceZone
        )

    private val setIdleIfShouldStopShooting =
        ShootingState.SHOOTING.trigger
            .and(shouldShootingToHubStop.or(shouldStopFeeding))
            .onTrue(ShootingState.IDLE.set())
            .logTrigger("$LOGGING_PATH/setIdleIfShouldStopShooting")

    private val setFunnel =
        (ShootingState.IDLE.trigger
                .negate()
                .or(IntakingStates.INTAKING.trigger))
            .and(isEnabled)
            .onTrue(Funnel.start())
            .onFalse(Funnel.stop())
}
