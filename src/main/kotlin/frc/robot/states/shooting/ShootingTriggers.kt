package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.drive
import frc.robot.field.inAllianceZone
import frc.robot.field.isHubActive
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.logTrigger
import frc.robot.lib.extensions.sec
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
import frc.robot.subsystems.sensors.Sensors
import frc.robot.subsystems.shooter.flywheel.Flywheel
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.pre_shooter.PreShooter
import frc.robot.subsystems.shooter.turret.isTurretAligned
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private const val LOGGING_PATH = "StateMachines/Shooting"
private val allSubsystemsAtSetpointDebounce = 0.2.sec

private val isIdle = ShootingState.IDLE.trigger.onTrue(idle())
private val isPriming = ShootingState.PRIMING.trigger.onTrue(priming())
private val isBackfeeding =
    ShootingState.BACKFEEDING.trigger.onTrue(backfeeding())
private val isShooting = ShootingState.SHOOTING.trigger.onTrue(shooting())

@LoggedOutput(path = LOGGING_PATH, level = LogLevel.COMP)
val allSubsystemsAtSetpoint: Trigger =
    Hood.atSetpoint
        .and(isTurretAligned)
        .and(Flywheel.atSetpoint)
        .and(PreShooter.atSetpoint)
        .debounce(allSubsystemsAtSetpointDebounce[sec])
        .logTrigger("$LOGGING_PATH/allSubsystemsAtSetpoint")

class Shooting(dontShootTrigger: Trigger) {
    private val canShoot =
        isHubActive
            .and(dontShootTrigger.negate())
            .and(inAllianceZone)
            .and(Sensors.hasFuel)
            .and(IntakingStates.INTAKING.trigger.negate())
            .logTrigger("$LOGGING_PATH/canShoot")

    private val cantShoot = canShoot.negate().onTrue(ShootingState.IDLE.set())

    private val idleAndCanShoot =
        ShootingState.IDLE.trigger
            .and(canShoot)
            .onTrue(ShootingState.PRIMING.set())
            .logTrigger("$LOGGING_PATH/idleAndCanShoot")

    private val shootingStatePrimingOrShooting =
        ShootingState.PRIMING.trigger.or(ShootingState.SHOOTING.trigger)

    private val lockIfNeeded =
        shootingStatePrimingOrShooting
            .and(isTurretAligned)
            .and(isShootingOnMove.negate())
            .whileTrue(drive.continousLock())
            .logTrigger("$LOGGING_PATH/lockIfNeeded")

    private val setShootingIfPrimed =
        ShootingState.PRIMING.trigger
            .and(allSubsystemsAtSetpoint)
            .and(canShoot)
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

    private val shouldShootingStop =
        Sensors.hasFuel.negate().or(IntakingStates.INTAKING.trigger)

    private val setIdleIfShouldStopShooting =
        ShootingState.SHOOTING.trigger
            .and(shouldShootingStop)
            .onTrue(ShootingState.IDLE.set())
            .logTrigger("$LOGGING_PATH/setIdleIfHasNoFuel")
}
