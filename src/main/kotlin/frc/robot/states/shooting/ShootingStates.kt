package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class ShootingState {
    IDLE,
    PRIMING,
    BACKFEEDING,
    SHOOTING;

    val trigger = Trigger { currentState == this }

    fun set(): Command = Commands.runOnce({ currentState = this })
}

@LoggedOutput(LogLevel.COMP, "shootingState")
var currentState: ShootingState = ShootingState.IDLE

val isIdle = ShootingState.IDLE.trigger
val isPriming = ShootingState.PRIMING.trigger
val isBackfeeding = ShootingState.BACKFEEDING.trigger
val isShooting = ShootingState.SHOOTING.trigger

fun bindStateMachine() {
    bindStateTriggers()
    bindShootingTriggers()
}

fun bindStateTriggers() {
    isIdle.onTrue(idle())
    isPriming.onTrue(priming())
    isBackfeeding.onTrue(backfeeding())
    isShooting.onTrue(shooting())
}

fun setIdle(): Command = ShootingState.IDLE.set()

fun setPriming(): Command = ShootingState.PRIMING.set()

fun setBackfeeding(): Command = ShootingState.BACKFEEDING.set()

fun setShooting(): Command = ShootingState.SHOOTING.set()
