package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.ShooterState.IDLE
import frc.robot.lib.logged_output.LoggedOutputManager.runOnce
import java.util.concurrent.ConcurrentHashMap

interface ShooterStateMachine {
    val context: Trigger.() -> Unit
    val transitions: Array<out Pair<Trigger, () -> ShooterState>> //need to add vararg so don't be confused

    companion object {
        var currentState: ShooterState = IDLE
    }
}

private val triggerCache = ConcurrentHashMap<Enum<*>, Trigger>()

fun ShooterState.stateTrigger(): Trigger =
    triggerCache.getOrPut(this) {
        Trigger { this == ShooterStateMachine.currentState }
            .apply(this.context)
            .apply {
                transitions.forEach { (cond, next) ->
                    and(cond).onTrue(runOnce { next().set() })
                }
            }
    }


fun ShooterState.set(): Command = runOnce {
    ShooterStateMachine.currentState = this
}

operator fun ShooterState.invoke() = stateTrigger()


enum class ShooterState(
    override val context: Trigger.() -> Unit,
    override vararg val transitions: Pair<Trigger, () -> ShooterState>
) : ShooterStateMachine {
    IDLE(
        {},
        Trigger { true } to { NONIDLE }
    ),
    NONIDLE(
        {},
        Trigger { false } to { IDLE }
    );

}