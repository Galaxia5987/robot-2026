package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.ShooterState.IDLE

interface ShooterStateMachine {
    val context: Trigger.() -> Unit
    val transitions: Array<out Pair<Trigger, () -> ShooterState>> //need to add vararg so don't be confused

    companion object {
        var currentState: ShooterState = IDLE
    }
}

fun ShooterState.set(): Command = runOnce({
    ShooterStateMachine.currentState = this
})

operator fun ShooterState.invoke() = stateTrigger



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

    val stateTrigger = Trigger { this == ShooterStateMachine.currentState }
        .apply(context)
        .apply {
            transitions.forEach { and(it.first).onTrue(it.second().set()) }
        }
}