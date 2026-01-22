package frc.robot.lib

import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team5987.annotation.machine_states.MachineStates


@MachineStates
enum class ShooterState(
    override val context: Trigger.() -> Unit,
    override vararg val transitions: Pair<Trigger, () -> ShooterState>
) : ShooterStateMachine {
    IDLE({}, Trigger { true } to { NONIDLE }),
    NONIDLE({}, Trigger { false } to { IDLE })
}

fun initStateMachine() {
    ShooterState.entries.forEach {
        it.stateTrigger()
    }
}