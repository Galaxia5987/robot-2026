package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.button.Trigger

enum class SpindexerStates {
    SPIN,
    IDLE;

    val trigger = Trigger { state == this }

    fun set() {
        state = this
    }
}

private var state = SpindexerStates.IDLE

fun robotContainer() {
    SpindexerStates.SPIN.set()
}
