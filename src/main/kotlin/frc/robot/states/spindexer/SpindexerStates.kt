package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class SpindexerStates {
    SPIN,
    IDLE;

    val trigger = Trigger { state == this }

    fun set() {
        state = this
    }
}

@LoggedOutput(LogLevel.COMP)private var state = SpindexerStates.IDLE

