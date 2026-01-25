package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.logged_output.LoggedOutputManager
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class SpindexerStates {
    SPIN,
    IDLE;

    val trigger = Trigger { state == this }

    fun set() {
        LoggedOutputManager.registerField("", LogLevel.COMP, ::state,"state")
        state = this
    }

}


    private var state:SpindexerStates= SpindexerStates.IDLE


