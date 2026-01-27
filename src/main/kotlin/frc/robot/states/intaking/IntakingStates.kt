package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val LOGGING_PATH = "StateMachines/Intaking"

enum class IntakingStates {
    CLOSED,
    INTAKING,
    OPEN,
    PUMPING;

    val trigger = Trigger { state == this }

    fun set(): Command = runOnce({
        state = this
        Logger.recordOutput("$LOGGING_PATH/state", state)
    })
}

private var state: IntakingStates = IntakingStates.CLOSED
