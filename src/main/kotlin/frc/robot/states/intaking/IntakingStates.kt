package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class IntakingStates {
    CLOSED,
    INTAKING,
    OPEN,
    PUMPING;

    val trigger = Trigger { state == this }

    fun set(): Command = runOnce({ state = this })
}

@LoggedOutput(LogLevel.COMP) var state: IntakingStates = IntakingStates.CLOSED
