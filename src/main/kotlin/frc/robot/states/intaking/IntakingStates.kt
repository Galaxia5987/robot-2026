package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger

private const val LOGGING_PATH = "StateMachines/Intaking"

enum class IntakingStates(private val bindCommand: Command) {
    CLOSED(closed()),
    INTAKING(intaking()),
    OPEN(open()),
    PUMPING(pumping());

    val trigger = Trigger { state == this }
    private val command = runOnce({
        state = this
        Logger.recordOutput("$LOGGING_PATH/state", state)
    })
        .alongWith(bindCommand)

    fun set(): Command = command
}

private var state: IntakingStates = IntakingStates.CLOSED
