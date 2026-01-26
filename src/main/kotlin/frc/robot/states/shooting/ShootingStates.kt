package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.logged_output.LoggedOutputManager
import org.team5987.annotation.LogLevel

enum class ShootingState(commandBind: Command) {
    IDLE(idle()),
    PRIMING(priming()),
    BACKFEEDING(backfeeding()),
    SHOOTING(shooting());

    val trigger = Trigger { state == this }

    init {
        trigger.onTrue(commandBind)
    }

    fun set(): Command = Commands.runOnce({
        LoggedOutputManager.registerField("", LogLevel.COMP, ::state, "state")
        state = this
    })
}

private var state: ShootingState = ShootingState.IDLE