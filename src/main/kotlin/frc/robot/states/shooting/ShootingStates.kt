package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

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
        state = this
        Logger.recordOutput("StateMachines/Shooting/state", state)
    })
}

var state: ShootingState = ShootingState.IDLE
