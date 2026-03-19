package frc.robot.states.shooting

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import org.littletonrobotics.junction.Logger

enum class ShootingState {
    IDLE,
    PRIMING,
    BACKFEEDING,
    SHOOTING;

    val trigger = Trigger { state == this }

    fun set(): Command =
        Commands.runOnce({
            state = this
            //            println("STATE IS NOW $state!!!")
            Logger.recordOutput("StateMachines/Shooting/state", state)
        })
}

var state: ShootingState = ShootingState.IDLE
