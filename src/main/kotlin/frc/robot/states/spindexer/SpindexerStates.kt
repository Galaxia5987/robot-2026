package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class SpindexerStates {
    ACTIVE,
    IDLE;

    val trigger = Trigger { state == this }

    fun set(): Command = runOnce({ state = this })
}

private var state: SpindexerStates = SpindexerStates.IDLE
