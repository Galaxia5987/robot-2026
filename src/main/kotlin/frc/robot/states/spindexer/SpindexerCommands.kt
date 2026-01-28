package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce

object SpindexerCommands {
    internal var isFeeding = false
    internal var isIntaking = false

    fun startFeeding(): Command = runOnce({ isFeeding = true })

    fun stopFeeding(): Command = runOnce({ isFeeding = false })

    fun startIntaking(): Command = runOnce({ isIntaking = true })

    fun stopIntaking(): Command = runOnce({ isIntaking = false })

    fun stop() =
        runOnce({
            isFeeding = false
            isIntaking = false
        })
}
