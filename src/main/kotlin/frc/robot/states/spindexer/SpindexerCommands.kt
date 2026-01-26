package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.spindexer.SpindexerVelocity

var isFeeding = false
var isIntaking = false

private fun setFeeding(value: Boolean) = runOnce({ isFeeding = value })

private fun setIntaking(value: Boolean) = runOnce({ isIntaking = value })

fun startFeeding (value: Boolean)= runOnce({isFeeding=true})
fun stopFeeding (value: Boolean)= runOnce({isFeeding=false})

fun startIntaking (value: Boolean)= runOnce({isIntaking=true})
fun stopIntaking (value: Boolean)= runOnce({isIntaking=false})

private fun idle(): Command = Spindexer.setTarget(SpindexerVelocity.STOP)

private fun spin(): Command = Spindexer.setTarget(SpindexerVelocity.START)

fun stop() =
    runOnce({
        isFeeding = false
        isIntaking = false
    })
