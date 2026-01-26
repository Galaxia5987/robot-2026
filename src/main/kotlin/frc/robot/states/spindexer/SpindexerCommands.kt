package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands.runOnce

var isFeeding = false
var isIntaking = false

private fun setFeeding(value: Boolean) = runOnce({ isFeeding = value })

private fun setIntaking(value: Boolean) = runOnce({ isIntaking = value })

fun startFeeding() = runOnce({ isFeeding = true })

fun stopFeeding() = runOnce({ isFeeding = false })

fun startIntaking() = runOnce({ isIntaking = true })

fun stopIntaking() = runOnce({ isIntaking = false })

fun stop() =
    runOnce({
        isFeeding = false
        isIntaking = false
    })
