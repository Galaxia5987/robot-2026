package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands.runOnce

var isFeeding = false
var isIntaking = false

private fun setFeeding(value: Boolean) = runOnce({ isFeeding = value })

private fun setIntaking(value: Boolean) = runOnce({ isIntaking = value })

fun startFeeding(value: Boolean) = runOnce({ isFeeding = true })

fun stopFeeding(value: Boolean) = runOnce({ isFeeding = false })

fun startIntaking(value: Boolean) = runOnce({ isIntaking = true })

fun stopIntaking(value: Boolean) = runOnce({ isIntaking = false })

fun stop() =
    runOnce({
        isFeeding = false
        isIntaking = false
    })
