package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands.runOnce

var isFeeding = false

var isIntaking = false

fun setFeeding(value: Boolean) = runOnce({ isFeeding = value })

fun setIntaking(value: Boolean) = runOnce({ isIntaking = value })

fun stop() =
    runOnce({
        isFeeding = false
        isIntaking = false
    })
