package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands.runOnce
import frc.robot.subsystems.shooter.turret.ConveyorVelocity
import frc.robot.subsystems.spindexer.Spindexer
import frc.robot.subsystems.spindexer.SpindexerVelocity

//var isFeeding = false
//var isIntaking = false
//private fun startFeeding(value: Boolean) = runOnce({ isFeeding = value })
//private fun startIntaking(value: Boolean) = runOnce({ isIntaking = value })

private fun ideal (): Command= Spindexer.setTarget(SpindexerVelocity.STOP)

private fun spin (): Command= Spindexer.setTarget(SpindexerVelocity.START)


//fun stop() =
//    runOnce({
//        isFeeding = false
//        isIntaking = false
//    })
