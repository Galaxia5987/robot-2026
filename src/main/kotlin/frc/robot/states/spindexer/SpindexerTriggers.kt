package frc.robot.states.spindexer

import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.spindexer
import frc.robot.subsystems.spindexer.ConveyorVelocity
import frc.robot.subsystems.spindexer.Spindexer
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

val spinState = spindexer.start()
val idleState = Spindexer.stop()

@LoggedOutput(LogLevel.COMP)
val spinRequested =
    Trigger { isFeeding || isIntaking }
        .onTrue(Commands.runOnce({ SpindexerStates.SPIN.set() }))
        .onFalse(Commands.runOnce({ SpindexerStates.IDLE.set() }))
