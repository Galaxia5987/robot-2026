package frc.robot.states.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.field_constants.CLIMB_LOCATION
import frc.robot.lib.extensions.toPose
import frc.robot.spindexer
import frc.robot.states.ExternalControl
import frc.robot.subsystems.drive.profiledAlignToPose

val overrideStates: Command = Commands.run({
    SpindexerStates.IDLE.set()
    IntakeState.Closed.set()
    ShooterState.IDLE.set()
})

fun climb(): Command = Commands.sequence(
    overrideStates,
    WaitUntilCommand(spindexer.isAtSetpoint),
    ConditionalCommand(
        Commands.none(),
        profiledAlignToPose(CLIMB_LOCATION.toPose()),
        ExternalControl.AlignmentOverride.trigger,
    ),
    ConditionalCommand(Climb.ENGAGED())
).withName("climb")
