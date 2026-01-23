package frc.robot.states.Climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import frc.robot.drive
import frc.robot.field_constants.CLIMB_LOCATION
import frc.robot.lib.extensions.distanceFromPoint
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toPose3d
import frc.robot.spindexer
import frc.robot.states.ExternalControl
import frc.robot.subsystems.drive.profiledAlignToPose


fun climb(): Command = Commands.runOnce(
    { SpindexerState.IDLE.set }
).until(spindexer.isAtSetpoint)
    .andThen(
    ConditionalCommand(
        Commands.none(),
        profiledAlignToPose(CLIMB_LOCATION.toPose()),
        ExternalControl.AlignmentOverride.trigger,
    ).andThen(
        ConditionalCommand(

        )
    )
        .andThen()
)