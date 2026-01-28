package frc.robot.states.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.ConditionalCommand
import edu.wpi.first.wpilibj2.command.WaitUntilCommand
import frc.robot.field_constants.CLIMB_LOCATION
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState
import frc.robot.states.spindexer.SpindexerStates
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.drive.profiledAlignToPose

val overrideStates: Command = Commands.run({
    SpindexerStates.IDLE.set()
    IntakingStates.CLOSED.set()
    ShootingState.IDLE.set()
})

fun climb(): Command = Commands.sequence(
    overrideStates,
    ConditionalCommand(
        Commands.none(),
        profiledAlignToPose(CLIMB_LOCATION),
        DriverOverrides.AlignmentOverride.trigger,
    ),
    Climb.engaged()
).withName("climb")
