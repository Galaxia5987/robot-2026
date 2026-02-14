package frc.robot.states.climb

import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.field.CLIMB_TRANSLATION
import frc.robot.states.DriverOverrides
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.shooting.ShootingState
import frc.robot.states.spindexer.SpindexerStates
import frc.robot.subsystems.climb.Climb
import frc.robot.subsystems.drive.profiledAlignToPose

val overrideStates: Command =
    Commands.run({
        SpindexerStates.IDLE.set()
        IntakingStates.CLOSED.set()
        ShootingState.IDLE.set()
    })

fun climb(climbLocation: Pose2d = CLIMB_TRANSLATION): Command =
    Commands.sequence(
            overrideStates,
            profiledAlignToPose(climbLocation)
                .unless(DriverOverrides.AlignmentOverride.trigger),
            Climb.engaged()
        )
        .withName("climb")
