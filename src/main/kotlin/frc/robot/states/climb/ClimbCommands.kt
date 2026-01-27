package frc.robot.states.climb

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.field_constants.CLIMB_LOCATION
import frc.robot.lib.extensions.toPose
import frc.robot.states.DriverOverrides
import frc.robot.subsystems.drive.profiledAlignToPose
import frc.robot.subsystems.spindexer.Spindexer

fun climb(): Command =
    Commands.runOnce({ Spindexer.stop() })
        .until(Spindexer.isAtSetpoint)
        .andThen(
            profiledAlignToPose(CLIMB_LOCATION.toPose())
                .onlyIf(DriverOverrides.AlignmentOverride.trigger.negate())
        )
