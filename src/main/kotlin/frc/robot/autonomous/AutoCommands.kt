package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.RobotContainer.autoFactory

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well

fun StartToFuelDepotSide(): Command =
    Commands.sequence(
        autoFactory().resetOdometry("StartToFuelDepotSide"),
        autoFactory().trajectoryCmd("StartToFuelDepotSide"),
    )

fun StartSomethingNew(): Command =
    Commands.sequence(
        autoFactory().resetOdometry("StartSomethingNew"),
        autoFactory().trajectoryCmd("StartSomethingNew"),
    )

fun FuelDepotSideToDepot(): Command =
    Commands.sequence(
        autoFactory().trajectoryCmd("FuelDepotSideToDepot"),
    )

fun StartToFuel2(): Command =
    Commands.sequence(
        autoFactory().trajectoryCmd("StartToFuel2"),
    )

fun Outpost(): Command =
    Commands.sequence(
        StartToFuelDepotSide(),
        FuelDepotSideToDepot(),
        StartToFuel2()
    )

internal fun runPath(name: String): Command {
    val path = PathPlannerPath.fromPathFile(name)
    val startPose = path.pathPoses[0]
    return AutoBuilder.resetOdom(startPose)
        .andThen(AutoBuilder.followPath(path))
}