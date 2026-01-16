package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command

internal fun runPath(name: String): Command {
    val path = PathPlannerPath.fromPathFile(name)
    val startPose = path.pathPoses[0]
    return AutoBuilder.resetOdom(startPose)
        .andThen(AutoBuilder.followPath(path))
}

fun startToFuelDepotSide(): Command = PathPlannerAuto("startToFuelDepotSide")

fun fuelStep1ToStep2DepotSide(): Command =
    PathPlannerAuto("fuelStep1ToStep2DepotSide")

fun stopBeforeDepot(): Command = PathPlannerAuto("stopBeforeDepot")

fun bumpToDepot(): Command = PathPlannerAuto("bumpToDepot")

fun startToFuelOutpostSide(): Command =
    PathPlannerAuto("startToFuelOutpostSide")

fun fuelStep1ToStep2(): Command = PathPlannerAuto("fuelStep1ToStep2OutpostSide")

fun stopBeforeOutpost(): Command = PathPlannerAuto("stopBeforeOutpost")

fun bumpToOutpost(): Command = PathPlannerAuto("bumpToOutpost")

fun BumpToDeputTest():Command= runPath("BumpToDeput")