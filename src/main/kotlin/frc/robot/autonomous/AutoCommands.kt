package frc.robot.autonomous

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well

private fun followPath(path: String) = AutoBuilder.followPath(PathPlannerPath.fromPathFile(path))

fun startToFuelDepotSide(): Command = followPath("StartToFuelDepotSide")

fun fuelToDepotStart(): Command = followPath("FuelToDepotStart")

fun test2(): Command = startToFuelDepotSide().andThen(fuelToDepotStart())

fun test(): Command =
    AutoBuilder.followPath(PathPlannerPath.fromPathFile("Test"))
//    Commands.sequence(
//        drive.autoFactory.resetOdometry("StartSomethingNew"),
//        drive.autoFactory.trajectoryCmd("StartSomethingNew"),
//    )

fun fuelDepotSideToDepot(): Command =
    Commands.sequence(
        drive.autoFactory.trajectoryCmd("FuelDepotSideToDepot"),
    )

fun startToFuel2(): Command =
    Commands.sequence(
        drive.autoFactory.trajectoryCmd("StartToFuel2"),
    )

fun outpost(): Command =
    Commands.sequence(
        startToFuelDepotSide(),
        fuelDepotSideToDepot(),
        startToFuel2()
    )

fun depotDoubleCycle(): Command = depotDoubleCycle()
