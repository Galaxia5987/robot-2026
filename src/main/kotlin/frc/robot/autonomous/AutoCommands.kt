package frc.robot.autonomous

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.drive

// TODO: After basic trajectory following is working open path planner and look at the paths then
// implement here the trajectory chaining as one command with state machine integrated commands as
// well

fun startToFuelDepotSide(): Command =
    Commands.sequence(
        drive.autoFactory.resetOdometry("StartToFuelDepotSide"),
        drive.autoFactory.trajectoryCmd("StartToFuelDepotSide"),
    )

fun startSomethingNew(): Command = drive.defer {
    drive.autoFactory.trajectoryCmd("StartSomethingNew")
}
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
