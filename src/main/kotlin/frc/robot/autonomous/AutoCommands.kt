package frc.robot.autonomous

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

// fun depotDoubleCycle(): Command = depotDoubleCycle()
