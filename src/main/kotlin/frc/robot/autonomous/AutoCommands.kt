package frc.robot.autonomous

import choreo.Choreo
import choreo.auto.AutoFactory
import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.commands.PathPlannerAuto
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.InstantCommand
import edu.wpi.first.wpilibj2.command.PrintCommand
import frc.robot.RobotContainer.autoFactory
import frc.robot.drive

// TODO: After basic trajectory following is working open path planner and look at the paths then implement here the trajectory chaining as one command with state machine integrated commands as well


fun smt(): Command = Commands.sequence(
 PrintCommand("is Starting"),
 autoFactory().resetOdometry("smt"),
 autoFactory().trajectoryCmd("smt"),
 PrintCommand("its done")
)


// fun depotDoubleCycle(): Command = depotDoubleCycle()

fun Test(): Command = PathPlannerAuto("Test")
