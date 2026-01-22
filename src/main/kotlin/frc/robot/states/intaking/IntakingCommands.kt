package frc.robot.states.intaking

import edu.wpi.first.wpilibj2.command.Command
import frc.robot.roller
import frc.robot.subsystems.roller.RollerPositions

fun


fun closed(): Command{
    return roller.setTarget(RollerPositions.STOP).alongWith()
}