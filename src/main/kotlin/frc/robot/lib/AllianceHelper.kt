package frc.robot.lib

import edu.wpi.first.wpilibj.DriverStation
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

@LoggedOutput(LogLevel.COMP)
val IS_RED: Boolean
    get() =
        DriverStation.getAlliance().isPresent &&
            DriverStation.getAlliance().get() == DriverStation.Alliance.Red
