package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.Mode
import frc.robot.lib.extensions.get
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val LOOP_TIME = 0.02 // [s]

val logLevel = LogLevel.DEBUG

@LoggedOutput(LogLevel.COMP)
val CURRENT_MODE: Mode
    get() =
        if (LoggedRobot.isReal()) {
            Mode.REAL
        } else {
            if (System.getenv("isReplay") == "true") {
                Mode.REPLAY
            } else {
                Mode.SIM
            }
        }

val IS_SIM = Trigger { CURRENT_MODE == Mode.SIM }

val isEnabled = Trigger { DriverStation.isEnabled() }
