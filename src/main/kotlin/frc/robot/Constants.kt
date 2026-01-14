package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.Mode
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val LOOP_TIME = 0.02 // [s]

val logLevel = LogLevel.DEV

val gameData: String = DriverStation.getGameSpecificMessage()

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

enum class Team(val color: String) {
    BLUE(Color.kBlue.toHexString()),
    RED(Color.kFirstRed.toHexString()),
    BOTH(Color.kPurple.toHexString()),
    NO_DATA(Color.kGray.toHexString())
}

val gameTimer
    get() = Timer.getTimestamp().sec

val timeStamp = listOf(0.sec, 20.sec, 45.sec, 70.sec, 95.sec, 120.sec, 150.sec)

@LoggedOutput(LogLevel.COMP)
val currentSide
    get() =
        (if (gameData.isNotEmpty()) {
            when (gameData.first()) {
                'B' -> Team.BLUE.color
                'R' -> Team.RED.color
                else -> Team.BOTH.color
            }
        } else Team.NO_DATA.color)

@LoggedOutput(LogLevel.COMP)
val timeUntilModeChange
    get() =
        gameTimer
            .let { now ->
                timeStamp.firstOrNull { it > now }?.minus(now) ?: now
            }[sec]
