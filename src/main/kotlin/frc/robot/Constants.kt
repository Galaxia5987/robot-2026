package frc.robot

import edu.wpi.first.units.Units
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.Mode
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

val timeStamp = listOf(0.sec, 20.sec, 45.sec, 70.sec, 95.sec, 120.sec, 150.sec)

@LoggedOutput(LogLevel.COMP)
val timeUntilModeChange
    get() =
        when (val timer = Timer.getTimestamp().sec) {
            in timeStamp[0]..timeStamp[1] -> timeStamp[1] - timer
            in timeStamp[1]..timeStamp[2] -> timeStamp[2] - timer
            in timeStamp[2]..timeStamp[3] -> timeStamp[3] - timer
            in timeStamp[3]..timeStamp[4] -> timeStamp[4] - timer
            in timeStamp[4]..timeStamp[5] -> timeStamp[5] - timer
            in timeStamp[5]..timeStamp[6] -> timeStamp[6] - timer
            else -> timer
        }.`in`(Units.Second)
