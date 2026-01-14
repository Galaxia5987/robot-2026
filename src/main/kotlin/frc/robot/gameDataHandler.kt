package frc.robot

import edu.wpi.first.hal.DriverStationJNI.getMatchTime
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.DriverStation.Alliance.Blue
import edu.wpi.first.wpilibj.DriverStation.Alliance.Red
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.sec
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val gameData: String
    get() = DriverStation.getGameSpecificMessage()

private val MAX_GAME_TIME = 160.sec
val gameTimer: Time
    get() = MAX_GAME_TIME - getMatchTime().sec

private val SHIFT_CHANGE_TIMES =
    listOf(0, 20, 30, 55, 80, 105, 130, 160).map { it.sec }

private var currentShift = SHIFT_CHANGE_TIMES[3]

private var allianceCaptured = false
private var currentActiveAlliance = Red

@LoggedOutput(LogLevel.COMP) var isHubActive: Boolean = false

@LoggedOutput(LogLevel.COMP)
val activeColor: String
    get() =
        if (!allianceCaptured) Color.kGray.toHexString()
        else if (currentActiveAlliance == Red) Color.kRed.toHexString()
        else Color.kBlue.toHexString()

fun getActiveAlliance() {
    val time = gameTimer
    if (time >= 23.sec && !allianceCaptured) {
        val data = gameData
        if (data.isNotEmpty()) {
            isHubActive =
                when (data.firstOrNull()) {
                    'B' -> {
                        currentActiveAlliance = Red
                        allianceCaptured = true
                        IS_RED
                    }
                    'R' -> {
                        currentActiveAlliance = Blue
                        allianceCaptured = true
                        !IS_RED
                    }
                    else -> {
                        true
                    }
                }
        }
        allianceCaptured = true
    }

    if (time >= SHIFT_CHANGE_TIMES[2] && allianceCaptured) {
        time.let { now ->
            val nextShift = SHIFT_CHANGE_TIMES.indexOfFirst { it > now }.sec
            if (currentShift != nextShift) {
                currentShift = nextShift
                isHubActive = !isHubActive
                currentActiveAlliance =
                    if (currentActiveAlliance == Red) Blue else Red
            }
        }
    }
}

@LoggedOutput(LogLevel.COMP)
val timeUntilModeChange
    get() =
        gameTimer
            .let { now ->
                SHIFT_CHANGE_TIMES.firstOrNull { it > now }?.minus(now) ?: now
            }[sec]
