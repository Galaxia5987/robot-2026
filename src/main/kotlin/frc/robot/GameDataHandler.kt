package frc.robot

import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.min
import frc.robot.lib.extensions.sec
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkString
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class ShiftType {
    WON_AUTO,
    LOST_AUTO,
    ALL
}

private val autoWinner = LoggedNetworkString("/DriverDashboard/WinningAuto", "AUTO")
private val ignoreShifts = LoggedNetworkBoolean("/DriverDashboard/IgnoreShifts", false)


data class GameShift(
    val startTime: Time,
    val endTime: Time,
    val shiftType: ShiftType
)

fun didWeWinAuto(): Boolean{
    val msg = when(autoWinner.get()){
        "RED" -> "R"
        "BLUE" -> "B"
        else -> DriverStation.getGameSpecificMessage()
    }
    if(msg.isNullOrEmpty()) return true

    return when(msg.firstOrNull()?.uppercaseChar()){
        'R' -> IS_RED
        'B' -> !IS_RED
        else -> true
    }
}

private val SHIFTS = listOf(
    GameShift(2.min + 40.sec, 2.min + 10.sec, ShiftType.ALL), // auto + first shift
    GameShift(2.min + 10.sec, 1.min + 45.sec, ShiftType.LOST_AUTO),
    GameShift(1.min + 45.sec, 1.min + 20.sec, ShiftType.WON_AUTO),
    GameShift(1.min + 20.sec, 55.sec, ShiftType.LOST_AUTO),
    GameShift(55.sec, 30.sec, ShiftType.WON_AUTO),
    GameShift(30.sec, 0.sec, ShiftType.ALL) // endgame
)

val matchTime: Time // getMatchTime returns time left in the current period, so add 2min 20sec when in auto
    get() {
        val teleopDuration = 2.min + 20.sec
        val rawTime = DriverStation.getMatchTime()

        if (rawTime < 0.0) return 0.sec

        return rawTime.sec + if (DriverStation.isAutonomous()) teleopDuration else 0.sec
    }

val currentShift: GameShift?
    get() {
        val currentTime = matchTime
        return SHIFTS.find { currentTime >= it.endTime && currentTime < it.startTime }
    }

@LoggedOutput(LogLevel.COMP)
val isOurHubActive: Boolean
    get() = when (currentShift?.shiftType) {
        ShiftType.ALL -> true
        ShiftType.WON_AUTO -> didWeWinAuto()
        ShiftType.LOST_AUTO -> !didWeWinAuto()
        else -> true
    } || ignoreShifts.get()