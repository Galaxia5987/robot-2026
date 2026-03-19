package frc.robot

import edu.wpi.first.units.Units
import edu.wpi.first.units.measure.Time
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.min
import frc.robot.lib.extensions.sec
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

enum class ShiftType {
    WON_AUTO,
    LOST_AUTO,
    ALL
}

private val autoWinner =
    LoggedDashboardChooser<String>("DriverDashboard/WinningAuto").apply {
        addDefaultOption("AUTO", "AUTO")
        addOption("RED", "RED")
        addOption("BLUE", "BLUE")
    }

data class GameShift(
    val startTime: Time,
    val endTime: Time,
    val shiftType: ShiftType
)

fun didWeWinAuto(): Boolean {
    val msg =
        when (autoWinner.get()) {
            "RED" -> "R"
            "BLUE" -> "B"
            else -> DriverStation.getGameSpecificMessage()
        }
    if (msg.isNullOrEmpty()) return true

    return when (msg.firstOrNull()?.uppercaseChar()) {
        'R' -> IS_RED
        'B' -> !IS_RED
        else -> true
    }
}

fun isOurShift(shift: GameShift?) =
    when ((shift?.shiftType)) {
        ShiftType.ALL -> true
        ShiftType.WON_AUTO -> didWeWinAuto()
        ShiftType.LOST_AUTO -> !didWeWinAuto()
        else -> true
    } || DriverStation.getMatchTime() < 0.0

private val SHIFTS =
    listOf(
        GameShift(
            2.min + 40.sec,
            2.min + 10.sec,
            ShiftType.ALL
        ), // auto + first shift
        GameShift(2.min + 10.sec, 1.min + 45.sec, ShiftType.LOST_AUTO),
        GameShift(1.min + 45.sec, 1.min + 20.sec, ShiftType.WON_AUTO),
        GameShift(1.min + 20.sec, 55.sec, ShiftType.LOST_AUTO),
        GameShift(55.sec, 30.sec, ShiftType.WON_AUTO),
        GameShift(30.sec, 0.sec, ShiftType.ALL) // endgame
    )

@LoggedOutput(LogLevel.COMP, path = "GameData")
val matchTime:
    Double // getMatchTime returns time left in the current period, so add 2min 20sec when in auto
    get() =
        (DriverStation.getMatchTime().sec +
                (if (DriverStation.isAutonomous()) (2.min + 20.sec) else 0.sec))
            .`in`(Units.Seconds)

val currentShiftIndex
    get() = SHIFTS.indexOfFirst { matchTime.sec in it.endTime..it.startTime }

val currentPreShift
    get() =
        SHIFTS.findLast {
            (matchTime.sec) in
                it.endTime..it.startTime +
                        (if (isOurShift(it)) 2.0.sec else 0.sec)
        }

val currentShift: GameShift?
    get() = SHIFTS.getOrNull(currentShiftIndex)

val nextShift: GameShift?
    get() {
        return if (currentShiftIndex < SHIFTS.size - 1)
            SHIFTS[currentShiftIndex + 1]
        else null
    }

val isOurHubPreActive: Boolean
    get() = isOurShift(currentPreShift)

@LoggedOutput(LogLevel.COMP, path = "GameData")
val isOurHubActive: Boolean
    get() = isOurShift(currentShift)

@LoggedOutput(LogLevel.COMP, path = "GameData")
val timeLeftForShift: Double
    get() = matchTime - (currentShift?.endTime ?: 0.sec)[sec]

private fun Boolean.toShiftActiveMessage() =
    if (matchTime <= 30) "ENDGAME"
    else if (matchTime > (2.min + 20.sec)[sec]) "AUTO"
    else if (matchTime in (2.min + 10.sec)[sec]..(2.min + 20.sec)[sec])
        "TRANSITION"
    else if (this) "ACTIVE" else "INACTIVE"

@LoggedOutput(LogLevel.COMP, path = "GameData")
val primaryDashboardShiftMessage: String
    get() = isOurHubActive.toShiftActiveMessage()

@LoggedOutput(LogLevel.COMP, path = "GameData")
val secondaryDashboardShiftMessage: String
    get() {
        val isNextShiftActive =
            !isOurHubActive || nextShift?.shiftType == ShiftType.ALL
        return "${isNextShiftActive.toShiftActiveMessage()} in ${timeLeftForShift.toInt()} seconds"
    }

@LoggedOutput(LogLevel.COMP, path = "GameData")
val shiftMessage: String
    get() = "${currentShiftIndex+1}/${SHIFTS.size}"
