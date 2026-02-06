package frc.robot

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.util.Color
import frc.robot.lib.IS_RED
import frc.robot.lib.extensions.min
import frc.robot.lib.extensions.sec
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private var isShiftOneActiveRedBackingField: Boolean? = null

private fun isShiftOneActiveRed(): Boolean? {
    if (isShiftOneActiveRedBackingField != null) {
        return isShiftOneActiveRedBackingField
    }
    val message = DriverStation.getGameSpecificMessage()
    if (message.isEmpty()) return null

    isShiftOneActiveRedBackingField =
        when (message.firstOrNull()) {
            'R' -> false
            'B' -> true
            else -> null
        }
    return isShiftOneActiveRedBackingField
}

private val matchTime
    get() = DriverStation.getMatchTime().sec

private val SHIFT_CHANGES =
    listOf(2.min + 10.sec, 1.min + 45.sec, 1.min + 20.sec, 55.sec, 30.sec)

@LoggedOutput(LogLevel.COMP)
val isOurHubActive: Boolean
    get() {
        // Both Hubs are active in the beginning and end of the match.
        val bothHubsActive =
            matchTime !in SHIFT_CHANGES.last()..SHIFT_CHANGES.first()

        val wasShiftOneOurs = IS_RED == isShiftOneActiveRed()

        val currentIndex = SHIFT_CHANGES.indexOfFirst { matchTime > it }
        val isCurrentShiftOdd = currentIndex % 2 == 1

        return bothHubsActive || (wasShiftOneOurs == isCurrentShiftOdd)
    }

@LoggedOutput(LogLevel.COMP)
val activeColor: Color
    get() {
        return (if (
            isShiftOneActiveRed() == null || matchTime < SHIFT_CHANGES.last()
        )
            Color.kPurple
        else {
            if (isOurHubActive)
                if (IS_RED) Color.kOrangeRed else Color.kFirstBlue
            else if (IS_RED) Color.kFirstBlue else Color.kOrangeRed
        })
    }

@LoggedOutput(LogLevel.COMP)
val timeUntilNextShift: Double
    get() {
        SHIFT_CHANGES.find { matchTime > it }
            ?.let {
                return (matchTime - it).`in`(sec)
            }
        return 0.0
    }
