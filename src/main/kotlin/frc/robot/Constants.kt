package frc.robot

import edu.wpi.first.math.geometry.Rectangle2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.units.measure.Distance
import frc.robot.lib.IS_RED
import frc.robot.lib.Mode
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import org.littletonrobotics.junction.LoggedRobot
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

const val LOOP_TIME = 0.02 // [s]

val logLevel = LogLevel.DEV

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

val ALLIANCE_ZONE_WIDTH: Distance = 4.03.m
val ALLIANCE_ZONE_HEIGHT: Distance = 8.07.m

val FIELD_WIDTH: Distance = 16.54.m
val FIELD_HEIGHT: Distance = 4.03.m

val ALLIANCE_ZONE
    get() =
        Rectangle2d(
            Translation2d(
                if (IS_RED) (FIELD_WIDTH - ALLIANCE_ZONE_WIDTH)
                else ALLIANCE_ZONE_WIDTH,
                ALLIANCE_ZONE_HEIGHT
            ),
            Translation2d(if (IS_RED) FIELD_WIDTH else 0.m, 0.m)
        )
