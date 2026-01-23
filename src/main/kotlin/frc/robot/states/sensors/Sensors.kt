package frc.robot.states.sensors

import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.unified_canrange.UnifiedCANRange
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val FULL = 1.m // TODO: change the value
private val HALF_FULL = 0.5.m // TODO: change the value
private val HAS_FUEL = 5.cm // TODO: change the value

private val spindexerSensor =
    UnifiedCANRange(
        SPINDEXER_SENSOR_PORT,
        configuration = SPINDEXER_SENSOR_CONFIG
    )
private val topSensor =
    UnifiedCANRange(TOP_SENSOR_PORT, configuration = TOP_SENSOR_CONFIG)
private val auxTopSensor =
    UnifiedCANRange(AUX_TOP_SENSOR, configuration = AUX_TOP_SENSOR_CONFIG)

@LoggedOutput(LogLevel.COMP)
val isHalfFull: Trigger = Trigger {
    (topSensor.inputs.distance < HALF_FULL).and(
        auxTopSensor.inputs.distance < HALF_FULL
    )
}
@LoggedOutput(LogLevel.COMP)
val isFull: Trigger = Trigger {
    (topSensor.inputs.distance <= FULL).and(
        auxTopSensor.inputs.distance <= FULL
    )
}

@LoggedOutput(LogLevel.COMP)
val hasFuel: Trigger = Trigger { spindexerSensor.inputs.distance < HAS_FUEL }
