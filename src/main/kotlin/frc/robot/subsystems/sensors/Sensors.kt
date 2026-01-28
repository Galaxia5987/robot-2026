package frc.robot.subsystems.sensors

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.m
import frc.robot.lib.unified_canrange.UnifiedCANRange
import org.littletonrobotics.junction.Logger

object Sensors : SubsystemBase() {

    private val FULL = 1.m // TODO: change the value
    private val HALF_FULL = 0.5.m // TODO: change the value

    private val spindexerSensor =
        UnifiedCANRange(
            SPINDEXER_SENSOR_PORT,
            configuration = SPINDEXER_SENSOR_CONFIG
        )
    private val topSensor =
        UnifiedCANRange(TOP_SENSOR_PORT, configuration = TOP_SENSOR_CONFIG)
    private val auxTopSensor =
        UnifiedCANRange(AUX_TOP_SENSOR, configuration = AUX_TOP_SENSOR_CONFIG)

    val isHalfFull: Trigger = Trigger {
        (topSensor.inputs.distance < HALF_FULL).and(
            auxTopSensor.inputs.distance < HALF_FULL
        )
    }

    val isFull: Trigger = Trigger {
        (topSensor.inputs.distance <= FULL).and(
            auxTopSensor.inputs.distance <= FULL
        )
    }

    val hasFuel: Trigger = Trigger { spindexerSensor.isInRange }

    override fun periodic() {
        spindexerSensor.periodic()
        topSensor.periodic()
        auxTopSensor.periodic()

        Logger.recordOutput("Subsystems/$name/hasFuel", hasFuel)
        Logger.recordOutput("Subsystems/$name/isHalfFull", isHalfFull)
        Logger.recordOutput("Subsystems/$name/isFull", isFull)
    }
}
