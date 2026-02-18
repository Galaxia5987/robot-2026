package frc.robot.subsystems.sensors

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.subsystems.intake.extender.Extender
import frc.robot.subsystems.intake.extender.EXTENDER_SETPOINT_TOLERANCE
import org.littletonrobotics.junction.Logger

object Sensors : SubsystemBase() {

    private val FULL = 0.2.m // TODO: change the value
    private val HALF_FULL = 0.5.m // TODO: change the value

    private val spindexerSensor =
        UnifiedCANRange(
            SPINDEXER_SENSOR_PORT,
            configuration = SPINDEXER_SENSOR_CONFIG,
            sensorName = "spindexerSensor"
        )
    private val topSensor =
        UnifiedCANRange(
            TOP_SENSOR_PORT,
            configuration = TOP_SENSOR_CONFIG,
            sensorName = "topSensor",
            simulationUsesNumber = true
        )
    private val middleTopSensor =
        UnifiedCANRange(
            MIDDLE_SENSOR_PORT,
            configuration = MIDDLE_SENSOR_CONFIG,
            sensorName = "auxTopSensor",
            simulationUsesNumber = true
        )

    fun averageFuelDistance(): Distance =
        (topSensor.inputs.distance + middleTopSensor.inputs.distance) / 2.0

    val cantCloseIntake: Trigger = Trigger {
        (averageFuelDistance() < HALF_FULL)
    }

    val isFull: Trigger = Trigger { (averageFuelDistance() < FULL) }

    val hasFuel: Trigger =
        Trigger {
                spindexerSensor.inputs.signalStrength >
                    MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT
            }.or { topSensor.isInRange }.or { middleTopSensor.isInRange }.or { Extender.inputs.distance > EXTENDER_SETPOINT_TOLERANCE }
            .debounce(HAS_FUEL_DEBOUNCE[sec], Debouncer.DebounceType.kFalling)
    val isSpindexerLoaded: Trigger = Trigger { spindexerSensor.isInRange }

    override fun periodic() {
        spindexerSensor.periodic()
        topSensor.periodic()
        middleTopSensor.periodic()

        Logger.recordOutput("Subsystems/$name/hasFuel", hasFuel)
        Logger.recordOutput(
            "Subsystems/$name/isSpindexerLoaded",
            isSpindexerLoaded
        )
        Logger.recordOutput("Subsystems/$name/cantCloseIntake", cantCloseIntake)
        Logger.recordOutput("Subsystems/$name/isFull", isFull)
    }
}
