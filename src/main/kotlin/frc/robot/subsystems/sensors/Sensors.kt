package frc.robot.subsystems.sensors

import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.m
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.subsystems.intake.roller.Roller.isConnected
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
    private val auxTopSensor =
        UnifiedCANRange(
            AUX_TOP_SENSOR,
            configuration = AUX_TOP_SENSOR_CONFIG,
            sensorName = "auxTopSensor",
            simulationUsesNumber = true
        )

    fun averageFuelDistance(): Distance =
        (topSensor.inputs.distance + auxTopSensor.inputs.distance) / 2.0

    val cantCloseIntake: Trigger = Trigger {
        (averageFuelDistance() < HALF_FULL)
    }

    val isFull: Trigger = Trigger { (averageFuelDistance() < FULL) }

    val hasFuel: Trigger = Trigger { spindexerSensor.isInRange }

    // --- Alerts ---



    val setStatus = Trigger { spindexerSensor.inputs.connected }
        .onChange(isConnected(spindexerSensor.inputs.connected))

    val setStatusTop = Trigger { topSensor.inputs.connected }
        .onChange(isConnected(topSensor.inputs.connected))

    val setStatusAuxTop = Trigger { auxTopSensor.inputs.connected }
        .onChange(isConnected(auxTopSensor.inputs.connected))

    override fun periodic() {
        spindexerSensor.periodic()
        topSensor.periodic()
        auxTopSensor.periodic()

        Logger.recordOutput("Subsystems/$name/hasFuel", hasFuel)
        Logger.recordOutput("Subsystems/$name/cantCloseIntake", cantCloseIntake)
        Logger.recordOutput("Subsystems/$name/isFull", isFull)
    }
}
