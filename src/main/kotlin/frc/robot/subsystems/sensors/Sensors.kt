package frc.robot.subsystems.sensors

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.states.intaking.IntakingStates
import frc.robot.subsystems.intake.extender.EXTENDER_SETPOINT_TOLERANCE
import frc.robot.subsystems.intake.extender.Extender
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

    val isFull: Trigger = Trigger { topSensor.isInRange }

    private val isIntakeOpen =
        Trigger { Extender.inputs.distance > EXTENDER_SETPOINT_TOLERANCE }
            .and(IntakingStates.CLOSED.trigger.debounce(0.4))

    private val preShooterHasBalls: Trigger = Trigger {
        spindexerSensor.isInRange
    }

    private val isSpindexerLoaded = Trigger {
        spindexerSensor.inputs.signalStrength >
            MIN_SIGNAL_STRENGTH_FOR_MEASUREMENT
    }

    val hasFuel: Trigger =
        isSpindexerLoaded
            .or(isFull)
            .or(isIntakeOpen)
            .debounce(HAS_FUEL_DEBOUNCE[sec], Debouncer.DebounceType.kFalling)

    override fun periodic() {
        spindexerSensor.periodic()
        topSensor.periodic()

        Logger.recordOutput("Subsystems/$name/hasFuel", hasFuel)
        Logger.recordOutput(
            "Subsystems/$name/preShooterHasBalls",
            preShooterHasBalls
        )
        Logger.recordOutput("Subsystems/$name/isSpindexerLoaded", isIntakeOpen)
        Logger.recordOutput("Subsystems/$name/isFull", isFull)
    }
}
