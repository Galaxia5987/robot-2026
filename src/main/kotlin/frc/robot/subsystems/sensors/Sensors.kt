package frc.robot.subsystems.sensors

import edu.wpi.first.math.filter.Debouncer
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.sec
import frc.robot.lib.unified_canrange.UnifiedCANRange
import frc.robot.lib.unified_canrange.UnifiedCANRangeLogging
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.setpoints_manager.SetpointsManager.isShootingOnMove
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
            sensorName = "spindexerSensor",
            loggingConfig = UnifiedCANRangeLogging(distance = false)
        )
    private val frontSensor =
        UnifiedCANRange(
            FRONT_SENSOR_PORT,
            configuration = FRONT_SENSOR_CONFIG,
            sensorName = "frontSensor",
            simulationUsesNumber = false,
            loggingConfig =
                UnifiedCANRangeLogging(distance = false, signalStrength = true)
        )


    val hopperFrontHasBalls: Trigger = Trigger {
        frontSensor.inputs.signalStrength > MIN_SIGNAL_STRENGTH_HOPPER_FRONT
    }

    private val isIntakeOpen =
        Trigger { Extender.inputs.distance > EXTENDER_SETPOINT_TOLERANCE }
            .and(IntakingStates.CLOSED.trigger.debounce(0.7))

    private val preShooterHasBalls: Trigger = Trigger {
        spindexerSensor.isInRange
    }

    private val isSpindexerLoaded = Trigger {
        spindexerSensor.inputs.signalStrength > MIN_SIGNAL_STRENGTH_SPINDEXER
    }

    val hasFuel: Trigger =
        isSpindexerLoaded
            .or(hopperFrontHasBalls)
            .or(isIntakeOpen)
            .or(isShootingOnMove)
            .debounce(HAS_FUEL_DEBOUNCE[sec], Debouncer.DebounceType.kFalling)

    override fun periodic() {
        spindexerSensor.periodic()
        frontSensor.periodic()

        Logger.recordOutput("Subsystems/Sensors/hasFuel", hasFuel)
        Logger.recordOutput(
            "Subsystems/Sensors/preShooterHasBalls",
            preShooterHasBalls
        )
        Logger.recordOutput(
            "Subsystems/Sensors/isSpindexerLoaded",
            isIntakeOpen
        )
        Logger.recordOutput(
            "Subsystems/Sensors/isSpindexerLoaded",
            isIntakeOpen
        )
        Logger.recordOutput(
            "Subsystems/Sensors/hopperFrontHasBalls",
            hopperFrontHasBalls
        )
    }
}
