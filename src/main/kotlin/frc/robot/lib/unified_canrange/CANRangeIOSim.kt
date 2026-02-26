package frc.robot.lib.unified_canrange

import frc.robot.lib.extensions.m
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

/**
 * Simulation implementation of CANRangeIO for testing.
 *
 * Supports two modes:
 * - Number mode: Uses a LoggedNetworkNumber for distance in meters
 * - Boolean mode: Uses a LoggedNetworkBoolean for detection state
 */
class CANRangeIOSim(
    subsystemName: String,
    sensorName: String,
    private val usesNumber: Boolean = false,
    private val loggingConfig: UnifiedCANRangeLogging
) : CANRangeIO {

    override val inputs = LoggedSensorInputs()

    private val metersDetected: LoggedNetworkNumber?
    private val isDetecting: LoggedNetworkBoolean?

    init {
        if (usesNumber) {
            metersDetected =
                LoggedNetworkNumber(
                    "/Tuning/$subsystemName/$sensorName/metersDetected",
                    1.0
                )
            isDetecting = null
        } else {
            isDetecting =
                LoggedNetworkBoolean(
                    "/Tuning/$subsystemName/$sensorName/IsDetecting",
                    false
                )
            metersDetected = null
        }
    }

    override fun updateInputs() {
        val (distance, detecting) =
            when {
                usesNumber -> getNumberModeValues()
                else -> getBooleanModeValues()
            }
        if (loggingConfig.distance) inputs.distance = distance.m
        if (loggingConfig.isDetecting) inputs.isDetecting = detecting
    }

    private fun getNumberModeValues(): Pair<Double, Boolean> {
        val meters = metersDetected?.get() ?: 0.0
        val isDetecting = meters < DETECTION_THRESHOLD
        return Pair(meters, isDetecting)
    }

    private fun getBooleanModeValues(): Pair<Double, Boolean> {
        val isDetecting = this.isDetecting?.get() ?: false
        val meters = if (isDetecting) CLOSE_DISTANCE else FAR_DISTANCE
        return Pair(meters, isDetecting)
    }

    private companion object {
        const val DETECTION_THRESHOLD = 0.5
        const val CLOSE_DISTANCE = 0.01
        const val FAR_DISTANCE = 1.5
    }
}
