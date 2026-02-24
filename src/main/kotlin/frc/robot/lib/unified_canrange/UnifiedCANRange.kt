package frc.robot.lib.unified_canrange

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANrangeConfiguration
import frc.robot.CURRENT_MODE
import frc.robot.lib.Mode
import frc.robot.lib.getFileNameFromStack
import org.littletonrobotics.junction.Logger

data class UnifiedCANRangeLogging(
    val distance: Boolean = true,
    val isDetecting: Boolean = true,
    val signalStrength: Boolean = true
)

class UnifiedCANRange(
    private val port: Int,
    private val canbus: CANBus = CANBus("rio"),
    private val subsystemName: String = getFileNameFromStack(),
    private val sensorName: String = "CANRangeId $port",
    configuration: CANrangeConfiguration,
    private val loggingConfig: UnifiedCANRangeLogging =
        UnifiedCANRangeLogging(),
    private val simulationUsesNumber: Boolean = false
) {
    private val sensorIO: CANRangeIO =
        if (CURRENT_MODE == Mode.REAL) {
            CANRangeIOReal(port, canbus, configuration, loggingConfig)
        } else {
            CANRangeIOSim(
                subsystemName,
                sensorName,
                simulationUsesNumber,
                loggingConfig
            )
        }
    val isInRange: Boolean
        get() = sensorIO.inputs.isDetecting

    val inputs
        get() = sensorIO.inputs

    fun periodic() {
        sensorIO.updateInputs()
        Logger.processInputs("Subsystems/$subsystemName/$sensorName", inputs)
    }
}
