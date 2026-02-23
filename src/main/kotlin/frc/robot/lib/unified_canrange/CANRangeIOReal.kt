package frc.robot.lib.unified_canrange

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.CANrange

class CANRangeIOReal(
    private val port: Int,
    private val canbus: CANBus = CANBus("rio"),
    configuration: CANrangeConfiguration,
    private val loggingConfig: UnifiedCANRangeLogging
) : CANRangeIO {
    override val inputs = LoggedSensorInputs()

    private val CANrange = CANrange(port, canbus)

    init {
        CANrange.configurator.apply(configuration)
    }

    override fun updateInputs() {
        if (loggingConfig.distance) inputs.distance = CANrange.distance.value
        if (loggingConfig.isDetecting)
            inputs.isDetecting = CANrange.isDetected.value
        if (loggingConfig.signalStrength)
            inputs.signalStrength = CANrange.signalStrength.value
    }
}
