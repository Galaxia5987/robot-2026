package frc.robot.lib.unified_canrange

import com.ctre.phoenix6.CANBus
import com.ctre.phoenix6.configs.CANrangeConfiguration
import com.ctre.phoenix6.hardware.CANrange

class CANRangeIOReal(
    private val port: Int,
    private val canbus: CANBus = CANBus("rio"),
    configuration: CANrangeConfiguration
) : CANRangeIO {
    override val inputs = LoggedSensorInputs()

    private val CANrange = CANrange(port, canbus)

    init {
        CANrange.configurator.apply(configuration)
    }

    override fun updateInputs() {
        inputs.distance = CANrange.distance.value
        inputs.isDetecting = CANrange.isDetected.value
        inputs.signalStrength = CANrange.signalStrength.value
    }
}
