package frc.robot.lib.unified_canrange

import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean

class CANRangeIOSim(subsystemName: String, sensorName: String) : CANRangeIO {
    override val inputs = LoggedSensorInputs()
    private val isDetecting =
        LoggedNetworkBoolean("/Tuning/$subsystemName/$sensorName/IsDetecting", false)

    override fun updateInputs() {
        val detecting = this.isDetecting.get()
        inputs.isDetecting = detecting
        if (detecting) {
            inputs.distance = 1.cm
        } else {
            inputs.distance = 3.m
        }
    }
}
