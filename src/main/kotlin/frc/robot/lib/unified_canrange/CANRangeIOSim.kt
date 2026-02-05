package frc.robot.lib.unified_canrange

import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.meters
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean
import org.littletonrobotics.junction.networktables.LoggedNetworkNumber

class CANRangeIOSim(subsystemName: String, sensorName: String, private val usesNumber: Boolean = false) : CANRangeIO {
    override val inputs = LoggedSensorInputs()

    private val metersDetected =
        LoggedNetworkNumber(
            "/Tuning/$subsystemName/$sensorName/MetersDetected", 1.0)
    private val isDetecting = LoggedNetworkBoolean(
            "/Tuning/$subsystemName/$sensorName/isDetecting", false)


    override fun updateInputs() {


        var meters_value = 0.0
        var boolean_value = false

        if(usesNumber) {
            meters_value = metersDetected.get()
            boolean_value = meters_value < 0.5
        }
        else {
            boolean_value = isDetecting.get()
            if(isDetecting.get()){
                meters_value = 0.01
            }
            else{
                meters_value = 3.0
            }
        }

        inputs.distance= meters_value.m
        inputs.isDetecting= boolean_value
    }
}
