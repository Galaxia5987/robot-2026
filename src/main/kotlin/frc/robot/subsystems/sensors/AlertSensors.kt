package frc.robot.subsystems.sensors

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.Command
import frc.robot.lib.logged_output.LoggedOutputManager.runOnce

class AlertSensors {
    private val disconnectedAlertSpindexer =
        Alert("Spindexer's Sensor's motor is disconnected", AlertType.kWarning)

    private val connectedAlertSpindexer =
        Alert("Spindexer's Sensor's motor is connected", AlertType.kInfo)

    private val disconnectedAlertTop =
        Alert("Top Sensor's motor is disconnected", AlertType.kWarning)

    private val connectedAlertTop =
        Alert("Top Sensor's motor is connected", AlertType.kInfo)

    private val disconnectedAlertAuxTop =
        Alert("AuxTop's Sensor's motor is disconnected", AlertType.kWarning)

    private val connectedAlertAuxTop =
        Alert("AuxTop's Sensor's motor is connected", AlertType.kInfo)

    fun isConnected(condition: Boolean) : Command = runOnce {
        if (condition) {
            disconnectedAlertSpindexer.set(false)
            connectedAlertSpindexer.set(true)
        }
        else {
            disconnectedAlertSpindexer.set(true)
            connectedAlertSpindexer.set(false)
        }
    }

    fun isConnectedTop(condition: Boolean) : Command = runOnce {
        if (condition) {
            disconnectedAlertTop.set(false)
            connectedAlertTop.set(true)
        }
        else {
            disconnectedAlertTop.set(true)
            connectedAlertTop.set(false)
        }
    }
    fun isConnectedAuxTop(condition: Boolean) : Command = runOnce {
        if (condition) {
            disconnectedAlertAuxTop.set(false)
            connectedAlertAuxTop.set(true)
        }
        else {
            disconnectedAlertAuxTop.set(true)
            connectedAlertAuxTop.set(false)
        }
    }
}