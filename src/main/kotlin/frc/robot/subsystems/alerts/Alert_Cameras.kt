package frc.robot.subsystems.alerts

import edu.wpi.first.wpilibj.Alert
import edu.wpi.first.wpilibj.Alert.AlertType
import edu.wpi.first.wpilibj2.command.SubsystemBase


enum class CameraAlert(name: String) {
    CAMERA1("Camera 1"),
    CAMERA2("Camera 2"),
    CAMERA3("Camera 3"),
    CAMERA4("Camera 4"),
    CAMERA5("Camera 5"),
    CAMERA6("Camera 6"),
    CAMERA7("Camera 7"),
    CAMERA8("Camera 8"),
    CAMERA9("Camera 9"),
    CAMERA10("Camera 10"),
    CAMERA11("Camera 11"),
    CAMERA12("Camera 12"),
    CAMERA13("Camera 13"),
    CAMERA14("Camera 14"),
    CAMERA15("Camera 15"),
    CAMERA16("Camera 16"),
    CAMERA17("Camera 17"),
    CAMERA18("Camera 18"),
    CAMERA19("Camera 19"),
    CAMERA20("Camera 20");
}

class CameraAlertSubsystem : SubsystemBase() {

    private val disconnectedAlert =
        Alert("Camera is disconnected", AlertType.kWarning)

    private val connectedAlert =
        Alert("Camera is connected", AlertType.kInfo)

    fun setCameraStatus(isConnected: Boolean) {
        if (isConnected) {
            disconnectedAlert.set(false)
            connectedAlert.set(true)
        } else {
            disconnectedAlert.set(true)
            connectedAlert.set(false)
        }
    }

    override fun periodic() {
        val isConnected = false // IDK how to do for this an automatic val
        setCameraStatus(isConnected)
    }
}





