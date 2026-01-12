package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.logged_output.LoggedOutputManager.runOnce
import org.littletonrobotics.junction.Logger.recordOutput

class UnifiedController(val port: Int) {
    init {
        recordOutput("controllerType", "PsWindows")
    }

    private val hid =
        CommandGenericHID(port) // easier than creating povs for each one

    private var controller: UnifiedControllerIO =
        PsControllerMap(port) // default value

    var isXboxController = false
    var isSonyLinux = false

    fun updateControllerType() {
        isXboxController =
            DriverStation.getJoystickName(port).lowercase().contains("box")
        isSonyLinux =
            DriverStation.getJoystickName(port).lowercase().contains("sony")
    }

    val controllerTrigger =
        Trigger {
                updateControllerType()
                isXboxController || isSonyLinux
            }
            .apply {
                and { isXboxController }
                    .onTrue(
                        runOnce {
                            controller = XboxControllerMap(port)
                            recordOutput("controllerType", "Xbox")
                        }
                    )
                and { isSonyLinux }
                    .onTrue(
                        runOnce {
                            controller = LinuxPsControllerMap(port)
                            recordOutput("controllerType", "PsLinux")
                        }
                    )
                onFalse(
                    runOnce {
                        controller = PsControllerMap(port)
                        recordOutput("controllerType", "PsWindows")
                    }
                )
            } // has to be a trigger. need to wait until the controller is readable.

    fun cross() = controller.cross()
    fun circle() = controller.circle()
    fun square() = controller.square()
    fun triangle() = controller.triangle()
    fun L1() = controller.L1()
    fun R1() = controller.R1()
    fun L2() = controller.L2()
    fun R2() = controller.R2()
    fun create() = controller.create()
    fun options() = controller.options()

    val leftX
        get() = controller.leftX()
    val leftY
        get() = controller.leftY()
    val rightX
        get() = controller.rightX()
    val rightY
        get() = controller.rightY()
    val l2Axis
        get() = controller.l2Axis()
    val r2Axis
        get() = controller.r2Axis()

    fun povUp() = hid.povUp()
    fun povUpRight() = hid.povUpRight()
    fun povRight() = hid.povRight()
    fun povDownRight() = hid.povDownRight()
    fun povDown() = hid.povDown()
    fun povDownLeft() = hid.povDownLeft()
    fun povLeft() = hid.povLeft()
    fun povUpLeft() = hid.povUpLeft()
}
