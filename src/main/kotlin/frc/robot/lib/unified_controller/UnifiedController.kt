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

    private var isXboxController = false
    private var isSonyLinux = false

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

    fun cross() = Trigger { controller.cross().asBoolean }
    fun circle() = Trigger { controller.circle().asBoolean }
    fun square() = Trigger { controller.square().asBoolean }
    fun triangle() = Trigger { controller.triangle().asBoolean }
    fun L1() = Trigger { controller.L1().asBoolean }
    fun R1() = Trigger { controller.R1().asBoolean }
    fun L2() = Trigger { controller.L2().asBoolean }
    fun R2() = Trigger { controller.R2().asBoolean }
    fun create() = Trigger { controller.create().asBoolean }
    fun options() = Trigger { controller.options().asBoolean }

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
