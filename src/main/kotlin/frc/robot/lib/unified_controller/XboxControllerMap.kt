package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class XboxControllerMap(port: Int) : UnifiedControllerIO {
    private val xboxController = CommandXboxController(port)
    override fun cross(): Trigger = xboxController.a()

    override fun circle(): Trigger = xboxController.b()

    override fun square(): Trigger = xboxController.x()

    override fun triangle(): Trigger = xboxController.y()

    override fun L1(): Trigger = xboxController.leftBumper()

    override fun R1(): Trigger = xboxController.rightBumper()

    override fun L2(): Trigger = xboxController.leftTrigger()

    override fun R2(): Trigger = xboxController.rightTrigger()

    override fun create(): Trigger = xboxController.back()

    override fun options(): Trigger = xboxController.start()

    override fun leftX(): Double = xboxController.leftX

    override fun leftY(): Double = xboxController.leftY

    override fun rightX(): Double = xboxController.rightX

    override fun rightY(): Double = xboxController.rightY

    override fun l2Axis(): Double = xboxController.leftTriggerAxis

    override fun r2Axis(): Double = xboxController.rightTriggerAxis
}
