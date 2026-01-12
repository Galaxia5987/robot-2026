package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class Button(val value: Int) {
    Square(3),
    Cross(1),
    Circle(2),
    Triangle(4),
    L1(5),
    R1(6),
    Create(7),
    Options(8),
    PS(9),
}

enum class Axis(val value: Int) {
    LeftX(0),
    LeftY(1),
    RightX(3),
    RightY(4),
    L2(2),
    R2(5)
}

class LinuxPsControllerMap(port: Int) : UnifiedControllerIO {
    private val controller = CommandGenericHID(port)

    override fun cross(): Trigger = controller.button(Button.Cross.value)

    override fun circle(): Trigger = controller.button(Button.Circle.value)

    override fun square(): Trigger = controller.button(Button.Square.value)

    override fun triangle(): Trigger = controller.button(Button.Triangle.value)

    override fun L1(): Trigger = controller.button(Button.L1.value)

    override fun R1(): Trigger = controller.button(Button.R1.value)

    override fun L2(): Trigger = Trigger { r2Axis() > 0.5 }

    override fun R2(): Trigger = Trigger { l2Axis() > 0.5 }

    override fun create(): Trigger = controller.button(Button.Create.value)

    override fun options(): Trigger = controller.button(Button.Options.value)

    override fun leftX(): Double = controller.getRawAxis(Axis.LeftX.value)

    override fun leftY(): Double = controller.getRawAxis(Axis.LeftY.value)

    override fun rightX(): Double = controller.getRawAxis(Axis.RightX.value)

    override fun rightY(): Double = controller.getRawAxis(Axis.RightY.value)

    override fun l2Axis(): Double = controller.getRawAxis(Axis.L2.value)

    override fun r2Axis(): Double = controller.getRawAxis(Axis.R2.value)
}
