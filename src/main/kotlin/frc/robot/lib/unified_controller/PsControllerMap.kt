package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller
import edu.wpi.first.wpilibj2.command.button.Trigger

class PsControllerMap(port: Int) : UnifiedControllerIO {
    private val ps5Controller = CommandPS5Controller(port)

    override fun cross(): Trigger = ps5Controller.cross()

    override fun circle(): Trigger = ps5Controller.circle()

    override fun square(): Trigger = ps5Controller.square()

    override fun triangle(): Trigger = ps5Controller.triangle()

    override fun L1(): Trigger = ps5Controller.L1()

    override fun R1(): Trigger = ps5Controller.R1()

    override fun L2(): Trigger = ps5Controller.L2()

    override fun R2(): Trigger = ps5Controller.R2()

    override fun create(): Trigger = ps5Controller.create()

    override fun options(): Trigger = ps5Controller.options()

    override fun leftX(): Double = ps5Controller.leftX

    override fun leftY(): Double = ps5Controller.leftY

    override fun rightX(): Double = ps5Controller.rightX

    override fun rightY(): Double = ps5Controller.rightY

    override fun l2Axis(): Double = ps5Controller.l2Axis

    override fun r2Axis(): Double = ps5Controller.r2Axis
}
