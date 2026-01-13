package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj2.command.button.Trigger

interface UnifiedControllerIO {
    fun cross(): Trigger
    fun circle(): Trigger
    fun square(): Trigger
    fun triangle(): Trigger
    fun L1(): Trigger
    fun R1(): Trigger
    fun L2(): Trigger
    fun R2(): Trigger
    fun create(): Trigger
    fun options(): Trigger

    fun leftX(): Double
    fun leftY(): Double
    fun rightX(): Double
    fun rightY(): Double
    fun l2Axis(): Double
    fun r2Axis(): Double
}
