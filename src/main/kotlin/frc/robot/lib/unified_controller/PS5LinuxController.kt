package frc.robot.lib.unified_controller

import edu.wpi.first.wpilibj2.command.CommandScheduler
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.button.Trigger

class PS5LinuxController(port: Int) {
    private val controller = CommandXboxController(port)

    fun square(): Trigger = controller.x()

    /**
     * Constructs a Trigger instance around the cross button's digital signal.
     *
     * @return a Trigger instance representing the cross button's digital signal
     * attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .cross
     */
    fun cross(): Trigger = controller.a()

    /**
     * Constructs a Trigger instance around the circle button's digital signal.
     *
     * @return a Trigger instance representing the circle button's digital
     * signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .circle
     */
    fun circle(): Trigger = controller.b()

    /**
     * Constructs a Trigger instance around the triangle button's digital
     * signal.
     *
     * @return a Trigger instance representing the triangle button's digital
     * signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .triangle
     */
    fun triangle(): Trigger = controller.y()

    /**
     * Constructs a Trigger instance around the left trigger 1 button's digital
     * signal.
     *
     * @return a Trigger instance representing the left trigger 1 button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .L1
     */
    fun L1(): Trigger = controller.leftBumper()

    /**
     * Constructs a Trigger instance around the right trigger 1 button's digital
     * signal.
     *
     * @return a Trigger instance representing the right trigger 1 button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .R1
     */
    fun R1(): Trigger = controller.rightBumper()

    /**
     * Constructs a Trigger instance around the left trigger 2 button's digital
     * signal.
     *
     * @return a Trigger instance representing the left trigger 2 button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .L2
     */
    fun L2(): Trigger = controller.leftTrigger()

    /**
     * Constructs a Trigger instance around the right trigger 2 button's digital
     * signal.
     *
     * @return a Trigger instance representing the right trigger 2 button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .R2
     */
    fun R2(): Trigger = controller.rightTrigger()

    /**
     * Constructs a Trigger instance around the create button's digital signal.
     *
     * @return a Trigger instance representing the create button's digital
     * signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .create
     */
    fun create(): Trigger = controller.back()

    /**
     * Constructs a Trigger instance around the options button's digital signal.
     *
     * @return a Trigger instance representing the options button's digital
     * signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .options
     */
    fun options(): Trigger = controller.start()

    /**
     * Constructs a Trigger instance around the L3 (left stick) button's digital
     * signal.
     *
     * @return a Trigger instance representing the L3 (left stick) button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .L3
     */
    fun L3(): Trigger = controller.leftStick()

    /**
     * Constructs a Trigger instance around the R3 (right stick) button's
     * digital signal.
     *
     * @return a Trigger instance representing the R3 (right stick) button's
     * digital signal attached to the [default scheduler button loop]
     * [CommandScheduler.getDefaultButtonLoop].
     * @see .R3
     */
    fun R3(): Trigger = controller.rightStick()

    /**
     * Get the X axis value of left side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    val leftX
        get() = controller.leftX

    /**
     * Get the Y axis value of left side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    val leftY
        get() = controller.leftY

    /**
     * Get the X axis value of right side of the controller. Right is positive.
     *
     * @return The axis value.
     */
    val rightX
        get() = controller.rightX

    /**
     * Get the Y axis value of right side of the controller. Back is positive.
     *
     * @return The axis value.
     */
    val rightY
        get() = controller.rightY

    /**
     * Get the left trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    val L2Axis
        get() = controller.leftTriggerAxis

    /**
     * Get the right trigger 2 axis value of the controller. Note that this axis
     * is bound to the range of [0, 1] as opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    val R2Axis
        get() = controller.rightTriggerAxis
}
