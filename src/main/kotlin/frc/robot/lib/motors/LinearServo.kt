package frc.robot.lib.motors

import edu.wpi.first.math.MathUtil
import edu.wpi.first.units.measure.Distance
import edu.wpi.first.wpilibj.Servo
import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.button.Trigger
import frc.robot.lib.extensions.get
import frc.robot.lib.extensions.mm

/**
 * Parameters for L16-R Actuonix Linear Actuators
 *
 * @param channel PWM channel used to control the servo
 * @param length max length of the servo [mm]
 * @param speed max speed of the servo [mm/second]
 */
class LinearServo(
    channel: Int,
    length: Int,
    speed: Int,
    positionTolerance: Double
) : Servo(channel) {
    private var speed: Double
    private var length: Double
    private var setpoint = 0.0
    private var position = 0.0
    private var lastTime = 0.0

    /**
     * Miniature Linear Servo Actuators - User Guide (Rev 1) Page 10Table of
     * Contentswcproducts.com Run this method in any periodic function to update
     * the position estimation of your servo
     *
     * @param setpoint the target position of the servo [mm]
     */
    override fun setPosition(setpoint: Double) {
        this.setpoint = MathUtil.clamp(setpoint, 0.0, length)
        setSpeed((this.setpoint / length * 2.0) - 1.0)
    }

    fun setPosition(setpoint: Distance) {
        setPosition(setpoint[mm])
    }

    init {
        setBoundsMicroseconds(2000, 0, 0, 0, 1000)
        this.length = length.toDouble()
        this.speed = speed.toDouble()
    }

    /**
     * Run this method in any periodic function to update the position
     * estimation of your servo
     */
    fun updatePosition() {
        val dt = Timer.getFPGATimestamp() - lastTime
        if (position > setpoint + speed * dt) {
            position -= speed * dt
        } else if (position < setpoint - speed * dt) {
            position += speed * dt
        } else {
            position = setpoint
        }
    }

    /**
     * Current position of the servo, must be calling
     * [updatePosition()][.updatePosition] periodically
     *
     * @return Servo Position [mm]
     */
    override fun getPosition(): Double {
        return position
    }

    /**
     * Checks if the servo is at its target position, must be calling
     * [updatePosition()][.updatePosition] periodically
     * @return true when servo is at its target
     */
    val reachedSetpoint = Trigger {
        MathUtil.isNear(setpoint, position, positionTolerance)
    }
}
