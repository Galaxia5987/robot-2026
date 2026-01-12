package frc.robot.lib.controllers;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.lib.Utils;

/*
This class contains the Dieter controller.
This controller is a PID controller with an additional term that
acts as a bang-bang controller,which operates at a certain dead zone.
 */
public class DieterController extends PIDController {

    // The Dieter constant
    private double kDieter;
    // The dead zone of the bang-bang controller
    private double dieterBand = 0;

    /**
     * Constructor for DieterController.
     *
     * @param kp The proportional gain.
     * @param ki The integral gain.
     * @param kd The derivative gain.
     * @param kDieter The Dieter constant.
     * @param period The period of the controller.
     */
    public DieterController(double kp, double ki, double kd, double kDieter, double period) {
        super(kp, ki, kd, period);
        this.kDieter = kDieter;
    }

    public DieterController(double kp, double ki, double kd, double kDieter) {
        super(kp, ki, kd);
        this.kDieter = kDieter;
    }

    /**
     * Updates the Dieter constant.
     *
     * @param kDieter The new Dieter constant.
     */
    public void setDieter(double kDieter) {
        this.kDieter = kDieter;
    }

    /**
     * Updates the PIDF constants.
     *
     * @param kP The new proportional gain.
     * @param kI The new integral gain.
     * @param kD The new derivative gain.
     * @param kDieter The new Dieter constant.
     */
    public void setPIDF(double kP, double kI, double kD, double kDieter) {
        setPID(kP, kI, kD);
        setDieter(kDieter);
    }

    /**
     * Updates the dead zone of the bang-bang controller.
     *
     * @param dieterBand The new dead zone.
     */
    public void setDieterBand(double dieterBand) {
        this.dieterBand = dieterBand;
    }

    @Override
    public double calculate(double measurement, double setpoint) {
        setSetpoint(setpoint);
        return this.calculate(measurement);
    }

    @Override
    public double calculate(double measurement) {
        double val = super.calculate(measurement);
        if (!Utils.epsilonEquals(getSetpoint(), measurement, dieterBand)) {
            val += Math.copySign(kDieter, val);
        }
        return val;
    }
}
