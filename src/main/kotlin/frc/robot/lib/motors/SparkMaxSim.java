package frc.robot.lib.motors;

import com.revrobotics.spark.SparkBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;

public class SparkMaxSim extends SimMotor {

    public SparkMaxSim(
            int numMotors, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(DCMotor.getNEO(numMotors), jKgMetersSquared, gearing, conversionFactor);
    }

    public SparkMaxSim(
            DCMotor motor, double gearing, double jKgMetersSquared, double conversionFactor) {
        super(motor, jKgMetersSquared, gearing, conversionFactor);
    }

    public SparkMaxSim(
            LinearSystem<N2, N1, N2> model,
            int numMotors,
            double gearing,
            double conversionFactor) {
        super(model, DCMotor.getNEO(numMotors), gearing, conversionFactor);
    }

    public void set(double speed) {
        setInputVoltage(speed * 12.0);
    }

    public void setInputVoltage(double voltage) {
        voltageRequest = MotorSetpoint.simpleVoltage(voltage);
    }

    public void setReference(double value, SparkBase.ControlType ctrl) {
        setReference(value, ctrl, 0);
    }

    private void setInputVoltage(MotorSetpoint voltage) {
        voltageRequest = voltage;
    }

    public void setReference(double value, SparkBase.ControlType ctrl, double arbFeedforward) {
        switch (ctrl) {
            case kDutyCycle:
                set(value);
                break;
            case kPosition:
                setInputVoltage(() -> controller.calculate(getPosition(), value) + arbFeedforward);
                break;
            case kMAXMotionPositionControl:
                setInputVoltage(
                        () -> profiledController.calculate(getPosition(), value) + arbFeedforward);
                break;
            case kVelocity:
                setInputVoltage(() -> controller.calculate(getVelocity(), value) + arbFeedforward);
                break;
            case kMAXMotionVelocityControl:
                setInputVoltage(
                        () -> profiledController.calculate(getVelocity(), value) + arbFeedforward);
                break;
            case kVoltage:
                setInputVoltage(value);
                break;
            case kCurrent:
                System.out.println("Can't use current control for spark max in sim!");
                break;
        }
    }

    public double getBusVoltage() {
        return voltageRequest.getAsDouble();
    }

    public double getAppliedOutput() {
        return voltageRequest.getAsDouble() / 12.0;
    }

    public double getVelocity() {
        return motorSim.getAngularVelocityRPM() * conversionFactor;
    }

    public double getPosition() {
        return motorSim.getAngularPositionRotations() * conversionFactor;
    }

    public double getOutputCurrent() {
        return motorSim.getCurrentDrawAmps();
    }
}
