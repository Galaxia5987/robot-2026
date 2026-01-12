package frc.robot.lib.motors;

import edu.wpi.first.math.system.plant.DCMotor;

public enum TalonType {
    FALCON,
    FALCON_FOC,
    KRAKEN,
    KRAKEN_FOC;

    static DCMotor getDCMotor(TalonType motorType, int numMotors) {
        return switch (motorType) {
            case FALCON -> DCMotor.getFalcon500(numMotors);
            case FALCON_FOC -> DCMotor.getFalcon500Foc(numMotors);
            case KRAKEN -> DCMotor.getKrakenX60(numMotors);
            case KRAKEN_FOC -> DCMotor.getKrakenX60Foc(numMotors);
        };
    }
}
