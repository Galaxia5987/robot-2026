# SysIdCommand Factory

`SysIdCommand` is a Kotlin-based helper for characterizing FRC subsystems using WPILib's SysId framework. It simplifies the creation of dynamic and quasistatic tests for any subsystem that implements the `SysIdable` interface and extends `SubsystemBase`.

---
## Usage

### Implement `SysIdable`

To use `SysIdCommand`, your subsystem must implement the `SysIdable` interface, which requires a method to apply voltage to the mechanism.

```kotlin
class Wrist : SubsystemBase(), SysIdable {
    override fun setVoltage(voltage: Voltage) {
        // Apply voltage to motors
    }
}
````

### Create and Configure a `SysIdCommand`

Use the `sysId()` extension function to generate a `SysIdCommand` and configure it with forward and backward routines.

```kotlin
val sysIdCommand = wrist
    .sysId()
    .withForwardRoutineConfig(/* ... */)
    .withBackwardRoutineConfig(/* ... */)
    .command()
```
---

## Examples

### Example #1 - Wrist
This example demonstrates how to implement SysIdable for a mechanism using the Universal TalonFX motor. It defines voltage control for system identification and includes additional functionality for position control using Motion Magic. The configuration is fully parameterized in `WristConstants.kt`, showcasing how to integrate tuning parameters, gearing ratios, and current limits.

`Wrist.kt`
```kotlin
class Wrist : SubsystemBase(), SysIdable {

    private val motor =
        UniversalTalonFX(
            MOTOR_ID,
            config = MOTOR_CONFIG,
            gearRatio = GEAR_RATIO
        )

    private val voltageRequest = VoltageOut(0.0)
    private val positionRequest = MotionMagicTorqueCurrentFOC(0.0)

    override fun setVoltage(voltage: Voltage) {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setPosition(angle: Angle) {
        motor.setControl(positionRequest.withPosition(angle))
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Subsystems/$name", motor.inputs)
    }
}
```

`WristConstants.kt`
```kotlin
const val MOTOR_ID = 15

const val GEAR_RATIO = 1 / ((1.0 / 9.0) * (20.0 / 66.0) * (16.0 / 42.0))

val PID_GAINS = Gains(11.168, 0.0, 0.2, 0.0, 0.047964, 0.038807, 0.1)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = PID_GAINS.toSlotConfig()

        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }

        Feedback =
            FeedbackConfigs().apply {
                RotorToSensorRatio = 1 / ((1.0 / 9.0) * (20.0 / 66.0))
                SensorToMechanismRatio = 1 / (16.0 / 42.0)
            }

        MotionMagic =
            MotionMagicConfigs().apply {
                MotionMagicCruiseVelocity = 30.0
                MotionMagicAcceleration = 50.0
                MotionMagicJerk = 1000.0
            }
    }
```

### Example #2 - Elevator

This example illustrates a dual-motor linear mechanism with one motor following the other. It implements SysIdable by applying voltage control to the main motor while the auxiliary motor follows it. The subsystem also supports position control based on linear distance, automatically converting it to angular units using the sprocket diameter and gear ratio. All configuration and tuning constants are organized in `ElevatorConstants.kt`, providing a clean structure for gains, motion parameters, and current limits.

`Elevator.kt`
```kotlin
class Elevator : SubsystemBase(), SysIdable {

    private val mainMotor =
        UniversalTalonFX(
            MOTOR_ID_MAIN,
            config = MOTOR_CONFIG,
            linearSystemWheelDiameter = SPROCKET_DIAMETER,
            gearRatio = GEAR_RATIO
        )
    private val auxMotor =
        UniversalTalonFX(
            MOTOR_ID_AUX,
            config = MOTOR_CONFIG,
            linearSystemWheelDiameter = SPROCKET_DIAMETER,
            gearRatio = GEAR_RATIO
        )

    private val voltageRequest = VoltageOut(0.0)
    private val positionRequest = MotionMagicTorqueCurrentFOC(0.0)

    init {
        auxMotor.setControl(Follower(MOTOR_ID_MAIN, false))
    }

    override fun setVoltage(voltage: Voltage) {
        mainMotor.setControl(voltageRequest.withOutput(voltage))
    }

    fun setHeight(height: Distance) {
        mainMotor.setControl(
            positionRequest.withPosition(
                height.toAngle(SPROCKET_DIAMETER, GEAR_RATIO)
            )
        )
    }

    override fun periodic() {
        mainMotor.updateInputs()
        Logger.processInputs("Subsystem/$name", mainMotor.inputs)
    }
}
```

`ElevatorConstants.kt`
```kotlin
const val MOTOR_ID_MAIN = 13
const val MOTOR_ID_AUX = 14
val SPROCKET_DIAMETER = 36.4.mm
const val GEAR_RATIO = (12.0 / 72.0) * 2.0

val PID_GAINS = Gains(5.0, 0.0, 0.4, 0.11, 0.021235, 0.002, 0.6)

val MOTOR_CONFIG =
    TalonFXConfiguration().apply {
        Slot0 = PID_GAINS.toSlotConfig()

        MotorOutput =
            MotorOutputConfigs().apply {
                NeutralMode = NeutralModeValue.Brake
                Inverted = InvertedValue.CounterClockwise_Positive
            }

        CurrentLimits =
            CurrentLimitsConfigs().apply {
                StatorCurrentLimitEnable = true
                SupplyCurrentLimitEnable = true
                StatorCurrentLimit = 80.0
                SupplyCurrentLimit = 40.0
            }

        Feedback = FeedbackConfigs().apply { RotorToSensorRatio = 1.0 }

        MotionMagic =
            MotionMagicConfigs().apply {
                MotionMagicCruiseVelocity = 60.0
                MotionMagicAcceleration = 100.0
                MotionMagicJerk = 1000.0
            }
    }
```