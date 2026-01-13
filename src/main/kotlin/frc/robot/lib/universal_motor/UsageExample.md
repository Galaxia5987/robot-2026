# Universal Motor Subsystem Examples

This document demonstrates how to create subsystems using the `UniversalMotor` class.

---

## Elevator Example

The `Elevator` subsystem uses a `UniversalMotor` with a diameter and gear ratio to control position using closed-loop feedback:

```kotlin
private const val GEAR_RATIO = 1 / 10.0
private val DIAMETER = 0.06.meters

class Elevator : SubsystemBase() {
    @AutoLogOutput
    private var setPoint: Distance = m.zero()

    private val motor = UniversalMotor(
        port = 11,
        momentOfInertia = (0.003).kilogramSquareMeters,
        config = TalonFXConfiguration().withSlot0(
            Slot0Configs().withKP(3.0)
        ),
        gearRatio = GEAR_RATIO,
        diameter = DIAMETER
    )

    private val positionRequest = PositionVoltage(0.0)

    fun setPosition(position: Distance): Command = this.runOnce {
        setPoint = position
        motor.setControl(positionRequest.withPosition(position.toAngle(DIAMETER, GEAR_RATIO)))
    }

    override fun periodic() {
        motor.updateInputs()
        Logger.processInputs("Elevator", motor.inputs)
    }
}
```

## Gripper Example

The `Gripper` subsystem uses a `UniversalMotor` in order to control the voltage:

```kotlin
private val GRIPPER_INTAKE_VOLTAGE = 3.0.volts

class Gripper : SubsystemBase() {
    private val motor = UniversalMotor(
        port = 0,
        momentOfInertia = (0.003).kilogramSquareMeters,
    )

    private val voltageRequest = VoltageOut(0.0)

    private fun setVoltage(voltage: Voltage): Command = this.runOnce {
        motor.setControl(voltageRequest.withOutput(voltage))
    }

    fun intake(): Command = setVoltage(GRIPPER_INTAKE_VOLTAGE)
    fun outtake(): Command = setVoltage(-GRIPPER_INTAKE_VOLTAGE)
    fun stop(): Command = setVoltage(volts.zero())

    override fun periodic() {
        motor.updateInputs()
    }
}
```

