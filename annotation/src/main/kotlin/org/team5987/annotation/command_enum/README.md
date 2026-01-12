# CommandEnum Subsystem Example

## Overview

The `Wrist` subsystem demonstrates how to use an annotated `WristAngles` enum to create clean, type-safe command methods.

## Enum Definition

Define your angles using the `@CommandEnum` annotation:
```kotlin
@CommandEnum
enum class WristAngles(val angle: Angle) {
    CLOSED(0.rot),
    OPEN(15.5.rot),
    DEFAULT(2.4.rot)
}
```

## Subsystem Implementation

Implement the generated actions interface and define the angle-setting behavior:
```kotlin
object Wrist : SubsystemBase(), WristAnglesActions {
    ..
    
    override fun setAngle(value: WristAngles): Command = runOnce({
        setpoint = value.angle
        motor.setControl(positionRequest.withPosition(value.angle))
    })
    
    ..
}
```

## Command binding

Bind the generated command methods to controller buttons.
```kotlin
object RobotContainer {
    ..
    fun configureButtonBindings() {
        controller.a().onTrue(Wrist.closed())
    }
    ..
}
```
