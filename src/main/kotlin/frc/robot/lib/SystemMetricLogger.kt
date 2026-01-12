package frc.robot.lib

import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val rt = Runtime.getRuntime()
private const val MB = 1024.0 * 1024.0

@LoggedOutput(LogLevel.DEBUG, path = "SystemMetric")
val freeRAMMb: Double
    get() = rt.freeMemory() / MB

@LoggedOutput(LogLevel.DEBUG, path = "SystemMetric")
val totalRAMMb: Double
    get() = rt.totalMemory() / MB

@LoggedOutput(LogLevel.DEBUG, path = "SystemMetric")
val usedRAMMb: Double
    get() = totalRAMMb - freeRAMMb
