package frc.robot.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Subsystem

private val OuterStackTrace
    get() = Throwable().stackTrace[2]

fun Subsystem.namedRun(
    commandName: String = OuterStackTrace.methodName,
    action: () -> Unit
): Command = run(action).withName("$name/$commandName")

fun Subsystem.namedRunOnce(
    commandName: String = OuterStackTrace.methodName,
    action: () -> Unit
) = runOnce(action).withName("$name/$commandName")

fun Command.named(
    prefixName: String = OuterStackTrace.className.substringAfterLast('.'),
    commandName: String = OuterStackTrace.methodName
): Command {
    if (this.requirements.size == 1) {
        return this.withName("${this.requirements.first().name}/$commandName")
    }
    return this.withName("$prefixName/$commandName")
}
