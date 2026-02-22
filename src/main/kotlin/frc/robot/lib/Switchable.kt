package frc.robot.lib

fun <T> Boolean.switchable(trueValue: T, falseValue: T): T =
    if (this) trueValue else falseValue
