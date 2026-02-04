package frc.robot.sim

import edu.wpi.first.math.geometry.Rotation3d
import edu.wpi.first.math.geometry.Transform3d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import frc.robot.IS_SIM
import frc.robot.drive
import frc.robot.lib.extensions.cm
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.toPose
import frc.robot.lib.extensions.toPose3d
import frc.robot.lib.extensions.toTranslation3d
import frc.robot.lib.getTranslation2d
import frc.robot.subsystems.sensors.Sensors
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

private val BALL_POSES_RELATIVE_TO_ROBOT = arrayOf(
    getTranslation2d(5.cm, 0.cm),
    getTranslation2d(25.cm, 0.cm),
    getTranslation2d(-20.cm, 0.cm),
    getTranslation2d(5.cm, (-20).cm),
    getTranslation2d(25.cm, (-20).cm),
    getTranslation2d(-20.cm,-20.cm)

)
@LoggedOutput(LogLevel.COMP, key = "MapleSim/EMPTY")
val empty
    get()= arrayOf(0)

@LoggedOutput(LogLevel.COMP, key = "MapleSim/fuel15")
val fuel15
    get() = BALL_POSES_RELATIVE_TO_ROBOT.map {
    it.toPose().toPose3d()+(
        Transform3d(
            drive.pose.translation.toTranslation3d(0.2.m),
            Rotation3d()
        )
    )
}.toTypedArray()

@LoggedOutput(LogLevel.COMP, key = "MapleSim/fuel30")
val fuel30
    get() = BALL_POSES_RELATIVE_TO_ROBOT.map {
        it.toPose().toPose3d()+(
            Transform3d(
                drive.pose.translation.toTranslation3d(0.4.m),
                Rotation3d()
            )
        )
    }.toTypedArray()+(BALL_POSES_RELATIVE_TO_ROBOT.map {
        it.toPose().toPose3d()+(
            Transform3d(
                drive.pose.translation.toTranslation3d(0.2.m),
                Rotation3d()
            )
        )
    }.toTypedArray())

@LoggedOutput(LogLevel.COMP, key = "MapleSim/fuel50")
val fuel50
    get() = BALL_POSES_RELATIVE_TO_ROBOT.map {
        it.toPose().toPose3d()+(
            Transform3d(
                drive.pose.translation.toTranslation3d(0.4.m),
                Rotation3d()
            )
        )
    }.toTypedArray()+(BALL_POSES_RELATIVE_TO_ROBOT.map {
        it.toPose().toPose3d()+(
            Transform3d(
                drive.pose.translation.toTranslation3d(0.2.m),
                Rotation3d()
            )
        )
    }.toTypedArray())+BALL_POSES_RELATIVE_TO_ROBOT.map {
        it.toPose().toPose3d()+(
            Transform3d(
                drive.pose.translation.toTranslation3d(0.6.m),
                Rotation3d()
            )
        )
    }.toTypedArray()



fun HopperFuel(): Command {
    return Commands.runOnce({
        if (Sensors.isHalfFull.asBoolean == false) {
        fuel15
    }
    else if (Sensors.isHalfFull.asBoolean.and(Sensors.isFull.asBoolean==false )){
        fuel30
    }
    if (Sensors.isFull.asBoolean){
        fuel50
    }
    })


}

