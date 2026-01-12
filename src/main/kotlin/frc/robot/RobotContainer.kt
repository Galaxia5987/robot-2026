package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Transform2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.wpilibj.RobotController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID
import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.paths.deploy.pathplanner.AutoTest
import frc.robot.autonomous.paths.deploy.pathplanner.BackAutoTest
import frc.robot.autonomous.paths.deploy.pathplanner.ExamplePath
import frc.robot.autonomous.paths.deploy.pathplanner.trysomethingelse
import frc.robot.lib.Mode
import frc.robot.lib.extensions.*
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.onFalse
import frc.robot.lib.extensions.onTrue
import frc.robot.lib.extensions.sec
import frc.robot.lib.extensions.volts
import frc.robot.lib.getTranslation2d
import frc.robot.lib.shooting.toggleCompensation
import frc.robot.lib.sysid.sysId
import frc.robot.lib.unified_controller.UnifiedController
import frc.robot.robotstate.*
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.drive.profiledAlignToPose
import frc.robot.subsystems.roller.Roller
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.hopper.Hopper
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.wrist.Wrist
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.AutoLogOutput
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser
import org.team5987.annotation.LogLevel
import org.team5987.annotation.LoggedOutput

object RobotContainer {
    private val driverController = UnifiedController(0)
    private val switchController = CommandGenericHID(1)
    private val userButton = Trigger { RobotController.getUserButton() }
    private val autoChooser: LoggedDashboardChooser<Command>

    @LoggedOutput(LogLevel.COMP, path = COMMAND_NAME_PREFIX)
    val shouldShootOneBall: Trigger =
        switchController.button(SwitchInput.ShouldShootOneBall.buttonId)

    @LoggedOutput(LogLevel.COMP, path = COMMAND_NAME_PREFIX)
    val forceShoot: Trigger = driverController.triangle()

    enum class SwitchInput(val buttonId: Int) {
        DisableAutoAlign(0),
        StaticSetpoint(1),
        IntakeByVision(2),
        ShouldShootOneBall(12)
    }

    init {
        drive // Ensure Drive is initialized
        autoChooser =
            LoggedDashboardChooser(
                "Auto Choices",
                AutoBuilder.buildAutoChooser()
            )

        registerAutoCommands()
        configureButtonBindings()
        configureDefaultCommands()
        bindRobotCommands()

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance().resetFieldForAuto()
        }

        enableAutoLogOutputFor(this)
    }

    @AutoLogOutput(key = "MapleSimPose")
    private fun getMapleSimPose(): Pose2d? =
        driveSimulation?.simulatedDriveTrainPose

    private fun configureDefaultCommands() {

        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 0.8 }
            )

        Turret.defaultCommand = Turret.setAngle { turretAngleToHub }
        Hood.defaultCommand = hoodDefaultCommand()
        Wrist.defaultCommand = Wrist.open()
    }

    private fun configureButtonBindings() {
        driverController.povUp().onTrue(AutoTest())
        driverController.povLeft().onTrue(trysomethingelse())

        driverController
            .povDown()
            .onTrue(
                drive.defer {
                    profiledAlignToPose(
                        drive.pose +
                            Transform2d(
                                getTranslation2d(x = 3.m),
                                Rotation2d.kCCW_90deg
                            )
                    )
                }
            )

        driverController
            .square()
            .onTrue(
                Commands.runOnce({
                    AutoBuilder.resetOdom(
                        PathPlannerPath.fromPathFile(
                                autoChooser.sendableChooser.selected
                            )
                            .pathPoses[0]
                    )
                })
            )
        // reset swerve
        driverController.apply {
            options().onTrue(DriveCommands.resetGyro())

            circle().onTrue(setIntaking())
            L2()
                .onTrue(Roller.intake(), Hopper.startShoot())
                .onFalse(Roller.stop(), Hopper.stop())
            R2()
                .onTrue(Roller.outtake(), Hopper.outtake())
                .onFalse(Roller.stop(), Hopper.stop())
            cross().onTrue(setShooting())
            povDown().onTrue(setIdling())
            povUp().onTrue(toggleCompensation())

            L1().onTrue(alignToBall())
            povRight()
                .onTrue(
                    drive.defer {
                        profiledAlignToPose(
                            drive.pose +
                                Transform2d(
                                    Translation2d(3.0, 0.0),
                                    Rotation2d.kCCW_90deg
                                )
                        )
                    }
                )

            create().whileTrue(Wrist.reset())
        }

        userButton.onTrue(Turret.reset().ignoringDisable(true))

        switchController.apply {
            button(SwitchInput.DisableAutoAlign.buttonId)
                .whileTrue(disableAutoAlign())
                .whileFalse(enableAutoAlign())

            button(SwitchInput.StaticSetpoint.buttonId)
                .whileTrue(setStaticShooting())
                .whileFalse(setShooting())

            button(SwitchInput.IntakeByVision.buttonId)
                .whileTrue(setIntakeByVision())
                .onFalse(stopIntakeByVision())
        }
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        // SysIds
        autoChooser.addDefaultOption("AutoTest2", AutoTest())
        autoChooser.addOption("AutoTest", AutoTest())
        autoChooser.addOption("BackAutoTest", BackAutoTest())

        autoChooser.addOption(
            "Drive Wheel Radius Characterization",
            DriveCommands.wheelRadiusCharacterization()
        )
        autoChooser.addOption(
            "Drive Simple FF Characterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Forward)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Quasistatic Reverse)",
            drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Forward)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kForward)
        )
        autoChooser.addOption(
            "Drive SysId (Dynamic Reverse)",
            drive.sysIdDynamic(SysIdRoutine.Direction.kReverse)
        )

        autoChooser.addOption(
            "swerveFFCharacterization",
            DriveCommands.feedforwardCharacterization()
        )
        autoChooser.addOption(
            "hoodSysId",
            Hood.sysId()
                .withForwardRoutineConfig(1.8.volts.per(sec), 1.volts, 0.75.sec)
                .withBackwardRoutineConfig(
                    1.volts.per(sec),
                    0.8.volts,
                    0.75.sec
                )
                .command()
        )

        autoChooser.addOption(
            "Turret SysId",
            Turret.sysId()
                .withForwardRoutineConfig(1.volts.per(sec), 2.volts, 2.2.sec)
                .withBackwardRoutineConfig(1.volts.per(sec), 2.volts, 2.2.sec)
                .command()
        )

        // Autos
        autoChooser.addOption("TestPath", ExamplePath())
    }

    fun resetSimulationField() {
        if (CURRENT_MODE != Mode.SIM) return

        drive.resetOdometry(Pose2d(3.0, 3.0, Rotation2d()))
        SimulatedArena.getInstance().resetFieldForAuto()
    }
}
