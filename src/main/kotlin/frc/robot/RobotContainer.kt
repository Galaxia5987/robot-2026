package frc.robot

import choreo.Choreo
import choreo.auto.AutoFactory
import choreo.trajectory.SwerveSample
import choreo.trajectory.Trajectory
import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Pose2d
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.Outpost
import frc.robot.autonomous.StartSomethingNew
import frc.robot.autonomous.runPath
import frc.robot.lib.IS_RED
import frc.robot.lib.Mode
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.getFileNameFromStack
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.intaking.IntakingTriggers
import frc.robot.states.intaking.IntakingTriggers.canCloseIntake
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.shooting.Shooting
import frc.robot.subsystems.drive.AutoHeadingController
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import java.util.Optional
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.Logger
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

fun log_trajectory(sample: Trajectory<SwerveSample>, isPathStarted: Boolean) {
    Logger.recordOutput("Choreo/Trajectory", *sample.poses)
    Logger.recordOutput("Choreo/IsStarting", isPathStarted)
}

object RobotContainer {
    private val driverController = CommandXboxController(0)
    private val autoChooser: LoggedDashboardChooser<Command>
    fun autoFactory(): AutoFactory {
        return AutoFactory(
            drive::getPose,
            drive::resetOdometry,
            drive::followTrajectory,
            true,
            drive,
            ::log_trajectory
        )
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

        if (CURRENT_MODE == Mode.SIM) {
            SimulatedArena.getInstance()
                .addDriveTrainSimulation(driveSimulation)
            SimulatedArena.getInstance().resetFieldForAuto()
            AutoHeadingController.enableContinuousInput(-Math.PI, Math.PI)
        }

        enableAutoLogOutputFor(this)
    }

    private fun configureDefaultCommands() {
        drive.defaultCommand =
            DriveCommands.joystickDrive(
                { -driverController.leftY },
                { -driverController.leftX },
                { -driverController.rightX * 0.8 }
            )
        Turret.defaultCommand = Turret.setAngle { Turret.aimingSetpoint() }
        Hood.defaultCommand = Hood.setAngle { Hood.aimingSetpoint() }
    }

    private fun configureButtonBindings() {
        //        driverController.b().onTrue(
        //                Commands.runOnce({ // Add the projectile to the simulated arena
        //                    SimulatedArena.getInstance()
        //                        .addGamePieceProjectile(
        //                            MapleSimShooter.createFuelOnFly()
        //                        )
        //                })
        //            )

        // Intake Bindings
        //        val intakeButton = driverController.R2()
        driverController.rightTrigger().onTrue(IntakingStates.INTAKING.set())
        driverController
            .rightTrigger()
            .negate()
            .and(canCloseIntake)
            .onTrue(IntakingStates.CLOSED.set())
        driverController
            .rightTrigger()
            .negate()
            .and(IntakingTriggers.cantCloseIntake)
            .onTrue(IntakingStates.OPEN.set())

        Shooting(driverController.leftTrigger())
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    @Override
    fun autonomousInit() {
        if (Choreo.loadTrajectory(getFileNameFromStack()).isPresent) {
            val initialPose: Optional<Pose2d> =
                Choreo.loadTrajectory(getFileNameFromStack())
                    .get()
                    .getInitialPose(IS_RED)
            if (initialPose.isPresent) {
                drive.resetOdometry(initialPose.get())
            }
        }
    }
    fun registerAutoCommands() {
        autoChooser.addOption("Outpost", Outpost())
        autoChooser.addOption("StartSomethingNew", StartSomethingNew())
        //        autoChooser.addOption(
        //            "bumbIntoDepot",
        //            depotDoubleCycle()
        //        )
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
            "Test1",
            runPath("Test1")
        )
    }
}
