package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import com.pathplanner.lib.path.PathConstraints
import com.pathplanner.lib.path.PathPlannerPath
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.button.CommandXboxController
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.autonomous.Test
import frc.robot.field.HUB_LOCATION
import frc.robot.lib.Mode
import frc.robot.lib.extensions.deg_ps
import frc.robot.lib.extensions.deg_ps_ps
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.mps
import frc.robot.lib.extensions.mps_ps
import frc.robot.lib.extensions.toPose
import frc.robot.lib.unified_controller.PS5LinuxController
import frc.robot.sim.MapleSimShooter
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.intaking.IntakingTriggers
import frc.robot.states.intaking.IntakingTriggers.canCloseIntake
import frc.robot.states.intaking.IntakingTriggers.cantCloseIntake
import frc.robot.states.setpoints_manager.aimingSetpoint
import frc.robot.states.shooting.Shooting
import frc.robot.states.shooting.ShootingState
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.shooter.hood.Hood
import frc.robot.subsystems.shooter.turret.Turret
import org.ironmaple.simulation.SimulatedArena
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = CommandXboxController(0)
    private val autoChooser: LoggedDashboardChooser<Command>

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
        driverController.rightTrigger()
            .negate()
            .and(canCloseIntake)
            .onTrue(IntakingStates.CLOSED.set())
        driverController.rightTrigger()
            .negate()
            .and(IntakingTriggers.cantCloseIntake)
            .onTrue(IntakingStates.OPEN.set())

        Shooting(driverController.leftTrigger())
    }

    fun getAutonomousCommand(): Command {
        val path = PathPlannerPath.fromPathFile("StartToFuelDepotSide")
        val startPose = path.pathPoses[0]
        return AutoBuilder.resetOdom(startPose).andThen(AutoBuilder.followPath(path))
//        return AutoBuilder.pathfindToPose(HUB_LOCATION.toPose(), PathConstraints(5.0.mps, 8.4.mps_ps, 360.deg_ps, 720.deg_ps_ps))
    }

    private fun registerAutoCommands() {
        // SysIds
        autoChooser.addOption(
            "Test",
            Test()
        )
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
    }
}
