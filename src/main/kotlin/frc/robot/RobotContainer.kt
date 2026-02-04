package frc.robot

import com.pathplanner.lib.auto.AutoBuilder
import edu.wpi.first.math.geometry.Rotation2d
import edu.wpi.first.math.geometry.Translation2d
import edu.wpi.first.math.kinematics.ChassisSpeeds
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.Commands
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine
import frc.robot.lib.Mode
import frc.robot.lib.extensions.deg
import frc.robot.lib.extensions.enableAutoLogOutputFor
import frc.robot.lib.extensions.m
import frc.robot.lib.extensions.mps
import frc.robot.lib.unified_controller.PS5LinuxController
import frc.robot.states.intaking.IntakingStates
import frc.robot.states.intaking.IntakingTriggers.canCloseIntake
import frc.robot.states.intaking.IntakingTriggers.cantCloseIntake
import frc.robot.states.shooting.Shooting
import frc.robot.subsystems.drive.DriveCommands
import frc.robot.subsystems.shooter.turret.Turret
import frc.robot.subsystems.shooter.turret.Turret.setAngle
import frc.robot.subsystems.shooter.turret.turretAngleToHub
import org.ironmaple.simulation.SimulatedArena
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser

object RobotContainer {
    private val driverController = PS5LinuxController(0)
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
        Turret.defaultCommand = setAngle { turretAngleToHub }
    }

    private fun configureButtonBindings() {
        driverController
            .cross()
            .onTrue(
                Commands.runOnce({ // Add the projectile to the simulated arena
                    SimulatedArena.getInstance()
                        .addGamePieceProjectile(
                            RebuiltFuelOnFly(
                                drive.pose.translation,
                                Translation2d(),
                                ChassisSpeeds(),
                                Rotation2d(),
                                0.5.m,
                                1.mps,
                                90.deg
                            )
                        )
                })
            )

        // Intake Bindings
        val intakeButton = driverController.R2()

        intakeButton.onTrue(IntakingStates.INTAKING.set().also {})
        intakeButton
            .negate()
            .and(canCloseIntake)
            .onTrue(IntakingStates.CLOSED.set())
        intakeButton
            .negate()
            .and(cantCloseIntake)
            .onTrue(IntakingStates.OPEN.set())

        Shooting(driverController.L2())
    }

    fun getAutonomousCommand(): Command = autoChooser.get()

    private fun registerAutoCommands() {
        // SysIds
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
