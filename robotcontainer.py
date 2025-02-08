#
# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.
#

import commands2
import commands2.button
import commands2.cmd
from commands2.sysid import SysIdRoutine

from constants import Constants
from subsystems.elevator.elevator import Elevator


class RobotContainer:
    """
    This class is where the bulk of the robot should be declared. Since Command-based is a
    "declarative" paradigm, very little robot logic should actually be handled in the :class:`.Robot`
    periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
    subsystems, commands, and button mappings) should be declared here.
    """

    def __init__(self) -> None:
        self._elevator = Elevator()
        self._joystick = commands2.button.CommandXboxController(
            Constants.OperatorIds.DRIVER_PORT
        )

        # Configure the button bindings
        self.configureButtonBindings()

    def configureButtonBindings(self) -> None:
        """
        Use this method to define your button->command mappings. Buttons can be created by
        instantiating a :GenericHID or one of its subclasses (Joystick or XboxController),
        and then passing it to a JoystickButton.
        """

        self._elevator.setDefaultCommand(
            self._elevator.manual_command(lambda: -self._joystick.getLeftY())
        )
        self._joystick.a().whileTrue(
            self._elevator.sys_id_quasistatic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.b().whileTrue(
            self._elevator.sys_id_quasistatic(SysIdRoutine.Direction.kReverse)
        )
        self._joystick.x().whileTrue(
            self._elevator.sys_id_dynamic(SysIdRoutine.Direction.kForward)
        )
        self._joystick.y().whileTrue(
            self._elevator.sys_id_dynamic(SysIdRoutine.Direction.kReverse)
        )

    def getAutonomousCommand(self) -> commands2.Command:
        """Use this to pass the autonomous command to the main {@link Robot} class.

        :returns: the command to run in autonomous
        """
        return commands2.cmd.print_("No autonomous command configured")
