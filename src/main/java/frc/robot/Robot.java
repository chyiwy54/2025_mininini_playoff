package frc.robot;

import org.littletonrobotics.urcl.URCL;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

@Logged
public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;

    @Override
    public void robotInit() {
        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        URCL.start();
        URCL.start(DataLogManager.getLog());
        m_robotContainer = new RobotContainer();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotPeriodic() {
        SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
        double batteryVoltage = RobotController.getBatteryVoltage();
        SmartDashboard.putNumber("Battery Voltage", batteryVoltage);
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {

    }

    @Override
    public void disabledPeriodic() {

    }

    /**
     * This autonomous runs the autonomous command selected by your
     * {@link RobotContainer} class.
     */

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel(); 
            
        }
    }

    @Override
    public void teleopPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }
}
