package frc.robot;

public class MainAuto2903{
    private boolean autoFinished;
    public MainAuto2903(){
        init();
    }

    public void run(){
        double startTime = System.currentTimeMillis();
        double driveTime = 500; //milliseconds
        while (System.currentTimeMillis() < startTime + driveTime) {
            Robot.swerveDriveSubsystem.TankDrive(1, 1);
        }
        Robot.swerveDriveSubsystem.stopDrive();
        autoFinished = true;
    }

    public void init(){
        autoFinished = false;
    }

    public boolean isFinished(){
        return autoFinished;
    }
}