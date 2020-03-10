package frc.robot;

public class MainAuto2903{
    private boolean autoFinished;
    final double TURN_ERROR = 3; // degree
    final double PIXEL_HEIGTH = 1080;
    final double PIXEL_WIDTH = 1920;
    final double PIXEL_ERROR = 5;
    boolean isFinished = false;
    int cellCount = 0;
    public MainAuto2903(){
        init();
    }

    public void run(){
        // double startTime = System.currentTimeMillis();
        // double driveTime = 500; //milliseconds
        // while (System.currentTimeMillis() < startTime + driveTime) {
        //     Robot.swerveDriveSubsystem.TankDrive(1, 1);
        // }
        // Robot.swerveDriveSubsystem.stopDrive();

        Robot.swerveDriveSubsystem.swerveDriveDistance(1, 180, false, 1.5);
        if(cellCount == 0)Robot.swerveDriveSubsystem.swerveDriveDistance(1, 180, false, 1.5);
        Robot.limelightSubsystem.setZoomMode();
        while (Robot.limelightSubsystem.getTV() == 0) {
            Robot.swerveDriveSubsystem.swerveDrive(0.5, 180, 1, false);
        }
        while (Math.abs(Robot.limelightSubsystem.getTX()) > TURN_ERROR / 2) {
            if (Robot.limelightSubsystem.getTX() > 0)
                Robot.swerveDriveSubsystem.swerveDrive(0.25, 180, 1, false);
            else
                Robot.swerveDriveSubsystem.swerveDrive(0.25, 180, -1, false);
        }

        Robot.shooterSubsystem.shooting(Robot.lidarSubsystem.getDistance(), 1, false);

        for (int i = 0; i < 3; ++i) {
            Robot.shooterSubsystem.waitForTargetSpeed();
            double lastSpeed = Robot.shooterSubsystem.getCurrentSpeed();
            while (lastSpeed - Robot.shooterSubsystem.getCurrentSpeed() < 0.2) {
                lastSpeed = Robot.shooterSubsystem.getCurrentSpeed();
                Robot.shooterSubsystem.intake(1);
            }
            Robot.shooterSubsystem.intake(0);
        }
        
        cellCount = 0;
        boolean isCellAtSensor = false;
        while(cellCount < 3){
            if(Robot.tensorTable.getEntry("EnergyCellCount").getDouble(0) > 0){
                if(Robot.tensorTable.getEntry("EnergyCellX").getDouble(PIXEL_WIDTH/2) > (PIXEL_WIDTH/2)+PIXEL_ERROR)
                Robot.swerveDriveSubsystem.swerveDrive(0.5, 0, 0.5, false);
                else if(Robot.tensorTable.getEntry("EnergyCellX").getDouble(PIXEL_WIDTH/2) < (PIXEL_WIDTH/2)-PIXEL_ERROR)
                Robot.swerveDriveSubsystem.swerveDrive(0.5, 0, -0.5, false);
                
            }
            if(Robot.shooterSubsystem.intakeDetect())isCellAtSensor = true;
            if(!Robot.shooterSubsystem.intakeDetect() && isCellAtSensor){
                isCellAtSensor = false;
                cellCount++;
            }
                
        }
        
        Robot.limelightSubsystem.setLight(false);
        

        autoFinished = true;
    }

    public void init(){
        autoFinished = false;
    }

    public boolean isFinished(){
        return autoFinished;
    }
}