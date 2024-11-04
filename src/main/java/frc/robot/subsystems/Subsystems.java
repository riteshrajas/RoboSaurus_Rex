package frc.robot.subsystems;

public enum Subsystems {
    SWERVE_DRIVE("Swerve Drive", "This Subsystem is responsible for controlling the Swerve Drive"),
    INTAKE("Intake", "This Subsystem is responsible for controlling the Intake"),
    SHOOTER("Shooter", "This Subsystem is responsible for controlling the Shooter"),
    CLIMBER("Climber", "This Subsystem is responsible for controlling the Climber"),
    CONTROL_PANEL("Control Panel", "This Subsystem is responsible for controlling the Control Panel"),
    DRIVEBASE("Drivebase", "This Subsystem is responsible for controlling the Drivebase"),
    LIMELIGHT("Limelight", "This Subsystem is responsible for controlling the Limelight"),
    PNEUMATICS("Pneumatics", "This Subsystem is responsible for controlling the Pneumatics"),
    TURRET("Turret", "This Subsystem is responsible for controlling the Turret"),
    VISION("Vision", "This Subsystem is responsible for controlling the Vision"),
    WHEEL_OF_FORTUNE("Wheel of Fortune", "This Subsystem is responsible for controlling the Wheel of Fortune"),
    ELEVATOR("Elevator", "This Subsystem is responsible for controlling the Elevator"),
    TESTER("Tester", "This Subsystem is responsible for controlling the Texter");

    private final String ntName;
    private final String description;

    Subsystems(String ntName, String description) {
        this.ntName = ntName;
        this.description = description;
    }

    public String getNetworkTable() {
        return ntName;
    }

    public String getDescription() {
        return description;
    }

    public SubsystemABS getInstance() {
        switch (this) {
            case SWERVE_DRIVE:
                return SWERVE_DRIVE.getInstance();
            case ELEVATOR:
                return ELEVATOR.getInstance();
            case TURRET:
                return TURRET.getInstance();
            case VISION:
                return VISION.getInstance();
            // Add other subsystems as needed
            default:
                return null;
        }
    }
}