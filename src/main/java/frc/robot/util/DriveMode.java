package frc.robot.util;

public enum DriveMode { 
  ARCADE, 
  TANK, 
  PROPORTIONAL,
  SHIFT;

  // gets the next enum value, effectively cycling through them. thx stackoverflow
  private static DriveMode[] vals = values();
  public DriveMode next()
  {
    return vals[(this.ordinal()+1) % vals.length];
  }
  public String toString() {
    switch (this) {
      case ARCADE:
        return "Arcade Drive";
      case PROPORTIONAL:
        return "Proportional Drive";
      case TANK:
        return "Tank Drive";
      case SHIFT:
        return "Shift Drive";
      default:
        return "Error";
    }
  }
}