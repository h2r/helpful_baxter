package baxter_core_msgs;

public interface AssemblyState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/AssemblyState";
  static final java.lang.String _DEFINITION = "bool enabled             # true if enabled\nbool stopped             # true if stopped -- e-stop asserted\nbool error               # true if a component of the assembly has an error\n#\n# The following are specific to the robot top-level assembly:\nuint8  estop_button      # One of the following:\n  uint8   ESTOP_BUTTON_UNPRESSED = 0   # Robot is not stopped and button is not pressed\n  uint8   ESTOP_BUTTON_PRESSED   = 1\n  uint8   ESTOP_BUTTON_UNKNOWN   = 2   # STATE_UNKNOWN when estop was asserted by a non-user source\n  uint8   ESTOP_BUTTON_RELEASED  = 3   # Was pressed, is now known to be released, but robot is still stopped.\n#\nuint8  estop_source      # If stopped is true, the source of the e-stop.  One of the following:\n  uint8  ESTOP_SOURCE_NONE      = 0   # e-stop is not asserted\n  uint8  ESTOP_SOURCE_USER      = 1   # e-stop source is user input (the red button)\n  uint8  ESTOP_SOURCE_UNKNOWN   = 2   # e-stop source is unknown\n  uint8  ESTOP_SOURCE_FAULT     = 3   # MotorController asserted e-stop in response to a joint fault\n  uint8  ESTOP_SOURCE_BRAIN     = 4   # MotorController asserted e-stop in response to a lapse of the brain heartbeat\n";
  static final byte ESTOP_BUTTON_UNPRESSED = 0;
  static final byte ESTOP_BUTTON_PRESSED = 1;
  static final byte ESTOP_BUTTON_UNKNOWN = 2;
  static final byte ESTOP_BUTTON_RELEASED = 3;
  static final byte ESTOP_SOURCE_NONE = 0;
  static final byte ESTOP_SOURCE_USER = 1;
  static final byte ESTOP_SOURCE_UNKNOWN = 2;
  static final byte ESTOP_SOURCE_FAULT = 3;
  static final byte ESTOP_SOURCE_BRAIN = 4;
  boolean getEnabled();
  void setEnabled(boolean value);
  boolean getStopped();
  void setStopped(boolean value);
  boolean getError();
  void setError(boolean value);
  byte getEstopButton();
  void setEstopButton(byte value);
  byte getEstopSource();
  void setEstopSource(byte value);
}
