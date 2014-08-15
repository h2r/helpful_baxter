package baxter_core_msgs;

public interface EndEffectorCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/EndEffectorCommand";
  static final java.lang.String _DEFINITION = "## Command to be sent to an end effector\nuint32 id       # target end effector id\nstring command  # operation to perform\n# Well known commands:\nstring   CMD_NO_OP           = no_op\nstring   CMD_SET             = set\nstring   CMD_CONFIGURE       = configure\nstring   CMD_REBOOT          = reboot\nstring   CMD_RESET           = reset\nstring   CMD_CALIBRATE       = calibrate\nstring   CMD_CLEAR_CALIBRATION = clear_calibration\nstring   CMD_PREPARE_TO_GRIP = prepare_to_grip\nstring   CMD_GRIP            = grip\nstring   CMD_RELEASE         = release\nstring   CMD_GO              = go\nstring   CMD_STOP            = stop\n#\nstring args     # JSON arguments to the command\n#\nstring sender   # optional identifier, returned in state when the command is handled\nuint32 sequence # optional sequence number, return in state when the command is handled\n\n";
  static final java.lang.String CMD_NO_OP = "no_op";
  static final java.lang.String CMD_SET = "set";
  static final java.lang.String CMD_CONFIGURE = "configure";
  static final java.lang.String CMD_REBOOT = "reboot";
  static final java.lang.String CMD_RESET = "reset";
  static final java.lang.String CMD_CALIBRATE = "calibrate";
  static final java.lang.String CMD_CLEAR_CALIBRATION = "clear_calibration";
  static final java.lang.String CMD_PREPARE_TO_GRIP = "prepare_to_grip";
  static final java.lang.String CMD_GRIP = "grip";
  static final java.lang.String CMD_RELEASE = "release";
  static final java.lang.String CMD_GO = "go";
  static final java.lang.String CMD_STOP = "stop";
  int getId();
  void setId(int value);
  java.lang.String getCommand();
  void setCommand(java.lang.String value);
  java.lang.String getArgs();
  void setArgs(java.lang.String value);
  java.lang.String getSender();
  void setSender(java.lang.String value);
  int getSequence();
  void setSequence(int value);
}
