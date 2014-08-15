package baxter_core_msgs;

public interface RobustControllerStatus extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/RobustControllerStatus";
  static final java.lang.String _DEFINITION = "# True if the RC is enabled and running, false if not.\nbool isEnabled\n\n# The state of the RC with respect to its completion goal.  One of\n# NOT_COMPLETE, COMPLETE_W_FAILURE, or COMPLETE_W_SUCCESS\nint32 complete\nint32 NOT_COMPLETE = 0\nint32 COMPLETE_W_FAILURE = 1\nint32 COMPLETE_W_SUCCESS = 2\n\n# Identifies the sender of the Enable message that the RC is using for its\n# commands.  This should correspond to the \"uid\" field of a recently published\n# RC *Enable message.\nstring controlUid\n\n# Set to true when the RC self-disables as a result of too much time elapsing\n# without receiving an Enable message.\nbool timedOut\n\n# A list of relevant error codes.  Error codes are defined by the individual\n# robust controllers, consult a robust controller\'s documentation to see what\n# error codes it generates.\nstring[] errorCodes\n\n# A list of current labels for the RC\'s current state. For example, \"fastapproach\",\n# \"slowapproach\", etc. Used primarily for the blended RCs, other RCs can leave this\n# blank. This will probably contains just one label, but it could contain multiple labels\n# in the future.\nstring[] labels\n";
  static final int NOT_COMPLETE = 0;
  static final int COMPLETE_W_FAILURE = 1;
  static final int COMPLETE_W_SUCCESS = 2;
  boolean getIsEnabled();
  void setIsEnabled(boolean value);
  int getComplete();
  void setComplete(int value);
  java.lang.String getControlUid();
  void setControlUid(java.lang.String value);
  boolean getTimedOut();
  void setTimedOut(boolean value);
  java.util.List<java.lang.String> getErrorCodes();
  void setErrorCodes(java.util.List<java.lang.String> value);
  java.util.List<java.lang.String> getLabels();
  void setLabels(java.util.List<java.lang.String> value);
}
