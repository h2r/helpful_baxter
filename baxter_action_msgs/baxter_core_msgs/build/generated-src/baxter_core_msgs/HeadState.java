package baxter_core_msgs;

public interface HeadState extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/HeadState";
  static final java.lang.String _DEFINITION = "float32 pan\nbool isPanning\nbool isNodding\n";
  float getPan();
  void setPan(float value);
  boolean getIsPanning();
  void setIsPanning(boolean value);
  boolean getIsNodding();
  void setIsNodding(boolean value);
}
