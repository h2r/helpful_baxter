package baxter_core_msgs;

public interface ListCamerasResponse extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/ListCamerasResponse";
  static final java.lang.String _DEFINITION = "string[]    cameras";
  java.util.List<java.lang.String> getCameras();
  void setCameras(java.util.List<java.lang.String> value);
}
