package baxter_core_msgs;

public interface AnalogOutputCommand extends org.ros.internal.message.Message {
  static final java.lang.String _TYPE = "baxter_core_msgs/AnalogOutputCommand";
  static final java.lang.String _DEFINITION = "##the name of the output\nstring name  \n##the value to set output \nuint16 value   \n";
  java.lang.String getName();
  void setName(java.lang.String value);
  short getValue();
  void setValue(short value);
}
