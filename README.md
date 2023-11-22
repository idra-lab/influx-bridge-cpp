# InfluxDB Bridge

The goal of this ROS2 package is to provide a simple interface to log data coming from
different topics and upload them into a [Influx Database](https://www.influxdata.com/).

## Usage

The library exposes the `influxdb-bridge` executable that can be used to log data coming
from particular topics.

To properly work it's necessary to setup in this folder (the source directory of the
package) at least the mandatory `config.txt` file as follows:

```
path/to/workspace/src/influxdb-bridge-cpp
├── config.txt [mandatory]
├── fields.txt [optional]
├── points.txt [automatically generated]
├── CMakeLists.txt
├── include
├── launch
├── package.xml
├── README.md
└── src
```

The `config.txt` must contain a list of lines consisting of `<topic> <msg_type> <measure_name (optional)>`, e.g.

```
/turtle1/pose turtlesim/Pose
/coppelia_joint_states sensor_msgs/JointState RobState
```

The `fields.txt` file is instead optional and can contain a list of extra fields that
will be applied to each collected measurement; fields are specified with the format
`<field_key> = <field_value>`, e.g.

```
robot = ur5e
test=2
```


## Adding support for new messages

To add suppport for a new message type follow this steps:

- Make sure that to list the _optional_ dependency on the package that contains the
  message definition by checking (and eventually adding) the [CMakeLists file](./CMakeLists.txt):
  ```
  set(COMPILABLE_MESSAGE_PACKAGES
      ...
      <package_name, e.g. std_msgs>
      ...
      )
  ```
  This will automatically define the preprocessor macro `WITH_<PACKAGE_NAME>`

- Taking reference from [here](./include/influxdb_bridge_cpp/msgs/base.hpp), create
  (if not existing) the header file that includes the necessary header files (e.g.
  [std_msgs.hpp](./include/influxdb_bridge_cpp/msgs/std_msgs.hpp) for the standard message);
  file must be created in [this](./include/influxdb_bridge_cpp/msgs/) folder.
  If the file already exists, simply add the include statement for the particular message.

- Similarly, the actual message implementation must be inserted in a `.cpp` file inside the
  [measurements implementation](./src/measurements_implementation/) folder.
  The [point.hpp](./include/influxdb_bridge_cpp/point.hpp) defines for convenience the macro
  `IMPLEMENT_MEASUREMENT` that can be used. As example, to export a `std_msgs/msg/String`
  ``` cpp
  IMPLEMENT_MEASUREMENT(
      std_msgs::msg::String,    // variable type
      "string",                 // default measure name
      point.add_key<decltype(message->data)>("content", message->data); // implementation
      );
  ```
  The macro defines internally a `point` object of type `Point` on which different
  key can be added. The template library should admit only types that actually can be
  converted (in some way) in a format manageable by Influx; in case, error will be issued
  at compile time.

- Finally, register the measurement to the ROS2 node in the file [influx_bridge_node.cpp](./src/influx_bridge_node.cpp).
  Inside the `InfluxBridgeNode::add_subscriber` function implementation, register each type
  by specifying the string that must be parsed to create the binding and the corresponding type:
  ``` cpp
  #ifdef WITH_STD_MSGS
      REGISTER_NEW_MSG_TYPE("std_msgs/String", std_msgs::msg::String)
      ...
  #endif
  ```


As shown here, remember to use the inclusion guards to allow everyone to build the
library (even though they don't have all packages installed).
For this very reason, in each implementation file include always only the header file
interface defined within this package.






