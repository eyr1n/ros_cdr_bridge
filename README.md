# ros_cdr_bridge

A bridge that relays ROS 2 topics/services over WebSocket using raw CDR byte payloads.

## Build

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select ros_cdr_bridge
```

## Run

```bash
ros2 run ros_cdr_bridge ros_cdr_bridge_node
```

Parameters:
- `port` (default: `9090`)
- `qos_depth` (default: `10`)

## Protocol

### Text frames (control)

Client -> Server:

```json
{"callId":0,"op":"create_subscription","name":"/chatter","type":"std_msgs/msg/String"}
```

```json
{"callId":1,"op":"create_publisher","name":"/chatter","type":"std_msgs/msg/String"}
```

```json
{"callId":2,"op":"create_service_client","name":"/add_two_ints","type":"example_interfaces/srv/AddTwoInts"}
```

```json
{"op":"destroy","id":0}
```

Server -> Client:

```json
{"id":0,"callId":0}
```

```json
{"id":1,"callId":1}
```

```json
{"id":2,"callId":2}
```

Notes:
- `id` is assigned by server per WebSocket session, starting from `0`.
- `id` namespace is shared by publishers, subscriptions, and service clients.
- `callId` is client-defined and echoed back in create responses.
- Unknown/invalid control messages are ignored (no explicit error frame).

### Binary frames (data)

Client -> Server:
- Topic publish: `[1][id:u32][cdr...]`
- Service request: `[2][id:u32][call_id:u32][cdr...]`

Server -> Client:
- Topic message from subscription: `[1][id:u32][cdr...]`
- Service response: `[3][id:u32][call_id:u32][cdr...]`

Notes:
- `id` must match the entity created by a text control frame.
- `call_id` is client-defined correlation ID echoed back in service responses.
- All `u32` values in binary frames are little-endian.

## License

MIT
