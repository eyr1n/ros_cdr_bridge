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
- `port` (default: `54321`)

## Protocol

### Client -> Server

#### create publisher

```ts
{
  op: "create_publisher";
  call_id: number;
  name: string;
  type: string;
  qos: QosProfile; // see QoS section
}
```

#### create subscription

```ts
{
  op: "create_subscription";
  call_id: number;
  name: string;
  type: string;
  qos: QosProfile; // see QoS section
}
```

#### create service client

```ts
{
  op: "create_service_client";
  call_id: number;
  name: string;
  type: string;
  qos: QosProfile; // see QoS section
}
```

#### destroy

```ts
{
  op: "destroy";
  id: number;
}
```

#### topic publish (binary)

```ts
[op: 0, id: number, ...cdr: bytes]
```

#### service request (binary)

```ts
[op: 1, id: number, call_id: number, ...cdr: bytes]
```

### Server -> Client

#### create response

```ts
{
  call_id: number;
  id: number;
}
```

#### topic message (binary)

```ts
[op: 0, id: number, ...cdr: bytes]
```

#### service response (binary)

```ts
[op: 2, call_id: number, ...cdr: bytes]
```

### QoS

```ts
type QosProfile = {
  profile?:
    | "sensor_data"
    | "parameters"
    | "default"
    | "services_default"
    | "parameter_events"
    | "system_default"
    | "best_available"; // if supported by target RMW
  history?: "system_default" | "keep_last" | "keep_all";
  depth?: number;
  reliability?: "system_default" | "reliable" | "best_effort" | "best_available";
  durability?: "system_default" | "transient_local" | "volatile" | "best_available";
  deadline?: { sec: number; nsec: number } | "infinite";
  lifespan?: { sec: number; nsec: number } | "infinite";
  liveliness?: "system_default" | "automatic" | "manual_by_topic" | "best_available";
  liveliness_lease_duration?: { sec: number; nsec: number } | "infinite";
  avoid_ros_namespace_conventions?: boolean;
};
```

Notes:
- `id` is assigned by server per WebSocket session, starting from `0`.
- `id` namespace is shared by publishers, subscriptions, and service clients.
- `call_id` is client-defined and echoed in create responses and service responses.
- `destroy` has no response.
- Unknown/invalid control messages are ignored (no explicit error frame).
- Text enum/time string values are case-sensitive and must be lowercase.
- `u32` values in binary frames are little-endian.

## License

MIT
