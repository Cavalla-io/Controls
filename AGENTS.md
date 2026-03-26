# Agent notes — safety_node

## Single responsibility

This node has **two jobs**: determine if the system is in a safe state and gate the command stream accordingly, multiplex teleop and automation commands.

- **Safe** → pass the command through
- **Unsafe** → publish a zero `ForkliftDirectCommand` and log the reason

## In scope

- Adding new safety predicates (new subscriptions, timeouts, status checks)
- Modifying trip conditions or thresholds

## Out of scope

Do not add any of the following to this node:

- Presets, scaling, or command shaping
- PID, fork height control, or navigation logic
- Anything that changes how the forklift behaves in normal operation

If a feature isn't answering **"is it safe to output this command?"** or **"is this the right time to send this command?"**, it belongs in a different node.

---

# Agent notes — driver_node

## Single responsibility

This node is a **pure CAN translator**. It takes a `ForkliftDirectCommand` from the safety node and converts it into CAN bus messages. It contains no business logic, no presets, no scaling, and no command shaping.

## Gateway rule

**Nothing publishes directly to the driver node.** All commands must flow through the safety node first. The driver's only command subscription is `/safe/raw_command`, which is owned by the safety node.

## In scope

- CAN bus initialization and hardware communication
- Reading message fields and mapping them to CAN frame values
- Applying hardware-level defaults when message fields are zero (e.g. `accel_time_s=0.0` → use `1.0s`)

## Out of scope

Do not add any of the following to this node:

- Presets, scaling, or command shaping
- Subscriptions to topics other than `/safe/raw_command`
- Fork height logic, navigation, or mode management
- Any decision-making about what the forklift should do
