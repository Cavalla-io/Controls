# Agent notes — safety_node

## Single responsibility

This node has **one job**: determine if the system is in a safe state and gate the command stream accordingly.

- **Safe** → pass the command through
- **Unsafe** → publish a zero `ForkliftDirectCommand` and log the reason

## In scope

- Adding new safety predicates (new subscriptions, timeouts, status checks)
- Modifying trip conditions or thresholds

## Out of scope

Do not add any of the following to this node:

- Presets, scaling, or command shaping
- PID, fork height control, or navigation logic
- Mode management or mission state
- Anything that changes how the forklift behaves in normal operation

If a feature isn't answering **"is it safe to output this command?"**, it belongs in a different node.
