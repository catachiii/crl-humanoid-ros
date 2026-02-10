# Finite State Machine

This repository contains an implementation of a finite state machine that activates or deactivates certain ROS2 nodes based on the current state and machine. Different machines may activate different nodes in the same state.

The implementation is designed with usability and easy of deployment in mind. The user only needs to specify the content of the states, the transitions, and the current machine. The framework will automatically compute and generate the code that needs to be run on the current machine. Using templates in C++17, the computation required for generation are delegated to compilation time, enabling a fast and smooth experience.

# Concepts

## States and Machines

It is necessary to let the framework know the different kinds of states and machines. They are defined as an enum wrapper using `crl_fsm_states(name, values...)` and `crl_fsm_machines(name, values...)`. Currently, the underlying type for both enums are `char`, thus there can be at most 256 different states and machines. If this is insufficient, the underlying representation can be changed in `common.h`. It is also necessary to change the `StateSwitch` service's input and return type to the corresponding type as well.

## Partial States

For a given state and machine, the ROS2 nodes that need to be launched can be further divided into subgroups. For example, in order to make a robot walk, the control pipeline may be divided into multiple different parts with different dependencies. The nodes required for each part are grouped as a partial state. This modularises a given state, and partial states can be swapped easily.

To define a partial state, one needs to inherit from `fsm::PartialStateTemplate` using the curiously recurring template pattern. The functions `void begin_cb(rclcpp::Executor&)` and `void end_cb(rclcpp::Executor&)` must be defined from the child case. The constructor of the class is called when the program begins and the destructor is called when the program ends. `begin_cb` and `end_cb` are called when the particular state is entered and exitted. To add or remove nodes, the nodes are added to or removed from `rclcpp::Executor`. Note that this object is persistent.

To create a partial state, one needs to return a function object that returns an instance of the defined partial state. This allows the definition of how to create the object without actually creating it. For easy of implementation, lambda functions are recommended.

An example is shown in `partial_state.h`.

## Transitions

Transitions are defined at compilation time by `fsm::Transition<from_state, to_state> name;`. These are unidirectional edges that indicate it is possible to transition from one state to the other. Currently, transitions do not support callbacks.

## Collections

The partial states and transitions are used to create state collections and transition collections respectively. The state collection performs the necessary filtering step to identify the subset of partial states necessary for a particular machine. The transition collection generates an adjacency matrix that is easily queried during runtime. Again, both these processes are performed during compilation to reduce runtime overhead.

## FSM

Both collections are passed to create finite state machine. One instance is created for every possible machine and takes in the initial state as well. Each fsm creates an ROS2 node that handles the communication and is in control of a `rclcpp::Executor`. The ROS2 node acts as an ROS2 service server. When a state switch request is recieved, it first checks whether it is feasible. If so, then the `begin_cb` and `end_cb` callbacks defined by the partial states corresponding to the current/previous state and machine are called. A response with enum type `fsm::StateSwitchRes` will then be provided. 

## Client

Since multiple FSM exist, there exists multiple ROS2 service servers as well. In order to more efficiently control the communication to and from the servers, `fsm::Client` was created.