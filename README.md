# ModRob-protocol
## ModRob-protocol description
ModRob is a lightweight protocol based on CAN bus for modular mobile robots. In our project we consider mobile robots with hierarchical modular architecture. In general, the control system structure of a typical mobile robot can be divided into the following main functional modules:
1. **Intelligent control module** - planning and distribution of tasks between the modules of the second level (the system-wide control function).
2. **Transport module** - robot’s motion in an environment (the transport function).
3. **Power module** - provides power to the robot components (the power function).
4. **Remote sensor module** - search for obstacles and manipulation objects (the information function).
5. **Special-purpose module** – the technological function.
6. **Wireless channel creation module** – communication with the external supervisor (the communication function).

These modules belongs to second level of hierarchy.  
Some second-level modules must perform a large amount of work. Therefore, according to our ModRob concept, each fully functional module should also be built as a modular architecture with submodule nodes (first level of the hierarchy). This architecture will allow to distribute the computational load of various functional significance between submodules, especially in cases where the number of drives and sensors may increase or their type may change. Additionally, one can combine modules at both the first and second levels of the hierarchy, respectively. 
For example the structure of the Transport module is based on three types of submodules:
1. **The Cognition submodule** - responsible for the administration of submodules on the bus.
2. **The Actuator submodule** – control connected electromotor: DC-motors, BLDC motors, stepper motors etc.
3. **The General Sensor submodule** - allows connecting a variety of distance sensors, encoders, gyroscopes, etc.  

ModRob protocol was developed for inter-module interaction of aforementioned submodules. The protocol is very simple: it is based on two main source files `modrob.hpp` and `modrob_aggregator.hpp`, user must write CAN driver which links CAN low level commands with `modrob.hpp` API. For debug code with USART user must also write USART driver for microcontroller in use.  
A software node is deployed on each hardware submodule. Each node has its ID (`moduleID`) and a collection of variables with their IDs (`variableID`). Therefore the full module address consists of `moduleID` and `variableID` (24 bits). The protocol is based on the principle of "binding" variables: when a given variable of one module changes, other variables of the modules that subscribed to it will change. To do this, a message containing information about the new value of the specified variable must be published with a non-zero frequency.  
A software node can receive and publish multicast messages. Each **ModRob message** occupies one **extended CAN frame** (128 bits). The message contains the following basic information: the module address, the command interpretation bit, the variable value, and the module type specifier.  
The ModRob message has next data fields:
1. `moduleID` - module address (identifier), which defines either the target address of the module to which the message will be sent, or the outgoing address of the module publishing a new message. 
2. `variableID` - the address (identifier) of any variable of the module, which defines either the target variable of the module, the properties of which must somehow be changed (value or frequency of publication), or a variable that will be published.
3. `messageType` - variable that defines message type:  
    3.1. `Command` - the modrob message acts like a command for a specific module with moduleID and variableID (set variable, variable publish frequency, set subscription).    
    3.2. `Publication` - the modrob message acts like The modrob message acts like a published message that is sent by the publisher module with moduleID and variableID, i.e. this message only affects the variables of those modules that subscribe to it.  
4. `typeID` - the identifier of the module type, which determines the belonging of the module to a particular class according to its functional role.  
5. `operationType` - the variable that defines the type of modrob message `Command`:  
    5.1. `SetVariable` - the command will set the value of a target variable of some module in accordance with the identifiers.  
    5.2. `SetFreqOfPublications` - the command will set the frequency of publication of a target variable of some module in accordance with the identifiers.  
    5.3. `SetSubscriptionAddress` - the command will set the subscription of a variable of one module to a variable of another module.  
6. `value` - the payload, which is defined differently depending on the specific command, i.e. depends on message's `operationType`:  
    6.1. if the `Command` is `SetVariable` then `value` is interpreted as a new floating point value (`float`) to be set for the given module variable.  
    6.2. if the `Command` is `SetFreqOfPublications` then `value` is interpreted as a new frequency of publication (`int` value) to be set for the given module variable.  
    6.3. if the `Command` is `SetSubscriptionAddress` then `value` is interpreted as two identifiers: the identifier of the module, the variable of which must be subscribed to, and the identifier of this variable itself.  

Messages are sent to all bus nodes, but they are processed if the message is intended for the target submodule. The incoming message is interpreted as either “set” (`Command`) or “update” (`Publication`).  
Depending on the message mode, the **set-message** (`Command`) can be:
- write the value to a receiver variable (by the variable ID) - `SetVariable`,
- set the publication frequency on a bus of a publisher variable (by the variable ID) - `SetFreqOfPublications`,
- subscribe a receiver variable to a publisher’s variable (using two IDs) – variables “binding” - `SetSubscriptionAddress`.  

The **update-message** (`Publication`) works as follows: the receiver variable that was previously subscribed to the publisher variable (by the set-message) gets a new value (updated).  
Now there are 5 modrob messages:
| Message | Input parameters | Description | 
| ----------- | ----------- | ----------- | 
| `HeartBeat` | 1. Module's ID<br/>2. Module's typeID | Message used to synchronize submodules on the bus. <br/> It allows monitoring the presence of submodules on the bus. |
| `commandSetValue` | 1. Target module's ID<br/> 2. Target variable's ID<br/> 3. Value of a variable | The command sets new value of a target variable of a target module  | 
| `commandSetHertz` | 1. Target module's ID<br/> 2. Target variable's ID<br/> 3. Frequency value | The command sets new frequency of publication of a target variable of a target module |
| `publishNewValue` | 1. Module's typeID<br/> 2. Publishing module's ID<br/> 3. Publishing variable's ID<br/> 4. Value of a variable | The command allows publishing value of a target variable on a bus (multicast message) |
| `commandSetSubscriptionAddress` | 1. Subscriber module's ID<br/> 2. ID of subscriber module's variable<br/> 3. Publisher module's ID<br/> 4. ID of publisher module's variable | The command subscribes the specified subscriber module variable to the specified publisher module variable, i.e. changing the publisher variable will change the subscriber variable. However, to do this, the subscriber variable must be published with a non-zero frequency |
 

