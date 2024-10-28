# TalkMachines

## Structure

### Control + Perception
This module is responsible for executing commands based on the input received. It is subdivided into the following components:

- **say_move**: 
  - **say_move**: Utilizes the OpenAI API for general command execution.
  - **say_move_copilot**: Uses the Sydney Copilot API for specialized command handling.
  - **say_move_copilot_v2**: An enhanced version of the Sydney Copilot API integration.

### Perception (Visual + Sensor Data)
This module processes visual and sensor data to understand the environment and inform decision-making.

- **verbal_states**: 
  - **say_move_copilot_verbal_states**: Integrates verbal state data with the Sydney Copilot API.

