# New MiRo Demo Mode 

## Installation

1. Use the `pip_requirements.txt` and `conda_requirements.txt` to set up a Python virtual environment using the corresponding manager.

2. Set up your laptop to connect to and control a MiRo. Run a simple check, e.g. `miro_gui` to confirm that it works.

## Running

1. Activate your virtual environment 

3. Run `client demo` within the `core` folder:

```python
cd core/
python3 client_demo.py
```

> 
    (Some paid API keys have been removed so the demo might not run as expected, e.g. ElevenLabs, Porcupine, Whisper)
    Search the package for `key` to find all such instances.  

## Further details

### What are Actions and Nodes, and how are they different?

An **action** is a behaviour that MiRo can exhibit over a period of time in response to internal / external stimuli. MiRo has numerous actions that can be performed, the action to be perfomed at any given time is selected via a computed priority and as the demo mode runs, these priorities change in response to different stimuli that MiRo encounters, the action with the highest priority is the one that is performed.

A **node** is processing that can be done seperately from the actions, they can be called upon when required by the actions (e.g dance_miro can retrieve information from node_detect_word to see if a wake word has been used) and most have a tick function that is continously executed regardless of which action has the highest priority. They are typically used to process sensory information for use by client_demo or the actions compute_priority functions.


### How to create and add a new **Action**?
To create a new action using your own behaviour, **action_skeleton.py** has been provided to use as a base to build upon. The comments for each function show what it does and whether it is necessary which will help when implementing your own action. The existing actions can also be used as examples.  

When the action_{whatever}.py file has been completed, to add that action to the roster do the following steps:

1. In node_action.py, add the following import: `from action.YOUR_ACTION import YOUR_ACTION_CLASS_NAME` 

2. In the initialisation function of node_action.py, there is a list called self.actions with all the actions in, add a new instance of your action there.


### How to create a new **Node**?
Nodes are more flexible than actions in that they don't have as many preset functions that need to be filled in order to be used. A **node_skeleton.py** is provided but developing the functionality is more dependent on what you need it to do and what sensory input is required. The tick function allows the processing to be done consistently as the demo mode continues.

When the node_{whatever}.py file is ready, to integrate the node so it can be accessed and used:

1. In client_demo.py, find the function **instantiate** in the class **DemoNodes**. Add your new node as a class variable depending on which client type it suits. If it uses the MiRo's cameras, it usually will go under the "camera" client type, if not it will go under "main".

2. If your node uses a tick function (which it most likely will), the function needs to be called from somewhere in client_demo.py. For example detect_music.tick is called from the microphone callback because it requires the mic data, ticks that require no additional data can be called from the tick function in **DemoNodes**. Some work/intuition may be required to find the right place to call your tick function.

### How do I access variables/functions from nodes in my actions.
If you have added both the action and node correctly, use this path `self.parent.nodes.NODE_NAME.VARIABLE/FUNCTION`. This is because client_demo is the parent to the actions and "nodes" is the list of nodes in client_demo.py.
