Hierarchical State Machine (HSM) - Fuji Xerox Project
Author: Virginie Roupenel, Ecole Centrale de Nantes, Creative Robotics Lab; 2019

Description:

event_hdl.cpp : A program listening to the robot’s sensors to detect possible interactions, obstacles coming its way, the current battery state or every other relevant event which might trigger a transition in the state machine, and printing the according transition index.

hsm.h :  A header file allowing the description of a state machine using classes of templates.

main.cpp : : A program which defines the entire state machine and handles the transitions.

planner.py : Piped to ./main, this program triggers the sets of behaviours the robot should have according to the current state, in the following order : EXIT actions, TRANSITION actions, ENTRY actions.

Bonus :  the “behavior” directory: A set of compensatory behaviours for obstacle avoidance, adapted to the Fetch robot.


Relevant mistakes made and how I solved them: 

1. In ./main : Initially, the state machine description did not allow the identification of the previous state, whereas some of the current transitions depend on the former state of the robot (ex. : AVOIDING-exit).
Solution chosen: A get_prev_state_id() function was added to the state-machine class.

2. In ./main : The “hide behind shield” action was initially a TIRED-exit one : this solution prevented the robot to avoid obstacle once it reached the TIRED state, as the robot would have hidden behind its shield when exiting the tired state, after detecting an obstacle, blocking its own sensors from following it.
Solution chosen: The “hide behind shield” action  became a transition action (the only one in this state-machine), meaning that the robot will hide only when the transition from the TIRED state to the SLEEPING state is triggered.

3. In the terminal : Due to the buffering when piping the different programs, the different events detected and the actions triggered would only be printed out on the terminal once the program was terminated and not as the events were detected. 
Solution chosen: Buffering in the pipes is now turned off with the following commands :
$ unbuffer program1 | unbuffer -p program2 | program3

4. In ./event_hdl  : A few recurrent conditions were missing in the functions, in particular to break them when an interaction is requested or an obstacle is detected.
Solution chosen: The missing conditions of the "interaction requested"/"obstacle detected" events were revealed by multiple tests and added when necessary.

5. In ./event_hdl: The tiredness and boredom threads were initially joined in the routine functions where they were created and started, which would pause the different routine checks executed in the main thread.
Solution chosen: Another thread was created for the routine functions. The tiredness and boredom threads are now created as global variables, started in the routine checks and joined in the main thread, whose purpose is now to check if the thread functions are done. That solution also prevents a thread from restarting before being joined.

6. Initially, the DOCKING state was separated in two sub-states whose purpose was to find the dock and go to it. If an interaction was requested after the robot entered the second state, ./event_hdl would trigger the transition to the WHINIG state then re-trigger the transition from DOCKING_1 to DOCKING_2, while ./main considered going back directly from WHINING to DOCKING_2. 
Solution chosen: The use of a DOCKING state in two parts was discarded and the two sub-states were merged. (In the future we might want to consider a way to save the last trajectory started in the state action code to avoid re-planning.) 


Future directions:

1. Complete the different functions of the routine checks and threads: 
The routine functions will eventually listen to the relevant robot’s sensors and send a message accordingly to trigger the right transitions.
In particular, the check_battery function needs the percentage of battery charge left.
The INTERACTING state was left deliberately mostly empty as choices are to be made in the future :
- How does the robot recognize when someone wants to interact? (check_proximity function)
- When is the interaction considered as finished? (conversation thread function)
- How does the robot start an interaction? Cf. CURIOUS/BORED future sets of behaviours.

Moreover, some choices have to be made regarding the way the robot deals with obstacles. In particular, how does the robot detect obstacles coming its way? (check_obstacle function) And when is it done avoiding them? (obstacle_avoided function)

What is considered a loud noise and how to detect it? (surprised function) This event might not be compatible with the robots’ sensors, it is only an example of what could trigger the EMBARRASSED state.
In the current ./event_hdl program, the embarrassment thread is in the experimental state : 
- Loud noises are only detected in the CURIOUS state, 
- The robot cannot interact when EMBARRASSED,
- Obstacles are not detected when the robot is EMBARRASSED  as it is supposed to hide behind its shield, therefore possibly blocking its own sensors from detecting the obstacle.

The “ignored” event depends on how the robot considers that its attempt to interact (wave, bow its head,…) has failed : I would recommend that the planner publishes an attempt to start an interaction on a topic that the interaction_failed function would listen to, and wait a certain amount of time for an interaction request before considering that the attempt has failed. 

Finally, regarding the ready_to_sleep function, (i.e. when the robot is docked) : we need to clarify how the robot considers that it is docked (ex: in a specific pose facing the room?), which depends on how the navigation of the robot is implemented.

Some complements might lead to the creation of new threads : in that case, the main thread must be updated to join the new ones.


2. Security measure:
I would recommend adding a LOST state as a security measure : in case of problems in localisation, avoidance, etc. , the robot could transition to that state and “recover” before resuming it’s activities. This would require to substancially modify the ./main program in order to create the new state and add a new “security” thread to the ./event_hdl program. Currently, the robot cannot detect any event when waking up or avoiding obstacles as those states are not processed in separate threads : this can be updated by following the example of how a new interaction is handled.

3. ROS conversion: 
The routine functions in the ./event_hdl program should listen to the relevant topics and, for instance, publish the event on a topic the main program will subscribe to. I would recommend turning the ./event_hdl program into a ROS node on its own, as it is the program most likely to be modified. 
In ./event_hdl, the different durations in states and transitions, currently configured for simulation, need to be set to their final value.

4. Sets of behaviours : 
In the actual planner.py program : behavioural/action functions, executed during each state, need to be created to replace the actual ACTIONS.write() commands. 
Note : Some of the behaviours should be looping actions which will only cease when a new relevant event is detected, while other should finish before a new event is taken into account.


How to use it:

In the “HSM-build” directory:
Compile event_hdl.cpp:
$ g++ -std=c++0x -o event_hdl event_hdl.cpp -pthread 

Execute the whole system: (turn off buffering in pipes)
$ unbuffer ./event_hdl | unbuffer -p ./main | ./planner.py

Alternatively:
You may want to check the order in which the events are detected:
$ ./event_hdl
Test the consequences of any modification made in the event_hdl file in the main program: (each line must end with “;”)
$ unbuffer ./event_hdl | ./main 

How I tested it:

1. Drafted by hand the diagram according to the final ./main program transitions, leaf and composite states, and compared it to the initial diagram State_machine_FXP. Then I added a user input function to test the machine transitions.
2. Wrote a list of tests, as a simulation thread started in the main thread of the ./event_hdl program, to challenge the system and reveal any forgotten conditions. It also allowed me to test the planner of the system once it was piped to ./main, which was piped to ./event_hdl itself.

How I would like to have it tested:

1. Have the ./main program automatically creating a diagram representing the states, sub-states and transitions, then compare it to the initial diagram State_machine_FXP.
2. Challenge the ./event_hdl program with data from actual sensors and test the output of the system when multiple events are detected, to highlight any missing conditions in the program's functions.
