#!/usr/bin/env python
import random
import sys

TRANSITIONS = sys.stderr
ACTIONS = sys.stdout

while True:
    #print(random.choice('abcdefghijkl'))
    transition = sys.stdin.readline()
    if not transition:
        break
    TRANSITIONS.write(transition)

    if "WAKING_UP-EXIT" in transition:
            ACTIONS.write("Stretch\n")

    if "EMBARRASSED-EXIT" in transition:
            ACTIONS.write("Open shield\n")


    if "ON DOCK" in transition:
            ACTIONS.write("Hide behind shield\n")


    if "SLEEPING-ENTRY" in transition:
            ACTIONS.write("Zzz\n")

    if "WAKING_UP-ENTRY" in transition:
            ACTIONS.write("Open shield\n")
            ACTIONS.write("Look around\n")
    
    if "STANDARD-ENTRY" in transition:
            ACTIONS.write("Adopt the standard stance\n")

    if "CURIOUS-ENTRY" in transition:
            ACTIONS.write("Do random curious behaviours\n")

    if "BORED-ENTRY" in transition:
            ACTIONS.write("Do random bored behaviours, Look for interaction\n")

    if "EMBARRASSED-ENTRY" in transition:
            ACTIONS.write("Hide behind shield\n")
   
    if "AVOIDING-ENTRY" in transition:
            ACTIONS.write("Do dynamical obstacle compensation behaviours\n")

    if "TIRED-ENTRY" in transition:
            ACTIONS.write("Adopt a tired stance\n")

    if "DOCKING-ENTRY" in transition:
            ACTIONS.write("Plan and execute trajectory to the docking area\n")

    if "WHINING-ENTRY" in transition:
            ACTIONS.write("Reprimand interactor, Avoid interactor\n")

 






