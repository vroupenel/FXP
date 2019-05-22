#include <iostream>
#include <cstdio>
#include <ctime>
#include <thread>


/// _________________________________________________________________________________

// Tools

/* g++ -std=c++0x -o event event.cpp -pthread : compile.
 * unbuffer ./event | unbuffer -p ./main | ./planner.py : turn off buffering in pipes.
 *
 * Note : use "fflush(stdout);" after a "printf" in a thread.
 */

void wait(int seconds) {
  clock_t endwait;
  endwait = clock() + seconds * CLOCKS_PER_SEC ;
  while (clock() < endwait) {}
}

// Events index
/*
a = HIGH BATTERY
b = TIMEOUT 30S
c = INTERACTION
d = TIMEOUT 20MIN
e = LOUD NOISE
f = TIMEOUT 5S
g = IGNORED
h = ACTION ENDED
i = LOW BATTERY
j = OBSTACLE
k = ON DOCK
*/

/// ________________________________________________________________________________

// Global variables

bool low_battery = true;
float batt_state = 0.5;                     // Percentage of battery left (0-1), DECLARE in check_battery when completed !
bool cannot_interact = true;
bool cannot_avoid = true;

bool interaction = false;
bool obstacle = false;

bool bored = false;                         // Robot in BORED state
bool loud_noise = false;                    // Way to go to the EMBARRASSED state (once again, only an EXPERIMENT here)
bool on_dock = true;

std::thread t1;                             // Boredom thread
bool stop_t1 = false;
std::thread t2;                             // tiredness thread
bool stop_t2 = false;
std::thread t3;                             // Embarrassment thread
bool stop_t3 = false;
std::thread t4;                             // Conversation thread
bool stop_t4 = false;


/// _________________________________________________________________________________

// Thread functions

void boredom() {
    /* ------------------------------------------------------------------------------
     * Thread leading to the BORED state (if after the endwait time, the robot was
     * not interrupted by an obstacle, a "loud noise", a low battery or an interaction)
     -------------------------------------------------------------------------------*/

    clock_t endwait;
    endwait = clock() + 12 * CLOCKS_PER_SEC ;              // Set a realistic endwait time (ex : 1200) before getting BORED
    bool getting_bored = true;
    while (clock() < endwait) {
        if (interaction || obstacle || low_battery || (loud_noise || t3.joinable())){            // ?
            getting_bored = false;
            endwait = clock();
        }
    }

    if (getting_bored) {
        printf ("d\n");
        fflush(stdout);
        getting_bored = false;
        bored = true;
    }
    stop_t1 = true;
}

void tiredness() {
    /* --------------------------------------------------------------------------------
     * Thread leading the robot to the SLEEPING state, if not interrupted by an obstacle
     * or an interaction.
     ---------------------------------------------------------------------------------*/

    clock_t endwait;
    endwait = clock() + 600 * CLOCKS_PER_SEC ;              // Set a security duration for reaching the dock
    bool tired = true;
    while (!on_dock && clock() < endwait) {
        if (interaction||obstacle){
            tired = false;
            endwait = clock();
        }
    }
    if (!on_dock && !interaction && !obstacle && (clock() > endwait)) {
        std::cerr << "Error : Unable to reach dock\n" ;
    }
    else {
        if (tired) {
            while (t4.joinable()) {}
            printf ("k\n");
            fflush(stdout);
            cannot_interact = true;
            cannot_avoid = true;
        }
    }

    stop_t2 = true;
}

void embarrassment() {
    /* --------------------------------------------------------------------------------
     * Thread leading the robot to the EMBARRASSED state from the BORED states,
     * if not interrupted by an obstacle or a low battery.
     ---------------------------------------------------------------------------------*/
    if (bored) {
        bool ignored = true;
        clock_t endwait;
        endwait = clock() + 10 * CLOCKS_PER_SEC ;           // Waiting 10s for someone to interact after asking
        while (clock() < endwait) {
            if (interaction||obstacle||low_battery){
                ignored = false;
                endwait = clock();
            }
        }
        if (ignored) {
            ignored = false;
            cannot_interact = true;
            printf ("g\n");
            fflush(stdout);
            wait(5);                                        // Duration in the EMBARRASSED state : 5s
            cannot_interact = false;
            printf ("h\n");
            fflush(stdout);
        }
    }
    else {
        printf ("e\n");
        cannot_interact = true;
        wait(5);                                       // Duration in the EMBARRASSED state : 5sec
        printf ("f\n");
        cannot_interact = false;
    }

    stop_t3 = true;
}

/// THE FOLLOWING FUNCTION NEEDS TO BE COMLETED
/// ___________________________________________

void conversation() {
    /* --------------------------------------------------------------------------------
     * Thread handling the robot's interactions
     ---------------------------------------------------------------------------------*/

    if (low_battery && t2.joinable()) {                       // Robot in TIRED state
        clock_t endwait;
        bool timeout = true;
        endwait = clock() + 5 * CLOCKS_PER_SEC ;              // Set a whining duration (in seconds), update later?
        while (clock() < endwait) {
            if (obstacle){
                timeout = false;
                endwait = clock();
            }
        }
        interaction = false;
        if (timeout) {
            printf ("h\n");
            fflush(stdout);
        }
    }
    else {

        /* ------------------------------------------------------------------
         * To fill :
         * Find a way to check that the interaction has ended.
         *
         *
         -------------------------------------------------------------------*/
        // Temporary solution : interaction duration = 5sec

        clock_t endwait;
        bool timeout = true;
        endwait = clock() + 5 * CLOCKS_PER_SEC ;
        while (clock() < endwait) {
            if (obstacle || low_battery){
                timeout = false;
                endwait = clock();
            }
        }
        interaction = false;
        if (timeout) {
            printf ("h\n");
            fflush(stdout);
        }
    }
    stop_t4 = true;
}

/// THE FOLLOWING SECTION NEEDS TO BE COMLETED WITH THE RIGHT TOPICS/FUNCTIONS
/// __________________________________________________________________________
// Routine functions


void check_battery() {
    /* ------------------------------------------------------------------
     * To fill :
     * Update the battery percentage "batt_state" (between 0 and 1).
     *
     *
     -------------------------------------------------------------------*/

    if ( (batt_state > 0.8) && (low_battery) && !t2.joinable()){                       // Set the "high_battery" percentage : here 80%  // Robot not in TIRED state (for simulation only)
        on_dock = false;
        low_battery = false;
        printf ("a\n");
        wait(3);                                           //<PB HERE?!>         // Set a realistic waking up duration (ex : 30sec)
        printf ("b\n");
        cannot_avoid = false;
        cannot_interact = false;

        t1 = std::thread(boredom);

    }

    if ( (batt_state < 0.15) && (!low_battery) && (!obstacle)){       // Set the "low_battery" percentage : here 15%
        low_battery = true;
        if (bored) { bored = false;}
        printf ("i\n");                                               // Cannot go to the TIRED state while avoiding obstacles

        t2 = std::thread(tiredness);
    }
}


void check_proximity() {
    if (!obstacle && !interaction && !cannot_interact && !t3.joinable()) {
        /* ------------------------------------------------------------------    // NOTE : Cannot interact when EMBARRASSED
         * To fill, then uncomment function in main :
         * Check if someone wish to start an interaction with the robot.
         *
         *
         -------------------------------------------------------------------*/
            interaction = true;
            if (bored) { bored = false;}
            printf ("c\n");
    }

}


void check_obstacle() {
    if (!obstacle && !cannot_avoid && !t3.joinable()) {                          // Only the first obstacle detected matters
        /* ------------------------------------------------------------------    // NOTE : Cannot detect obstacle when EMBARRASSED
         * To fill, then uncomment function in main :
         * Check if there is an obstacle coming the robot's way.
         *
         *
         -------------------------------------------------------------------*/
            cannot_interact = true;
            obstacle = true;
            if (bored) { bored = false;}
            printf ("j\n");
    }
}


void surprised() {
    /* ------------------------------------------------------------------
     * To fill :
     * Find a way to check if there is a "loud noise" which might surprise
     * the robot
     *
     *
     -------------------------------------------------------------------*/
    loud_noise = true;
    if (loud_noise) {
        if (low_battery || obstacle || interaction || bored || cannot_interact) {
            loud_noise =false;
        }
        else {
            loud_noise = false;
            while (t3.joinable()){}
            t3 = std::thread(embarrassment);


        }
    }
}


void obstacle_avoided() {
    if (obstacle) {
        /* ------------------------------------------------------------------
         * To fill :
         * Check if the robot is done avoiding obstacles.
         *
         * NOTE : the robot can only avoid one obstacle at a time here :
         * implement "multiple obstacles" case directly in the action script.
         * (ex : "obstacle" becomes false once all the obstacles are avoided)
         *
         *
         *
         -------------------------------------------------------------------*/
        // Temporay solution : 10sec = avoidance time (delete it when the section is filled)
        clock_t endwait;
        endwait = clock() + 10 * CLOCKS_PER_SEC ;
        while (clock() < endwait) {
            if (interaction){ interaction = false;}     // only for simulation of interaction : won't detect it here in reality
            if (loud_noise){ loud_noise = false;}       // only for simulation of noises : won't detect it here in reality
        }

        obstacle = false;
        cannot_interact = false;
        printf ("h\n");

        if (low_battery) {
            while (t2.joinable()){};
            t2 = std::thread(tiredness);

        }
        else {
            while (t1.joinable()){};
            t1 = std::thread(boredom);
        }
    }
}

void interaction_finished() {
    if (interaction && !t4.joinable()) {                // Check if not already interacting
        t4 = std::thread(conversation);
    }
}

void ready_to_sleep() {
    if (t2.joinable() && !on_dock) {                    // Robot in TIRED state
        /* ------------------------------------------------------------------
         * To fill :
         * Find a way to check if robot is on dock
         *
         *
         -------------------------------------------------------------------*/
        // Temporary solution : 10sec = time to reach dock after tiredness thread is started, w/o interruption
            bool timeout = true;
            clock_t endwait;
            endwait = clock() + 10 * CLOCKS_PER_SEC ;
            while (clock() < endwait) {
                if (interaction||obstacle){
                    timeout = false;
                    endwait = clock();
                }
            }
            if (timeout) {on_dock = true;}
    }
}


void interaction_failed() {
        if (bored) {
    /* ------------------------------------------------------------------
     * To fill :
     * Find a way to check if robot tried to interract and failed
     * Listen to a topic published on by planner.py and sending a bool when the robot try to interact,
     * if a 1 is published on the topic :
     *
     *
     * while (t3.joinable()){};
     * t3 = std::thread(embarrassment);
     -------------------------------------------------------------------*/
    }

}

/// _________________________________________________________________________________
/*Serie of tests possible to activate with the t0 thread in the main thread*/

void test() {
    //Simulate a list of events

    batt_state = 0.9;
    wait(1);
    check_obstacle();
    wait(5);
    surprised();
    wait(6);
    check_proximity();
    wait(2);
    check_obstacle();
    wait(30);
    batt_state = 0.12;
    wait(5);
    check_proximity();
    wait(20);
    batt_state = 0.9;

}

void test_interaction() {
    // Simulate a request for interaction at every possible moment

    batt_state = 0.9;
    wait(1);
    check_proximity();
    wait(5);
    check_proximity();
    wait(7);
    surprised();
    wait(1);
    check_proximity();
    wait(15);
    check_proximity();
    wait(10);
    check_obstacle();
    wait(2);
    check_proximity();
    wait(10);
    batt_state = 0.12;
    wait(5);
    check_proximity();
    wait(13);
    batt_state = 0.9;

}

void test_obstacle() {
    // Simulate an obstacle at every possible moment

    batt_state = 0.9;
    wait(1);
    check_obstacle();
    wait(15);
    check_obstacle();
    wait(7);
    surprised();
    wait(1);
    check_obstacle();
    wait(30);
    check_obstacle();
    wait(2);
    check_obstacle();
    wait(10);
    batt_state = 0.12;
    wait(2);
    check_obstacle();
    wait(13);
    batt_state = 0.9;

}

void test_noise() {
    // Simulate loud noises at every possible moment

    batt_state = 0.9;
    wait(1);
    surprised();
    wait(5);
    surprised();
    wait(5);
    check_proximity();
    wait(3);
    surprised();
    wait(20);
    surprised();
    wait(5);
    check_obstacle();
    wait(2);
    surprised();
    wait(10);
    batt_state = 0.12;
    wait(5);
    surprised();
    wait(2);
    check_proximity();
    wait(2);
    surprised();
    wait(13);
    batt_state = 0.9;
}

void test_mult_infos() {
    // Simulate multiple relevant infos arriving at the same time
    batt_state = 0.9;
    wait(1);
    check_obstacle();
    check_proximity();
    surprised();
    wait(5);

    surprised();
    check_proximity();
    wait(6);

    check_obstacle();
    check_obstacle();
    wait(12);

    surprised();
    check_obstacle();
    wait(12);

    check_proximity();
    surprised();
    wait(2);
    check_obstacle();
    surprised();

    wait(12);
    batt_state = 0.12;
    wait(5);
    check_proximity();
    check_obstacle();
    wait(20);
    batt_state = 0.9;
}

/// _________________________________________________________________________________

void checks() {
    // Do a list of checks in loop

    while(true) {
        check_battery();
        // std::cout<<"1\n";

        // check_proximity();                       // Uncomment when the corresponding section is filled
        // check_obstacle();                        // Uncomment when the corresponding section is filled
        // surprised();                             // Uncomment when the corresponding section is filled

        obstacle_avoided();
        // std::cout<<"2\n";
        interaction_finished();
        // std::cout<<"3\n";
        ready_to_sleep();
        // std::cout<<"4\n";
        interaction_failed();
        // std::cout<<"5\n";
    }
}
/// _________________________________________________________________________________

int main()
{
    
    std::thread t0(test);                           // Use only for simulation of events
    std::thread th(checks);

    // Join the corresponding threads when done
    while(true){

        if (stop_t1) {
            t1.join();
            stop_t1 = false;
        }

        if (stop_t2) {
            t2.join();
            stop_t2 = false;
        }

        if (stop_t3) {
            t3.join();
            stop_t3 = false;
            if (!bored){
                while (t1.joinable()){}
                t1 = std::thread(boredom);
            }
        }

        if (stop_t4) {
            t4.join();
            stop_t4 = false;
            if (low_battery && !obstacle) {
                while (t2.joinable()){}
                t2 = std::thread(tiredness);
            }
            else {
                if (!obstacle) {
                    while (t1.joinable()){}
                    t1 = std::thread(boredom);
                }
            }
        }
    }

    t0.join();                                      // Use only for simulation of events
    th.join();

    return 0;
}

/// _________________________________________________________________________________

