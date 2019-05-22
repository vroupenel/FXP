#include <cstdio>
#include "hsm.h"
#include <iostream>

class TestHSM;

typedef CompState<TestHSM,0>            Top;
typedef LeafState<TestHSM,1,Top>          SLEEPING;
typedef LeafState<TestHSM,2,Top>          WAKING_UP;
typedef CompState<TestHSM,3,Top>          STANDARD;
typedef LeafState<TestHSM,4,STANDARD>               CURIOUS;
typedef LeafState<TestHSM,5,STANDARD>               BORED;
typedef LeafState<TestHSM,6,STANDARD>               EMBARRASSED;
typedef LeafState<TestHSM,7,STANDARD>               INTERACTING;
typedef LeafState<TestHSM,8,Top>          AVOIDING;
typedef CompState<TestHSM,9,Top>          TIRED;
typedef LeafState<TestHSM,10,TIRED>                 DOCKING;
typedef LeafState<TestHSM,11,TIRED>                 WHINING;

enum Signal { A_SIG, B_SIG, C_SIG, D_SIG, E_SIG, F_SIG, G_SIG, H_SIG, I_SIG, J_SIG, K_SIG };

/*
A = HIGH BATTERY
B = TIMEOUT 30S
C = INTERACTION
D = TIMEOUT 20MIN
E = LOUD NOISE
F = TIMEOUT 5S
G = IGNORED
H = ACTION ENDED
I = LOW BATTERY
J = OBSTACLE
K = DOCK FOUND
L = ON DOCK

*/


class TestHSM;

#define HSMINIT(State, InitState) \
    template<> inline void State::init(TestHSM& h) { \
       Init<InitState> i(h); \
       printf(#State "-INIT;"); \
    }

HSMINIT(Top, SLEEPING)
HSMINIT(STANDARD, CURIOUS)
HSMINIT(TIRED, DOCKING)

class TestHSM {
public:
    TestHSM() { Top::init(*this); }
    ~TestHSM() {}
    void next(const TopState<TestHSM>& state) {
        prev_state_ = state_;
        state_ = &state;
    }
    Signal getSig() const { return sig_; }

    void dispatch(Signal sig) {
        sig_ = sig;
        state_->handler(*this);
    }
    unsigned getPrevStateId() { return (prev_state_->getId());}
    unsigned getStateId() { return (state_->getId());}


private:
    const TopState<TestHSM>* state_;
    const TopState<TestHSM>* prev_state_;
    Signal sig_;
};

//----------------------------------------------------------------------------------------------------------------------

int main (int argc, char *argv[]) {

    TestHSM test;
    while(true) {
        printf("\nSignal<-");
        char c = getc(stdin);
        getc(stdin); // discard '\n'

        if(c<'a' || 'k'<c) {
            return 0;
        }
        test.dispatch((Signal)(c-'a'));
    }
    return 0;
}

//----------------------------------------------------------------------------------------------------------------------

#define HSMHANDLER(State) \
    template<> template<typename X> inline void State::handle(TestHSM& h, const X& x) const

HSMHANDLER(SLEEPING) {
    switch (h.getSig()) {
        case A_SIG: { Tran<X, This, WAKING_UP> t(h);
            printf("HIGH BATTERY;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(WAKING_UP) {                                 // handler function called whenever an event occurs
    switch (h.getSig()) {
        case B_SIG: { Tran<X, This, STANDARD> t(h);
            printf("AWAKE NOW;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(STANDARD) {
    switch (h.getSig()) {
        case I_SIG: { Tran<X, This, TIRED> t(h);
            printf("LOW BATTERY;"); return; }
        case J_SIG: { Tran<X, This, AVOIDING> t(h);
            printf("OBSTACLE;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(CURIOUS) {
    switch (h.getSig()) {
        case C_SIG: { Tran<X, This, INTERACTING> t(h);
            printf("INTERACTION;"); return; }
        case D_SIG: { Tran<X, This, BORED> t(h);
            printf("TIMEOUT 20MIN;"); return; }
        case E_SIG: { Tran<X, This, EMBARRASSED> t(h);
            printf("LOUD NOISE;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(BORED) {
    switch (h.getSig()) {
        case C_SIG: { Tran<X, This, INTERACTING> t(h);
            printf("INTERACTION;"); return; }
        case G_SIG: { Tran<X, This, EMBARRASSED> t(h);
            printf("IGNORED;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(EMBARRASSED) {
    switch (h.getSig()) {
        case F_SIG: if (h.getPrevStateId() == 4) { Tran<X, This, CURIOUS> t(h);        // CURIOUS STATE ID = 4
            printf("TIMEOUT 5S;"); return; }
                    else { Tran<X, This, BORED> t(h);
            printf("TIMEOUT 5S;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(INTERACTING) {
    switch (h.getSig()) {
        case H_SIG: { Tran<X, This, CURIOUS> t(h);
            printf("INTERACTION ENDED;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(AVOIDING) {
    switch (h.getSig()) {
        case H_SIG: if (h.getPrevStateId()>=3 && h.getPrevStateId()<=7) { Tran<X, This, STANDARD> t(h);
                        printf("ACTION ENDED;"); return; }
                    else { Tran<X, This, TIRED> t(h);
                        printf("ACTION ENDED;"); return; }
        case J_SIG: { Tran<X, This, AVOIDING> t(h);             // UPDATE ?
            printf("OBSTACLE;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(TIRED) {
    switch (h.getSig()) {
        case J_SIG: { Tran<X, This, AVOIDING> t(h);         // NO TRANSITION TOWARD SLEEPING STATE
            printf("OBSTACLE;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(DOCKING) {
    switch (h.getSig()) {
        case K_SIG: { Tran<X, This, SLEEPING> t(h);
            printf("ON DOCK;"); return; }
        case C_SIG: { Tran<X, This, WHINING> t(h);
            printf("INTERACTION;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

HSMHANDLER(WHINING) {
    switch (h.getSig()) {
        case H_SIG: { Tran<X, This, DOCKING> t(h);        // Dock still to be found, go back to DOCKING
                        printf("INTERACTION ENDED;"); return; }
        default: break;
    }
    return Base::handle(h, x);
}

//----------------------------------------------------------------------------------------------------------------------

#define HSMENTRY(State) \
    template<> inline void State::entry(TestHSM&) { \
        printf(#State "-ENTRY;"); \
    }

HSMENTRY(SLEEPING)
HSMENTRY(WAKING_UP)
HSMENTRY(STANDARD)
HSMENTRY(CURIOUS)
HSMENTRY(EMBARRASSED)
HSMENTRY(BORED)
HSMENTRY(INTERACTING)
HSMENTRY(AVOIDING)
HSMENTRY(TIRED)
HSMENTRY(DOCKING)
HSMENTRY(WHINING)



#define HSMEXIT(State) \
    template<> inline void State::exit(TestHSM&) { \
        printf(#State "-EXIT;"); \
    }

HSMEXIT(SLEEPING)
HSMEXIT(WAKING_UP)
HSMEXIT(STANDARD)
HSMEXIT(CURIOUS)
HSMEXIT(EMBARRASSED)
HSMEXIT(BORED)
HSMEXIT(INTERACTING)
HSMEXIT(AVOIDING)
HSMEXIT(TIRED)
HSMEXIT(DOCKING)
HSMEXIT(WHINING)
