#ifndef STATE_H
#define STATE_H

typedef struct States{
    void (*action_function)(void);
    struct States * next_state;
}states;

#endif