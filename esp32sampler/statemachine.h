#include <stdbool.h>

#include "regression.h"

typedef enum {
    STATE_IDLE,
    STATE_LOW,
    STATE_SLOPE
} EState;

class StateMachine {

  private:
    double low { 0 };
    double high { 0 };
    EState state { STATE_IDLE };
    double result { 0 };
    Regression regression;

  public:
    /**
     * Constructor.
     */
     explicit StateMachine(double low, double high);

    bool process(double time, double v);

    double get_result();
};
