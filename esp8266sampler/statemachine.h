#include <stdbool.h>

#include "regression.h"

typedef enum {
    STATE_IDLE,
    STATE_LOW,
    STATE_SLOPE
} EState;

class StateMachine {

  private:
    double low;
    double high;
    EState state;
    double result;
    Regression regression;

  public:
    /**
     * Constructor.
     */
     explicit StateMachine(double low, double high);

    bool process(double time, double v);

    double get_result();
};
