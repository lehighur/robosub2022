#include <cstdint>
class StateMachine {
  public:
    StateMachine();
    uint8_t get_state();
  private:
    uint8_t current_state;
    enum States {
      IDLE,
      PREQUAL,
      COINFLIP,
      GATE,
      PATH,
      BUOY,
      BIN,
      TORPEDO,
      OCTAGON
    };
};
