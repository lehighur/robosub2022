#include <cstdint>
class StateMachine {
  public:
    StateMachine();
    uint8_t get_state();
  private:
    uint8_t current_state;
    enum STATE: uint8_t {
      IDLE,
      TEST,
      PREQUAL,
      COINFLIP,
      GATE,
      PATH,
      BUOY,
      BIN,
      TORPEDO,
      OCTAGON,
      DONE,
    };
};
