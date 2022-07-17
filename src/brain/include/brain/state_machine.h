#include <cstdint>
enum STATE: uint8_t {
  IDLE,
  TEST,
  QUAL,
  COINFLIP,
  GATE,
  PATH,
  BUOY,
  BIN,
  TORPEDO,
  OCTAGON,
  DONE,
};

class StateMachine {
  public:
    StateMachine();
    STATE get_state();
    void run();
  private:
    uint8_t current_state;
};
