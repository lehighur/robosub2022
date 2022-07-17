#include <cstdint>
enum QUAL_STATE: uint8_t {
  START,
  GATE_DETECTED,
  APPROACH,
  THROUGH,
  TURN,
  BACK,
  LEAVE,
  DONE,
};

class Qual {
  public:
    Qual();
    QUAL_STATE get_state();
    void run();
  private:
    uint8_t current_state;
};
