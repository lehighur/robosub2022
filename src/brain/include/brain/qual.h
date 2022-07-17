#include <cstdint>

#include "common.h"

enum QUAL_STATE: uint8_t {
  START,
  GATE_DETECTED,
  APPROACH,
  THROUGH,
  TURN,
  BACK,
  LEAVE,
  QUAL_DONE,
};

class Qual {
  public:
    Qual();
    QUAL_STATE get_state();
    int process_event(lur::Event);
    void run();
  private:
    uint8_t current_state;
    TSQueue<lur::Event> q;
};
