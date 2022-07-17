#include "qual.h"

Qual::Qual() : current_state(0) { printf("Qual constructor\n"); }
QUAL_STATE Qual::get_state() {
  return this->current_state;
}
void run() {
  while (this->current_state != QUAL_STATE::DONE) {
    switch (this->current_state) {
      case 0:
        // QUAL_STATE::START
        break;
      case 1:
        // QUAL_STATE::GATE_DETECTED
        break;
      case 2:
        // QUAL_STATE::APPROACH
        break;
      case 3:
        // QUAL_STATE::THROUGH
        break;
      case 4:
        // QUAL_STATE::TURN
        break;
      case 5:
        // QUAL_STATE::BACK
        break;
      case 6:
        // QUAL_STATE::LEAVE
        break;
      case 7:
        // QUAL_STATE::DONE
        break;
    }
  }
}
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
