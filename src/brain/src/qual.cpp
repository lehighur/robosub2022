#include "qual.h"

Qual::Qual() : q(), current_state(0) { printf("Qual constructor\n"); }

QUAL_STATE Qual::get_state() {
  return (QUAL_STATE)this->current_state;
}

int Qual::process_event(lur::Event) {
  return 0;
}

void Qual::run() {
  while (this->current_state != QUAL_STATE::QUAL_DONE) {
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
