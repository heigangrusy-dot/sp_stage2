#include "can.hpp"

#include <cassert>
#include <iostream>

CAN_TypeDef global_regs[2];
CAN_HandleTypeDef hcan1{&global_regs[0]}, hcan2{&global_regs[1]};
uint32_t mock_tick = 0;

struct Fixture {
  CAN_TypeDef regs;
  CAN_HandleTypeDef handle{&regs};
  sp::CAN can{&handle};
  Fixture() { mock_tick = 0; }
};

int main()
{
  { // Full mailboxes must not silently accept new commands; old commands expire.
    Fixture f;
    f.can.tx_data[0] = 3;
    assert(f.can.send(0x1FE) == HAL_OK);
    assert(f.can.send(0x200) == HAL_OK);
    assert(f.can.send(0x200) == HAL_OK);
    assert(f.can.send(0x200) == HAL_BUSY);
    assert(f.can.tx_fail_count == 1 && f.can.tx_full_count == 1);
    f.can.service(9);
    assert(f.handle.abort_calls == 0);
    f.can.service(10);
    assert(f.handle.last_abort_mask == 7 && f.handle.pending == 0);
    mock_tick = 11;
    f.can.tx_data[0] = 9;
    assert(f.can.send(0x200) == HAL_OK);
    assert(f.handle.last_data[0] == 9 && f.can.tx_queued_count == 4);
  }
  { // Completed commands are never mistaken for a stalled mailbox.
    Fixture f;
    assert(f.can.send(0x200) == HAL_OK);
    f.handle.pending = 0;
    f.can.service(100);
    assert(f.handle.abort_calls == 0);
  }
  { // Expire only the old mailbox, preserving a more recent command.
    Fixture f;
    f.can.send(0x1FE);
    mock_tick = 8;
    f.can.send(0x200);
    f.can.service(10);
    assert(f.handle.last_abort_mask == 1 && f.handle.pending == 2);
  }
  { // Failed aborts retry at a bounded rate instead of spinning every tick.
    Fixture f;
    f.can.send(0x200);
    f.handle.abort_result = HAL_ERROR;
    f.can.service(10);
    f.can.service(11);
    assert(f.handle.abort_calls == 1 && f.can.last_abort_status == HAL_ERROR);
    f.handle.abort_result = HAL_OK;
    f.can.service(20);
    assert(f.handle.abort_calls == 2 && f.can.tx_abort_count == 1);
  }
  { // Timestamp arithmetic remains correct when the 32-bit clock wraps.
    Fixture f;
    mock_tick = UINT32_MAX - 4;
    f.can.send(0x200);
    f.can.service(4);
    assert(f.handle.abort_calls == 0);
    f.can.service(5);
    assert(f.handle.abort_calls == 1);
  }
  { // A CAN1-style transport failure does not modify another transport.
    Fixture a, b;
    a.handle.pending = 7;
    assert(a.can.send(0x200) == HAL_BUSY);
    assert(b.can.send(0x1FE) == HAL_OK);
    assert(b.can.tx_fail_count == 0);
    a.handle.pending = 0;
    a.handle.add_result = HAL_ERROR;
    assert(a.can.send(0x200) == HAL_ERROR && a.can.tx_fail_count == 2);
    a.can.service(20);
    assert(a.handle.abort_calls == 0);
  }
  { // Hardware error snapshots retain evidence after Bus-Off clears.
    Fixture f;
    f.regs.ESR = CAN_ESR_BOFF;
    f.can.service(0);
    f.can.service(1);
    assert(f.can.bus_off_count == 1 && f.can.error_status == CAN_ESR_BOFF);
    f.regs.ESR = 0;
    f.can.service(2);
    f.regs.ESR = CAN_ESR_BOFF;
    f.can.service(3);
    assert(f.can.bus_off_count == 2);
  }
  { // Short/error receive results cannot reuse the prior valid frame length.
    Fixture f;
    mock_tick = 42;
    f.can.recv();
    assert(f.can.rx_len == 8 && f.can.rx_count == 1 && f.can.last_rx_ms == 42);
    f.handle.rx_header.DLC = 2;
    f.can.recv();
    assert(f.can.rx_len == 2);
    f.handle.rx_result = HAL_ERROR;
    f.can.recv();
    assert(f.can.rx_len == 0 && f.can.rx_id == UINT32_MAX && f.can.rx_count == 2);
  }
  { // Extended frames share the same failure/recovery accounting.
    Fixture f;
    assert(f.can.send_ext(1, 0x1234, 3, 0) == HAL_OK);
    assert(f.handle.last_header.IDE == CAN_ID_EXT);
    assert(f.handle.last_header.ExtId == 0x01123403);
    f.can.service(10);
    assert(f.can.tx_abort_count == 1);
    f.handle.state = HAL_CAN_STATE_RESET;
    assert(f.can.send(0x200) == HAL_ERROR);
  }
  std::cout << "CAN transport: 9 failure/recovery scenarios passed\n";
}
