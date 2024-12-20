#include <stdint.h>

extern uint32_t _etext, _sdata, _edata, _la_data, _sbss, _ebss;
// _sdata == end of .text in ram ( _e(nd)text )
// _sdata == start of .data in ram ( _s(tart)data )
// _edata == end of .data in ram ( _e(nd)data )
// _la_data == load address of .data in flash ( _l(oad)a(ddress)_data )
// _sbss == start of .bss in ram ( _s(tart)bss )
// _ebss == end of .bss in ram ( _e(nd)bss )

extern unsigned SRAM_END;
#define STACK_POINTER_INIT_ADDRESS (SRAM_END)
int main(void);

void null_handler(void);
void reset_handler(void);
void default_handler(void);
void nmi_handler(void) __attribute__((weak, alias("default_handler")));
void hard_fault_handler(void) __attribute__((weak, alias("default_handler")));
void bus_fault_handler(void) __attribute__((weak, alias("default_handler")));
void memory_manage_fault_handler(void)
    __attribute__((weak, alias("default_handler")));
void usage_fault_handler(void) __attribute__((weak, alias("default_handler")));
void sv_call_handler(void) __attribute__((weak, alias("default_handler")));
void debug_monitor_handler(void)
    __attribute__((weak, alias("default_handler")));
void pend_sv_handler(void) __attribute__((weak, alias("default_handler")));
void systick_handler(void) __attribute__((weak, alias("default_handler")));

// defined in page 198 of reference manual
// struct here acts a list of memory addresses,
// should probably be changed to an array, idk
typedef struct VectorTable {
  unsigned int *sp_init;
  void (*reset)(void);
  void (*nmi)(void);
  void (*hard_fault)(void);
  void (*memory_manage_fault)(void);
  void (*bus_fault)(void);
  void (*usage_fault)(void);
  // pointer to an array of function pointers
  void (*reserved_x001c[4])(void);
  void (*sv_call)(void);
  void (*debug_monitor)(void);
  void (*reserved_x0034)(void);
  void (*pend_sv)(void);
  void (*systick)(void);
  void (*irq[68])(void);
} VectorTable;

__attribute__((section(".vector_table"))) VectorTable vector_table = {
    .sp_init = &STACK_POINTER_INIT_ADDRESS,
    .reset = reset_handler,
    .nmi = nmi_handler,
    .hard_fault = hard_fault_handler,
    .memory_manage_fault = memory_manage_fault_handler,
    .bus_fault = bus_fault_handler,
    .sv_call = sv_call_handler,
    .debug_monitor = debug_monitor_handler,
    .pend_sv = pend_sv_handler,
    .systick = systick_handler,
    .irq = {
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler, null_handler, null_handler,
        null_handler, null_handler, null_handler,
    }};

void reset_handler(void) {
  // Copy .data section from flash to RAM
  uint32_t data_size = (uint32_t)&_edata - (uint32_t)&_sdata;
  uint8_t *flash_data = (uint8_t*) &_etext;
  uint8_t *sram_data = (uint8_t*) &_sdata;

  for (uint32_t i = 0; i < data_size; i++)
  {
    sram_data[i] = flash_data[i];
  }

  // Zero-fill .bss section in SRAM
  uint32_t bss_size = (uint32_t)&_ebss - (uint32_t)&_sbss;
  uint8_t *bss = (uint8_t*) &_sbss;

  for (uint32_t i = 0; i < bss_size; i++)
  {
    bss[i] = 0;
  }

  // Your super cool code starts here :^)
  main();
}

void default_handler(void) {
  while (1) {
  };
}

void null_handler(void) {}
