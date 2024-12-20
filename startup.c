extern unsigned int _sdata;   // Start of .data in RAM
extern unsigned int _edata;   // End of .data in RAM
extern unsigned int _la_data; // Load address of .data in FLASH
extern unsigned int _sbss;    // Start of .bss in RAM
extern unsigned int _ebss;    // End of .bss in RAM
                              //
int main(void);
void reset_handler(void) {
  unsigned int *src, *dst;

  // Copy .data section from flash to RAM
  src = &_la_data;
  dst = &_sdata;
  while (dst < &_edata) {
    *(dst++) = *(src++);
  }

  // Zero initialize .bss section in RAM
  dst = &_sbss;
  while (dst < &_ebss) {
    *(dst++) = 0;
  }
  main();
}

void blocking_handler(void) {
  while (1)
    ;
}
void null_handler(void) {}
extern unsigned _stack;

__attribute__((section(".vectors"))) struct {
  unsigned int *initial_sp_value;
  void (*reset)(void);
  void (*nmi)(void);
  void (*hard_fault)(void);
  void (*memory_manage_fault)(void);
  void (*bus_fault)(void);
  void (*usage_fault)(void);
  void (*reserved_x001c[4])(void);
  void (*sv_call)(void);
  void (*debug_monitor)(void);
  void (*reserved_x0034)(void);
  void (*pend_sv)(void);
  void (*systick)(void);
  void (*irq[68])(void);
} vector_table = {
    .initial_sp_value = &_stack,
    .reset = reset_handler,
    .nmi = null_handler,
    .hard_fault = blocking_handler,

    .sv_call = null_handler,
    .pend_sv = null_handler,
    .systick = null_handler,
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
