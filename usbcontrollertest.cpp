#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/watchdog.h"
#include <gamepad.h>
#include "hardware/gpio.h"
#include "pico/multicore.h"

#define PIN_DATA  27  // RP2040 output to NES controller port
#define PIN_LATCH 28  // NES input: Latch (high pulse starts report)
#define PIN_CLOCK 29  // NES input: Clock (falling edge shifts bit)

#define PIN_DATA_2  14  // RP2040 output to NES controller port
#define PIN_LATCH_2 15  // NES input: Latch (high pulse starts report)
#define PIN_CLOCK_2 26  // NES input: Clock (falling edge shifts bit)

volatile uint8_t nes_button_state = 0xFF; // Populated externally
volatile uint8_t bit_index = 0;
volatile uint8_t nes_button_state_2 = 0xFF; 
volatile uint8_t bit_index_2 = 0;

#if CFG_TUH_RPI_PIO_USB
#include "bsp/board_api.h"
#endif
#include "tusb.h"

typedef unsigned long DWORD;
void InfoNES_PadState(DWORD *pdwPad1, DWORD *pdwPad2, DWORD *pdwSystem)
{
    static constexpr int LEFT = 1 << 6;
    static constexpr int RIGHT = 1 << 7;
    static constexpr int UP = 1 << 4;
    static constexpr int DOWN = 1 << 5;
    static constexpr int SELECT = 1 << 2;
    static constexpr int START = 1 << 3;
    static constexpr int A = 1 << 0;
    static constexpr int B = 1 << 1;

    static DWORD prevButtons[2]{};
    static int rapidFireMask[2]{};
    static int rapidFireCounter = 0;

    ++rapidFireCounter;
    bool reset = false;

    for (int i = 0; i < 2; ++i)
    {
        auto &dst = i == 0 ? *pdwPad1 : *pdwPad2;
        auto &gp = io::getCurrentGamePadState(i);

        int v = (gp.buttons & io::GamePadState::Button::LEFT ? LEFT : 0) |
                (gp.buttons & io::GamePadState::Button::RIGHT ? RIGHT : 0) |
                (gp.buttons & io::GamePadState::Button::UP ? UP : 0) |
                (gp.buttons & io::GamePadState::Button::DOWN ? DOWN : 0) |
                (gp.buttons & io::GamePadState::Button::A ? A : 0) |
                (gp.buttons & io::GamePadState::Button::B ? B : 0) |
                (gp.buttons & io::GamePadState::Button::SELECT ? SELECT : 0) |
                (gp.buttons & io::GamePadState::Button::START ? START : 0) |
                0;

        int rv = v;
        if (rapidFireCounter & 2)
        {
            // 15 fire/sec
            rv &= ~rapidFireMask[i];
        }

        dst = rv;

        auto p1 = v;
        auto pushed = v & ~prevButtons[i];

        prevButtons[i] = v;

        if(i == 0)
            nes_button_state = static_cast<int8_t>(~v & 0xFF);
        else
            nes_button_state_2 = static_cast<int8_t>(~v & 0xFF);

        //comment return for showing buttons states via serial console (gpio0)
        return;    
        
        if (pushed)
        {
            printf("%8b\n", nes_button_state);
        }
        if (pushed & LEFT)
        {
            printf("LEFT\n");
        }
        if (pushed & RIGHT)
        {
            printf("RIGHT\n");
        }
        if (pushed & START)
        {
            printf("START\n");
        }
        if (pushed & SELECT)
        {
            printf("SELECT\n");
        }
        if (pushed & A)
        {
            printf("A\n");
        }
        if (pushed & B)
        {
            printf("B\n");
        }
        if (pushed & UP)
        {
            printf("UP\n");
            //    watchdog_enable(500, 1);
            //    printf("Rebooting\n");
        }
        else if (pushed & DOWN)
        {
            printf("DOWN\n");
        }

    }
}

void nes_gpio_isr(uint gpio, uint32_t events) {
    if (gpio == PIN_LATCH) {
        // if (events & GPIO_IRQ_EDGE_RISE) {
            bit_index = 0;
            gpio_put(PIN_DATA, (nes_button_state >> bit_index) & 1);
        // }
    } else if (gpio == PIN_CLOCK) {
        // if (events & GPIO_IRQ_EDGE_FALL) {
            bit_index++;
            if (bit_index < 8) {
                gpio_put(PIN_DATA, (nes_button_state >> bit_index) & 1);
            } else {
                gpio_put(PIN_DATA, 1); // NES reads high after 8 bits
            }
        // }
    }
    else if (gpio == PIN_LATCH_2) {
        if (events & GPIO_IRQ_EDGE_RISE) {
            bit_index_2 = 0;
            gpio_put(PIN_DATA_2, (nes_button_state_2 >> bit_index_2) & 1);
        }
    } else if (gpio == PIN_CLOCK_2) {
        if (events & GPIO_IRQ_EDGE_FALL) {
            bit_index_2++;
            if (bit_index_2 < 8) {
                gpio_put(PIN_DATA_2, (nes_button_state_2 >> bit_index_2) & 1);
            } else {
                gpio_put(PIN_DATA_2, 1); // NES reads high after 8 bits
            }
        }
    }
}

void nes_joystick_init(void) {
    gpio_init(PIN_DATA);
    gpio_set_dir(PIN_DATA, GPIO_OUT);
    gpio_put(PIN_DATA, 1);

    gpio_init(PIN_LATCH);
    gpio_set_dir(PIN_LATCH, GPIO_IN);
    
    gpio_init(PIN_CLOCK);
    gpio_set_dir(PIN_CLOCK, GPIO_IN);

    gpio_set_irq_enabled_with_callback(PIN_LATCH, GPIO_IRQ_EDGE_RISE, true, &nes_gpio_isr);
    gpio_set_irq_enabled(PIN_CLOCK, GPIO_IRQ_EDGE_RISE, true);

    irq_set_enabled(IO_IRQ_BANK0, true);
}

void core1_entry(void) {
    nes_joystick_init();
    while (true) {
    }
}

int main()
{
   
    stdio_init_all();
    printf("Started.\n");

#if CFG_TUH_RPI_PIO_USB
    printf("Using PIO USB.\n");
    board_init();
    tusb_rhport_init_t host_init = {
        .role = TUSB_ROLE_HOST,
        .speed = TUSB_SPEED_AUTO};
    tusb_init(BOARD_TUH_RHPORT, &host_init);

    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }
#else
    printf("Using internal USB.\n");
    tuh_init(BOARD_TUH_RHPORT) ;
    // tusb_init();
#endif

    multicore_launch_core1(core1_entry);

    while (true)
    {
        // printf("Hello, world!\n");
        // sleep_ms(1000 / 60);
        tuh_task();
        DWORD pdwPad1, pdwPad2, pdwSystem;
        InfoNES_PadState(&pdwPad1, &pdwPad2, &pdwSystem);
    }
}
