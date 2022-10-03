#![no_std]
#![no_main]

use core::fmt::Write;

use bsp::{
    entry,
    hal::uart::{self, UartPeripheral},
};
use defmt::*;
use defmt_serial as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    gpio, pac,
    pio::{PIOBuilder, PIOExt},
    pwm,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::v2::OutputPin;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let _core = pac::CorePeripherals::take().unwrap();
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .unwrap();

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let uart_tx: rp_pico::Gp0Uart0Tx = pins.gpio0.into_mode();
    let uart_rx: rp_pico::Gp1Uart0Rx = pins.gpio1.into_mode();
    let uart_pins = (uart_tx, uart_rx);
    let uart = UartPeripheral::new(pac.UART0, uart_pins, &mut pac.RESETS)
        .enable(
            uart::common_configs::_115200_8_N_1,
            clocks.peripheral_clock.freq(),
        )
        .unwrap();
    defmt_serial::defmt_serial(uart);

    let _blink_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio6.into_mode();
    let blink_pin_id = 6;

    let _pio_debug_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio7.into_mode();
    let _pio_debug_pin_id = 7;

    let (mut pio0, sm0, _, _, _) = pac.PIO0.split(&mut pac.RESETS);

    let pulse_width_program = {
        use pio::JmpCondition::{PinHigh, XDecNonZero};
        use pio::SetDestination::{PINDIRS, PINS};
        use pio::WaitSource::PIN;
        use pio::{
            MovDestination::{ISR, X as XDst},
            MovOperation::{Invert, None},
            MovSource::{NULL, X as XSrc},
        };

        let mut a = pio::Assembler::new();
        let mut wrap_target = a.label();
        let mut wrap_source = a.label();

        let mut x_not_zero = a.label();

        let mut wait_loop_top = a.label();
        let mut wait_loop_bottom = a.label();

        a.set(PINDIRS, 0b10); // Set pin 0 to input and pin 1 to output
        a.bind(&mut wrap_target);
        // Initialize registers
        a.mov(XDst, Invert, NULL); // Fill X with ones
        a.wait(1, PIN, 0, false); // Wait for pin 0 to go high

        // Count number of "peripheral cycles" that pin is high
        a.bind(&mut wait_loop_top);
        a.set(PINS, 0b10);
        a.jmp(XDecNonZero, &mut x_not_zero); // Decrement X
        a.bind(&mut x_not_zero);
        a.jmp(PinHigh, &mut wait_loop_top); // If pin is still high, keep waiting
        a.bind(&mut wait_loop_bottom);

        a.set(PINS, 0b00);
        // Move counter to CPU
        a.mov(ISR, None, XSrc); // Move content of X register to ISR
        a.push(false, false); // Push to CPU

        a.bind(&mut wrap_source); // Wrap

        a.assemble_with_wrap(wrap_source, wrap_target)
    };
    let installed = pio0.install(&pulse_width_program).unwrap();
    let (sm, mut sm_rx, _tx) = PIOBuilder::from_program(installed)
        .set_pins(blink_pin_id, 3)
        .in_pin_base(blink_pin_id)
        .jmp_pin(blink_pin_id)
        .clock_divisor(2.0)
        .build(sm0);
    sm.start();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm = pwm_slices.pwm1;
    pwm.set_div_int(14);
    pwm.set_div_frac(5);
    pwm.set_ph_correct();
    pwm.enable();

    let mut motor_pwm_channel = pwm.channel_a;
    let _led_pwm_channel_pin = motor_pwm_channel.output_to(pins.gpio2);

    let mut motor_ctrl_0 = pins.gpio3.into_push_pull_output();
    let mut motor_ctrl_1 = pins.gpio4.into_push_pull_output();

    motor_ctrl_0.set_high().unwrap();
    motor_ctrl_1.set_low().unwrap();

    motor_pwm_channel.enable();

    use embedded_hal::PwmPin;
    motor_pwm_channel.set_duty(0);

    loop {
        let rx_val = sm_rx.read();
        match rx_val {
            Some(raw_value) => {
                let min_cycles = 20300;
                let max_cycles = 42000;
                let cycles = ((u32::MAX - raw_value) as u16).max(min_cycles);
                let max_delta = (max_cycles - min_cycles).max(1);
                let delta = cycles - min_cycles;
                let increment = 0xFFFF / max_delta;
                match delta.checked_mul(increment) {
                    Some(duty) => {
                        motor_pwm_channel.set_duty(duty);
                        info!("Duty: {}\r\n", duty);
                    }
                    None => motor_pwm_channel.set_duty(u16::MAX),
                };
            }
            _ => (),
        }
    }
}

// End of file
