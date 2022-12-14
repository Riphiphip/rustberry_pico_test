#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
// use defmt_serial as _;
use defmt_rtt as _;
use panic_probe as _;

use micromath::F32Ext;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::init_clocks_and_plls,
    gpio, pac,
    pio::{PIOBuilder, PIOExt},
    pwm,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::v2::OutputPin;

/** Parses a raw value read from the rc channel PIO peripheral
   into a value that can be used as a duty cycle for a pwm channel.
*/
fn rc_chan_to_pwm(raw_value: u32, min: u16, max: u16) -> u16 {
    let min_cycles = min;
    let max_cycles = max;
    let cycles = ((u32::MAX - raw_value) as u16).max(min_cycles);
    let max_delta = (max_cycles - min_cycles).max(1);
    let delta = cycles - min_cycles;
    let increment = 0xFFFF / max_delta;
    match delta.checked_mul(increment) {
        Some(duty) => duty,
        None => u16::MAX,
    }
}

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().expect("Could not take peripherals");
    let _core = pac::CorePeripherals::take().expect("Could not take core peripherals");
    let mut watchdog = Watchdog::new(pac.WATCHDOG);
    let sio = Sio::new(pac.SIO);

    // External high-speed crystal on the pico board is 12Mhz
    let external_xtal_freq_hz = 12_000_000u32;
    let _clocks = init_clocks_and_plls(
        external_xtal_freq_hz,
        pac.XOSC,
        pac.CLOCKS,
        pac.PLL_SYS,
        pac.PLL_USB,
        &mut pac.RESETS,
        &mut watchdog,
    )
    .ok()
    .expect("Could not set up clocks");

    let pins = bsp::Pins::new(
        pac.IO_BANK0,
        pac.PADS_BANK0,
        sio.gpio_bank0,
        &mut pac.RESETS,
    );

    let _rc_chan_0_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio0.into_mode();
    let rc_chan_0_pin_id = 0;

    let _rc_chan_1_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio1.into_mode();
    let rc_chan_1_pin_id = 1;

    let _rc_chan_2_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio2.into_mode();
    let rc_chan_2_pin_id = 2;

    let _rc_chan_3_pin: gpio::Pin<_, gpio::FunctionPio0> = pins.gpio3.into_mode();
    let rc_chan_3_pin_id = 3;

    let (mut pio0, sm0, sm1, sm2, sm3) = pac.PIO0.split(&mut pac.RESETS);

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

    let installed = pio0
        .install(&pulse_width_program)
        .expect("Could not install to PIO0");

    let (sm0_init, mut rc_chan_0, _) = unsafe {
        PIOBuilder::from_program(installed.share())
            .in_pin_base(rc_chan_0_pin_id)
            .jmp_pin(rc_chan_0_pin_id)
            .clock_divisor(2.0)
            .build(sm0)
    };

    let (sm1_init, mut rc_chan_1, _) = unsafe {
        PIOBuilder::from_program(installed.share())
            .in_pin_base(rc_chan_1_pin_id)
            .jmp_pin(rc_chan_1_pin_id)
            .clock_divisor(2.0)
            .build(sm1)
    };

    let (sm2_init, mut rc_chan_2, _) = unsafe {
        PIOBuilder::from_program(installed.share())
            .in_pin_base(rc_chan_2_pin_id)
            .jmp_pin(rc_chan_2_pin_id)
            .clock_divisor(2.0)
            .build(sm2)
    };

    let (sm3_init, mut rc_chan_3, _) = unsafe {
        PIOBuilder::from_program(installed.share())
            .in_pin_base(rc_chan_3_pin_id)
            .jmp_pin(rc_chan_3_pin_id)
            .clock_divisor(2.0)
            .build(sm3)
    };

    let _sm_group = sm0_init
        .with(sm1_init)
        .with(sm2_init)
        .with(sm3_init)
        .start();

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm0 = pwm_slices.pwm0;
    pwm0.set_div_int(14);
    pwm0.set_div_frac(5);
    pwm0.set_ph_correct();
    pwm0.enable();

    let mut motor_r_pwm_chan = pwm0.channel_a;
    motor_r_pwm_chan.output_to(pins.gpio16);

    let mut pwm2 = pwm_slices.pwm2;
    pwm2.set_div_int(14);
    pwm2.set_div_frac(5);
    pwm2.set_ph_correct();
    pwm2.enable();

    let mut motor_l_pwm_chan = pwm2.channel_a;
    motor_l_pwm_chan.output_to(pins.gpio4);

    let mut magnet_l_pwm_chan = pwm2.channel_b;
    magnet_l_pwm_chan.output_to(pins.gpio21);

    let mut pwm4 = pwm_slices.pwm4;
    pwm4.set_div_int(14);
    pwm4.set_div_frac(5);
    pwm4.set_ph_correct();
    pwm4.enable();

    let mut magnet_r_pwm_chan = pwm4.channel_b;
    magnet_r_pwm_chan.output_to(pins.gpio9);

    let mut motor_l_ctrl_0 = pins.gpio5.into_push_pull_output();
    let mut motor_l_ctrl_1 = pins.gpio6.into_push_pull_output();
    motor_l_ctrl_0
        .set_low()
        .expect("Failed to set motor ctrl 0 low");
    motor_l_ctrl_1
        .set_low()
        .expect("Failed to set motor ctrl 1 low");

    let mut motor_r_ctrl_0 = pins.gpio17.into_push_pull_output();
    let mut motor_r_ctrl_1 = pins.gpio18.into_push_pull_output();
    motor_r_ctrl_0
        .set_low()
        .expect("Failed to set motor ctrl 0 low");
    motor_r_ctrl_1
        .set_low()
        .expect("Failed to set motor ctrl 1 low");

    let mut magnet_l_ctrl_0 = pins.gpio19.into_push_pull_output();
    let mut magnet_l_ctrl_1 = pins.gpio20.into_push_pull_output();
    magnet_l_ctrl_0
        .set_high()
        .expect("Failed to set motor ctrl 0 high");
    magnet_l_ctrl_1
        .set_low()
        .expect("Failed to set motor ctrl 1 low");

    let mut magnet_r_ctrl_0 = pins.gpio8.into_push_pull_output();
    let mut magnet_r_ctrl_1 = pins.gpio7.into_push_pull_output();
    magnet_r_ctrl_0
        .set_high()
        .expect("Failed to set motor ctrl 0 high");
    magnet_r_ctrl_1
        .set_low()
        .expect("Failed to set motor ctrl 1 low");

    use embedded_hal::PwmPin;
    motor_l_pwm_chan.set_duty(0);
    motor_r_pwm_chan.set_duty(0);
    magnet_l_pwm_chan.set_duty(0);
    magnet_r_pwm_chan.set_duty(0);

    let mut magnet_pwm_duty = 0;

    struct StickPos {
        x: f32,
        y: f32,
    }

    let mut current_pos = StickPos { x: 0.0, y: 0.0 };

    loop {
        let rx_val = rc_chan_0.read();
        match rx_val {
            Some(raw_value) => {
                current_pos.x = rc_chan_to_pwm(raw_value, 20300, 42000).into();
                current_pos.x -= (u16::MAX / 2) as f32;
                if current_pos.x < 100.0 {
                    current_pos.x = 0.0;
                }
            }
            None => {}
        }

        let rx_val = rc_chan_1.read();
        match rx_val {
            Some(raw_value) => {
                current_pos.y = rc_chan_to_pwm(raw_value, 20300, 42000).into();
                current_pos.y -= (u16::MAX / 2) as f32;
                if current_pos.y < 100.0 {
                    current_pos.y = 0.0;
                }
            }
            None => {}
        }

        let rx_val = rc_chan_2.read();
        match rx_val {
            Some(raw_value) => {
                magnet_pwm_duty = rc_chan_to_pwm(raw_value, 20300, 42000);
            }
            None => {}
        }

        let rx_val = rc_chan_3.read();
        match rx_val {
            Some(raw_value) => {
                let switch_reading = rc_chan_to_pwm(raw_value, 20300, 42000);
                if switch_reading > u16::MAX / 2 {
                    magnet_l_pwm_chan.set_duty(magnet_pwm_duty);
                    magnet_r_pwm_chan.set_duty(magnet_pwm_duty);
                } else {
                    magnet_l_pwm_chan.set_duty(0);
                    magnet_r_pwm_chan.set_duty(0);
                }
            }
            None => {}
        }

        // Set motors
        motor_l_ctrl_0.set_high().unwrap();
        motor_l_pwm_chan.set_duty(unsafe { current_pos.x.to_int_unchecked() });
    }
}

// End of file
