#![no_std]
#![no_main]

use bsp::entry;
use defmt::*;
use defmt_rtt as _;
use panic_probe as _;

// Provide an alias for our BSP so we can switch targets quickly.
// Uncomment the BSP you included in Cargo.toml, the rest of the code does not need to change.
use rp_pico as bsp;
// use sparkfun_pro_micro_rp2040 as bsp;

use bsp::hal::{
    clocks::{init_clocks_and_plls, Clock},
    pac, pwm,
    sio::Sio,
    watchdog::Watchdog,
};

use embedded_hal::digital::v2::OutputPin;

#[entry]
fn main() -> ! {
    info!("Program start");
    let mut pac = pac::Peripherals::take().unwrap();
    let core = pac::CorePeripherals::take().unwrap();
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

    let pwm_slices = pwm::Slices::new(pac.PWM, &mut pac.RESETS);

    let mut pwm = pwm_slices.pwm1;
    pwm.set_div_int(19);
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
    motor_pwm_channel.set_duty(0xffff / 2);
    let mut delay = cortex_m::delay::Delay::new(core.SYST, clocks.system_clock.freq().to_Hz());

    loop {
        info!("on!");

        motor_pwm_channel.set_duty(0xffff / 2);
        delay.delay_ms(500);
        info!("off!");

        motor_pwm_channel.set_duty(0xffff);
        delay.delay_ms(500);
    }
}

// End of file
