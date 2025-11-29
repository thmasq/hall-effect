#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
              holding buffers for the duration of a data transfer."
)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_hal::analog::adc::{Adc, AdcCalCurve, AdcConfig, Attenuation};
use esp_hal::clock::CpuClock;
use esp_hal::gpio::Level;
use esp_hal::rmt::{PulseCode, Rmt, TxChannelConfig, TxChannelCreator};
use esp_hal::time::Rate;
use esp_hal::timer::timg::TimerGroup;
use nb;
use panic_rtt_target as _;
use smart_leds::RGB8;

// This creates a default app-descriptor required by the esp-idf bootloader.
esp_bootloader_esp_idf::esp_app_desc!();

// WS2812 timing constants (in nanoseconds)
const CODE_PERIOD_NS: u32 = 1250; // 800kHz
const T0H_NS: u32 = 400;
const T0L_NS: u32 = CODE_PERIOD_NS - T0H_NS;
const T1H_NS: u32 = 850;
const T1L_NS: u32 = CODE_PERIOD_NS - T1H_NS;

// Buffer size for one RGB LED (24 pulses + 1 delimiter)
const BUFFER_SIZE: usize = 25;

const MIN_VOLTAGE_MV: f32 = 500.0; // ~0.5V for strong north pole
const MAX_VOLTAGE_MV: f32 = 2800.0; // ~2.8V for strong south pole

fn led_pulses_for_clock(src_clock_mhz: u32) -> (PulseCode, PulseCode) {
    (
        PulseCode::new(
            Level::High.into(),
            ((T0H_NS * src_clock_mhz) / 1000) as u16,
            Level::Low.into(),
            ((T0L_NS * src_clock_mhz) / 1000) as u16,
        ),
        PulseCode::new(
            Level::High.into(),
            ((T1H_NS * src_clock_mhz) / 1000) as u16,
            Level::Low.into(),
            ((T1L_NS * src_clock_mhz) / 1000) as u16,
        ),
    )
}

fn ws2812_encode(
    color: RGB8,
    pulses: (PulseCode, PulseCode),
    rmt_buffer: &mut [PulseCode; BUFFER_SIZE],
) {
    let bytes = [color.g, color.r, color.b];
    let mut idx = 0;

    for &byte in bytes.iter() {
        for bit in (0..8).rev() {
            let is_set = (byte & (1 << bit)) != 0;
            rmt_buffer[idx] = if is_set { pulses.1 } else { pulses.0 };
            idx += 1;
        }
    }
    rmt_buffer[24] = PulseCode::new(Level::Low.into(), 0, Level::Low.into(), 0); // Delimiter
}

fn voltage_to_color(voltage_mv: u32) -> RGB8 {
    let v = voltage_mv as f32;
    let t = if v <= MIN_VOLTAGE_MV {
        0.0
    } else if v >= MAX_VOLTAGE_MV {
        1.0
    } else {
        (v - MIN_VOLTAGE_MV) / (MAX_VOLTAGE_MV - MIN_VOLTAGE_MV)
    };
    let r = (255.0 * (1.0 - t)) as u8; // Red for low voltage (north)
    let b = (255.0 * t) as u8; // Blue for high voltage (south)
    RGB8::new(r, 0, b)
}

#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 0.6.0
    rtt_target::rtt_init_defmt!();

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0);

    info!("Embassy initialized!");

    // Initialize ADC for hall effect sensor on GPIO4
    let mut adc_config = AdcConfig::new();
    let analog_pin = peripherals.GPIO4;
    let mut adc_pin =
        adc_config.enable_pin_with_cal::<_, AdcCalCurve<_>>(analog_pin, Attenuation::_6dB);
    let mut adc = Adc::new(peripherals.ADC1, adc_config);

    // Initialize RMT for WS2812 control
    let rmt = Rmt::new(peripherals.RMT, Rate::from_mhz(80)).unwrap();
    let tx_config = TxChannelConfig::default()
        .with_clk_divider(1)
        .with_idle_output_level(Level::Low)
        .with_carrier_modulation(false)
        .with_idle_output(true);
    let channel_creator = rmt.channel0;
    let mut channel = channel_creator
        .configure_tx(peripherals.GPIO48, tx_config)
        .unwrap();

    // Precompute pulses based on actual clock
    let src_clock_mhz = esp_hal::clock::Clocks::get().apb_clock.as_mhz();
    let pulses = led_pulses_for_clock(src_clock_mhz);

    info!("WS2812 LED initialized on GPIO48, ADC on GPIO4");

    let _ = spawner;

    let mut rmt_buffer = [PulseCode::default(); BUFFER_SIZE];

    loop {
        let raw: u16 = nb::block!(adc.read_oneshot(&mut adc_pin)).unwrap();
        let voltage_mv = ((raw as f32 / 4095.0) * 3300.0) as u32;
        let color = voltage_to_color(voltage_mv);
        ws2812_encode(color, pulses, &mut rmt_buffer);

        let transaction = channel.transmit(&rmt_buffer).unwrap();
        channel = transaction.wait().unwrap();

        info!(
            "Voltage: {}mV, LED color: R={}, G={}, B={}",
            voltage_mv, color.r, color.g, color.b
        );

        Timer::after(Duration::from_millis(10)).await;
    }
}
