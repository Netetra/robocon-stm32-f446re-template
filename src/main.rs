#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device=stm32f4xx_hal::pac, peripherals=true, dispatchers=[USART6])]
mod app {
    use bxcan::{filter::Mask32, Frame, Id, Rx0, Rx1, StandardId, Tx};
    use defmt::*;
    use stm32f4xx_hal::{
        can::Can,
        gpio::{Output, Pin},
        pac::{CAN1, TIM7},
        prelude::*,
        rcc::RccExt,
        timer::{Counter, Event},
    };
    use systick_monotonic::{ExtU64, Systick};

    static TICK_INTERVAL_MS: u32 = 1000;
    static LED_BLINK_TIME_MS: u64 = 25;

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1_000>;

    #[shared]
    struct Shared {
        tx_led: Pin<'A', 0, Output>,
        rx_led: Pin<'A', 1, Output>,
    }

    #[local]
    struct Local {
        tick: Counter<TIM7, 1_000>,
        tx: Tx<Can<CAN1>>,
        rx0: Rx0<Can<CAN1>>,
        rx1: Rx1<Can<CAN1>>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let rcc = ctx.device.RCC.constrain();
        let clocks = rcc
            .cfgr
            // .use_hse(16.MHz())
            .sysclk(64.MHz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();

        let mut bxcan = bxcan::Can::builder(ctx.device.CAN1.can((gpiob.pb9, gpiob.pb8)))
            .set_bit_timing(0x001c_0003)
            .set_automatic_retransmit(true)
            .set_loopback(true)
            .set_silent(false)
            .leave_disabled();
        let mut filters = bxcan.modify_filters();
        filters.enable_bank(0, bxcan::Fifo::Fifo0, Mask32::accept_all());
        filters.enable_bank(0, bxcan::Fifo::Fifo1, Mask32::accept_all());
        drop(filters);
        bxcan.enable_interrupt(bxcan::Interrupt::Fifo0MessagePending);
        bxcan.enable_interrupt(bxcan::Interrupt::Fifo1MessagePending);
        // 11個の連続したレセッシブビットを受け取ると有効化される
        nb::block!(bxcan.enable_non_blocking()).unwrap();
        let (tx, rx0, rx1) = bxcan.split();
        info!("bxCAN initialized.");

        let tx_led = gpioa.pa0.into_push_pull_output();
        let rx_led = gpioa.pa1.into_push_pull_output();
        info!("LED initialized.");

        let mut tick = ctx.device.TIM7.counter_ms(&clocks);
        tick.start(TICK_INTERVAL_MS.millis()).unwrap();
        tick.listen(Event::Update);
        info!("Ticker initialized.");

        let systick = ctx.core.SYST.monotonic(&clocks);
        info!("SysTick initialized.");

        (
            Shared { tx_led, rx_led },
            Local { tick, tx, rx0, rx1 },
            init::Monotonics(systick),
        )
    }

    #[task(binds=TIM7, local=[tick])]
    fn tick(ctx: tick::Context) {
        let frame = Frame::new_data(StandardId::new(0x000).unwrap(), [0, 1, 2]);
        let _ = transmit::spawn(frame);

        // Unset flag.
        ctx.local.tick.wait().unwrap();
    }

    #[task(local=[tx], shared=[tx_led])]
    fn transmit(mut ctx: transmit::Context, frame: Frame) {
        ctx.shared.tx_led.lock(|tx_led| {
            tx_led.set_high();
            let _ = tx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());
            nb::block!(ctx.local.tx.transmit(&frame)).unwrap();
        });
    }

    #[task(binds=CAN1_RX0, local=[rx0], shared=[rx_led])]
    fn receive_rx0(mut ctx: receive_rx0::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());

            let recv = nb::block!(ctx.local.rx0.receive());
            if let Ok(frame) = recv {
                if let Id::Standard(id) = frame.id() {
                    debug!("id: {}, data: {}", id.as_raw(), frame.data());
                    // do something.
                }
            }
        });
    }

    #[task(binds=CAN1_RX1, local=[rx1], shared=[rx_led])]
    fn receive_rx1(mut ctx: receive_rx1::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());

            let recv = nb::block!(ctx.local.rx1.receive());
            if let Ok(frame) = recv {
                if let Id::Standard(id) = frame.id() {
                    debug!("id: {}, data: {}", id.as_raw(), frame.data());
                    // do something.
                }
            }
        });
    }

    #[task(shared=[tx_led])]
    fn tx_led_off(mut ctx: tx_led_off::Context) {
        ctx.shared.tx_led.lock(|tx_led| {
            tx_led.set_low();
        });
    }

    #[task(shared=[rx_led])]
    fn rx_led_off(mut ctx: rx_led_off::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_low();
        });
    }
}
