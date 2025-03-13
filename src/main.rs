#![no_main]
#![no_std]

use defmt_rtt as _;
use panic_probe as _;

#[rtic::app(device=stm32f4xx_hal::pac, peripherals=true, dispatchers=[USART6])]
mod app {
    use bxcan::{filter::Mask32, Data, Frame, Id, Rx0, Rx1, StandardId, Tx};
    use defmt::*;
    use roboken_rs::can::id::{id_build, id_parse, Endpoint, NodeId};
    use stm32f4xx_hal::{
        can::Can,
        gpio::{Output, Pin},
        pac::{CAN1, TIM7},
        prelude::*,
        timer::{Counter, Event},
    };
    use systick_monotonic::{ExtU64, Systick};

    const TICK_INTERVAL_MS: u32 = 1000;
    const LED_BLINK_TIME_MS: u64 = 25;
    const NODE_ID: u8 = 0b_0001;

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1_000>;

    #[shared]
    struct Shared {
        tx_led: Pin<'A', 0, Output>,
        rx_led: Pin<'A', 1, Output>,
    }

    #[local]
    struct Local {
        ticker: Counter<TIM7, 1_000>,
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
            .pclk1(8.MHz())
            .freeze();

        let gpioa = ctx.device.GPIOA.split();
        let gpiob = ctx.device.GPIOB.split();

        let mut bxcan = bxcan::Can::builder(ctx.device.CAN1.can((gpiob.pb9, gpiob.pb8)))
            .set_bit_timing(0x00050000) // 1Mbps
            .set_automatic_retransmit(true)
            .set_loopback(false)
            .set_silent(false)
            .leave_disabled();
        let mut filters = bxcan.modify_filters();
        let mut mask = Mask32::frames_with_std_id(
            StandardId::new((NODE_ID << 3) as u16).unwrap(),
            StandardId::new(0b_0000_1111_000).unwrap(),
        );
        filters.enable_bank(0, bxcan::Fifo::Fifo0, *mask.data_frames_only());
        filters.enable_bank(0, bxcan::Fifo::Fifo1, *mask.remote_frames_only());
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

        let mut ticker = ctx.device.TIM7.counter_ms(&clocks);
        ticker.start(TICK_INTERVAL_MS.millis()).unwrap();
        ticker.listen(Event::Update);
        info!("Ticker initialized.");

        let systick = ctx.core.SYST.monotonic(&clocks);
        info!("SysTick initialized.");

        (
            Shared { tx_led, rx_led },
            Local {
                ticker,
                tx,
                rx0,
                rx1,
            },
            init::Monotonics(systick),
        )
    }

    #[task(binds=TIM7, local=[ticker])]
    fn tick(ctx: tick::Context) {
        // do something.

        // Unset flag.
        ctx.local.ticker.wait().unwrap();
    }

    #[task]
    fn pong(_: pong::Context, origin: NodeId) {
        let id = StandardId::new(id_build(
            NodeId::from_raw(NODE_ID).unwrap(),
            origin,
            Endpoint::Zero,
        ))
        .unwrap();
        let frame = Frame::new_data(id, [112, 111, 110, 103]); // pong.
        transmit::spawn(frame).unwrap();
    }

    #[task]
    fn data_frame_handle(_: data_frame_handle::Context, id: StandardId, data: Data) {
        let (origin, _, endpoint) = id_parse(id.as_raw());
        match endpoint {
            Endpoint::Zero => {
                if data == Data::new(&[112, 105, 110, 103]).unwrap() {
                    pong::spawn(origin).unwrap();
                }
            }
            _ => {}
        }
    }

    #[task]
    fn remote_frame_handle(_: remote_frame_handle::Context, id: StandardId, _dlc: u8) {
        let (_origin, _, endpoint) = id_parse(id.as_raw());
        match endpoint {
            _ => {}
        }
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
    fn receive_data(mut ctx: receive_data::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());
            let recv = nb::block!(ctx.local.rx0.receive()).unwrap();
            if let Id::Standard(id) = recv.id() {
                data_frame_handle::spawn(id, *recv.data().unwrap()).unwrap();
            }
        });
    }

    #[task(binds=CAN1_RX1, local=[rx1], shared=[rx_led])]
    fn receive_remote(mut ctx: receive_remote::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());
            let recv = nb::block!(ctx.local.rx1.receive()).unwrap();
            if let Id::Standard(id) = recv.id() {
                remote_frame_handle::spawn(id, recv.dlc()).unwrap();
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
