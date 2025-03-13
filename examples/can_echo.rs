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
        pac::CAN1,
        prelude::*,
        rcc::RccExt,
    };
    use systick_monotonic::{ExtU64, Systick};

    static LED_BLINK_TIME_MS: u64 = 25;
    static CAN_ID: u8 = 0b_0010;

    #[monotonic(binds = SysTick, default = true)]
    type Tonic = Systick<1_000>;

    #[shared]
    struct Shared {
        tx_led: Pin<'A', 0, Output>,
        rx_led: Pin<'A', 1, Output>,
    }

    #[local]
    struct Local {
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

        let systick = ctx.core.SYST.monotonic(&clocks);
        info!("SysTick initialized.");

        (
            Shared { tx_led, rx_led },
            Local { tx, rx0, rx1 },
            init::Monotonics(systick),
        )
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
    fn receive0(mut ctx: receive0::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());

            let recv = nb::block!(ctx.local.rx0.receive());
            if let Ok(frame) = recv {
                if let Id::Standard(id) = frame.id() {
                    let data = frame.data().unwrap();
                    debug!("id: {}, data: {}", id.as_raw(), data);
                    let (from, to, endpoint) = parse_id(&id);
                    if to == CAN_ID {
                        let id = build_id(CAN_ID, from, endpoint).unwrap();
                        let frame = Frame::new_data(id, *data);
                        let _ = transmit::spawn(frame);
                    }
                }
            }
        });
    }

    #[task(binds=CAN1_RX1, local=[rx1], shared=[rx_led])]
    fn receive1(mut ctx: receive1::Context) {
        ctx.shared.rx_led.lock(|rx_led| {
            rx_led.set_high();
            let _ = rx_led_off::spawn_after(LED_BLINK_TIME_MS.millis());

            let recv = nb::block!(ctx.local.rx1.receive());
            if let Ok(frame) = recv {
                if let Id::Standard(id) = frame.id() {
                    let data = frame.data().unwrap();
                    debug!("id: {}, data: {}", id.as_raw(), data);
                    let (from, to, endpoint) = parse_id(&id);
                    if to == CAN_ID {
                        let id = build_id(CAN_ID, from, endpoint).unwrap();
                        let frame = Frame::new_data(id, *data);
                        let _ = transmit::spawn(frame);
                    }
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

    fn build_id(from: u8, to: u8, endpoint: u8) -> Option<StandardId> {
        let raw_id = (from as u16) << 8 | (to as u16) << 3 | endpoint as u16;
        StandardId::new(raw_id)
    }

    fn parse_id(id: &StandardId) -> (u8, u8, u8) {
        let raw_id = id.as_raw();
        let from = ((raw_id & 0b_1111_0000_000) >> 8) as u8;
        let to = ((raw_id & 0b_0000_1111_000) >> 3) as u8;
        let endpoint = (raw_id & 0b_0000_0000_111) as u8;
        (from, to, endpoint)
    }
}
