#![no_std]
#![no_main]
#![feature(naked_functions)]
#![allow(static_mut_refs)]

#[macro_use]
extern crate rcore_console;

use core::mem::MaybeUninit;
use core::sync::{atomic::AtomicBool, atomic::AtomicU64, atomic::Ordering};
use core::{arch::asm, ptr::null};
use sbi::SbiRet;
use sbi_spec::binary::HartMask;
use sbi_spec::hsm::hart_state;
use sbi_testing::sbi;
use serde::Deserialize;
use serde_device_tree::{
    buildin::{Node, NodeSeq, Reg, StrSeq},
    error::Error,
    from_raw_mut, Dtb, DtbPtr,
};
use uart16550::Uart16550;

const RISCV_HEAD_FLAGS: u64 = 0;
const RISCV_HEADER_VERSION: u32 = 0x2;
const RISCV_IMAGE_MAGIC: u64 = 0x5643534952; /* Magic number, little endian, "RISCV" */
const RISCV_INAGE_MAGIC2: u32 = 0x05435352; /* Magic number 2, little endian, "RSC\x05" */

/// boot header
#[naked]
#[no_mangle]
#[link_section = ".head.text"]
unsafe extern "C" fn _boot_header() -> ! {
    asm!(
        "j _start",
        ".word 0",
        ".balign 8",
        ".dword 0x200000",
        ".dword iend - istart",
        ".dword {RISCV_HEAD_FLAGS}",
        ".word  {RISCV_HEADER_VERSION}",
        ".word  0",
        ".dword 0",
        ".dword {RISCV_IMAGE_MAGIC}",
        ".balign 4",
        ".word  {RISCV_IMAGE_MAGIC2}",
        ".word  0",
        RISCV_HEAD_FLAGS = const RISCV_HEAD_FLAGS,
        RISCV_HEADER_VERSION = const RISCV_HEADER_VERSION,
        RISCV_IMAGE_MAGIC = const RISCV_IMAGE_MAGIC,
        RISCV_IMAGE_MAGIC2 = const RISCV_INAGE_MAGIC2,
        options(noreturn)
    );
}

const STACK_SIZE: usize = 16384 * 16 * 16; // 16 KiB
const MAX_HART_NUM: usize = 128;

#[derive(Copy, Clone)]
struct HartStack([u8; STACK_SIZE]);

impl HartStack {
    pub const fn new() -> Self {
        HartStack([0; STACK_SIZE])
    }
}

#[link_section = ".bss.uninit"]
static mut STACK: HartStack = HartStack::new();
static mut HART_STACK: [HartStack; MAX_HART_NUM] = [HartStack::new(); MAX_HART_NUM];
static mut IPI_SENT: [MaybeUninit<AtomicBool>; MAX_HART_NUM] =
    [const { MaybeUninit::uninit() }; MAX_HART_NUM];
static mut GLOBAL_MASK: HartMask = HartMask::from_mask_base(0, 0);

/// 内核入口。
///
/// # Safety
///
/// 裸函数。
#[naked]
#[no_mangle]
#[link_section = ".text.entry"]
unsafe extern "C" fn _start(hartid: usize, device_tree_paddr: usize) -> ! {
    asm!(
        // clear bss segment
        "   la      t0, sbss
            la      t1, ebss
            la      t3, {ssoft_trap}
            csrw    stvec, t3
        1:  bgeu    t0, t1, 2f
            sd      zero, 0(t0)
            addi    t0, t0, 8
            j       1b",
        "2:",
        "   la sp, {stack} + {stack_size}",
        "   j  {main}",
        stack_size = const STACK_SIZE,
        ssoft_trap = sym ssoft_trap,
        stack      =   sym STACK,
        main       =   sym rust_main,
        options(noreturn),
    )
}

#[naked]
#[no_mangle]
unsafe extern "C" fn ssoft_trap() {
    asm!("sret", options(noreturn),)
}

#[naked]
#[no_mangle]
unsafe extern "C" fn init_hart(hartid: usize, opaque: usize) {
    asm!(
        "add sp, a1, zero",
        "csrw sscratch, sp",
        "call {init_main}",
        init_main = sym init_main,
        options(noreturn),
    )
}

#[naked]
#[no_mangle]
unsafe extern "C" fn core_send_ipi(hartid: usize, opaque: usize) {
    asm!(
        "add sp, a1, zero",
        "csrw sscratch, sp",
        "call {send_ipi}",
        send_ipi = sym send_ipi,
        options(noreturn),
    )
}

extern "C" fn send_ipi(hartid: usize) -> ! {
    if unsafe { !(IPI_SENT[hartid].assume_init_mut().load(Ordering::Relaxed)) } {
        unsafe {
            IPI_SENT[hartid]
                .assume_init_mut()
                .swap(true, Ordering::AcqRel);
            sbi::remote_sfence_vma(GLOBAL_MASK, 0, 0);
            WAIT_COUNT.fetch_sub(1, Ordering::AcqRel);
            while WAIT_COUNT.load(Ordering::Relaxed) != 0 {}
        }
    } else {
        unreachable!("send {}", hartid);
    }
    sbi::hart_suspend(sbi::NonRetentive, core_send_ipi as _, unsafe {
        core::ptr::addr_of!(HART_STACK[hartid + 1]) as _
    });
    unreachable!()
}

extern "C" fn init_main(hartid: usize) -> ! {
    sbi::hart_suspend(sbi::NonRetentive, core_send_ipi as _, unsafe {
        core::ptr::addr_of!(HART_STACK[hartid + 1]) as _
    });
    loop {
        core::hint::spin_loop();
    }
}

static mut WAIT_COUNT: AtomicU64 = AtomicU64::new(0);

// const STARTED: SbiRet = SbiRet::success(hart_state::STARTED);
// const STOPPED: SbiRet = SbiRet::success(hart_state::STOPPED);
const SUSPENDED: SbiRet = SbiRet::success(hart_state::SUSPENDED);

fn get_time() -> u64 {
    const CSR_TIME: u32 = 0xc01;
    let mut low_time: u64;
    unsafe {
        asm!("csrr {}, {CSR_TIME}", out(reg) low_time, CSR_TIME = const CSR_TIME);
        //        asm!("csrr {}, {CSR_TIMEH}", out(reg) upper_time, CSR_TIMEH = const CSR_TIMEH);
    }

    low_time
}

extern "C" fn rust_main(hartid: usize, dtb_pa: usize) -> ! {
    #[derive(Deserialize)]
    struct Tree<'a> {
        cpus: Cpus<'a>,
        chosen: Option<Chosen<'a>>,
    }
    #[derive(Deserialize)]
    #[serde(rename_all = "kebab-case")]
    struct Cpus<'a> {
        timebase_frequency: u32,
        cpu: NodeSeq<'a>,
    }
    #[derive(Deserialize)]
    #[serde(rename_all = "kebab-case")]
    struct Chosen<'a> {
        stdout_path: Option<StrSeq<'a>>,
    }
    unsafe { UART = Uart16550Map(268435456 as _) };
    rcore_console::init_console(&Console);
    rcore_console::set_log_level(option_env!("LOG"));
    println!("main");
    let dtb_ptr = DtbPtr::from_raw(dtb_pa as _).unwrap();
    let dtb = Dtb::from(dtb_ptr).share();
    let root: Node = serde_device_tree::from_raw_mut(&dtb).unwrap();
    let tree: Tree = root.deserialize();
    let smp = tree.cpus.cpu.len();
    let frequency = tree.cpus.timebase_frequency;
    println!("parsed");
    println!(
        r"
 _____         _     _  __                    _
|_   _|__  ___| |_  | |/ /___ _ __ _ __   ___| |
  | |/ _ \/ __| __| | ' // _ \ '__| '_ \ / _ \ |
  | |  __/\__ \ |_  | . \  __/ |  | | | |  __/ |
  |_|\___||___/\__| |_|\_\___|_|  |_| |_|\___|_|
================================================
| boot hart id          | {hartid:20} |
| smp                   | {smp:20} |
| timebase frequency    | {frequency:17} Hz |
| dtb physical address  | {dtb_pa:#20x} |
------------------------------------------------"
    );
    let mut mask = HartMask::from_mask_base(0, 0);
    for i in 0..smp {
        unsafe {
            IPI_SENT[i].write(AtomicBool::new(false));
        }
        if i != hartid {
            sbi::hart_start(i, init_hart as _, unsafe {
                core::ptr::addr_of!(HART_STACK[i + 1]) as _
            });
            while sbi::hart_get_status(i) != SUSPENDED {
                core::hint::spin_loop();
            }
            let _ = mask.insert(i);
        }
    }
    unsafe {
        GLOBAL_MASK = mask;
    }
    for _ in 0..4 {
        unsafe {
            for i in 0..smp {
                IPI_SENT[i].assume_init_mut().swap(false, Ordering::AcqRel);
                if i != hartid {
                    while sbi::hart_get_status(i) != SUSPENDED {
                        core::hint::spin_loop();
                    }
                }
            }
            WAIT_COUNT.swap((smp - 1) as u64, Ordering::AcqRel);
        }
        println!("send!");
        let start_time = get_time();
        sbi::send_ipi(mask);
        while unsafe { WAIT_COUNT.load(Ordering::Relaxed) } != 0 {}
        let end_time = get_time();
        println!("{} {} {}", start_time, end_time, end_time - start_time);
    }
    sbi::system_reset(sbi::Shutdown, sbi::NoReason);
    unreachable!()
}

#[cfg_attr(not(test), panic_handler)]
fn panic(info: &core::panic::PanicInfo) -> ! {
    let (hart_id, pc): (usize, usize);
    unsafe { asm!("mv    {}, tp", out(reg) hart_id) };
    unsafe { asm!("auipc {},  0", out(reg) pc) };
    println!("[test-kernel-panic] hart {hart_id} {info}");
    println!("[test-kernel-panic] pc = {pc:#x}");
    println!("[test-kernel-panic] SBI test FAILED due to panic");
    sbi::system_reset(sbi::Shutdown, sbi::SystemFailure);
    loop {}
}

struct Console;
static mut UART: Uart16550Map = Uart16550Map(null());

pub struct Uart16550Map(*const Uart16550<u8>);

unsafe impl Sync for Uart16550Map {}

impl Uart16550Map {
    #[inline]
    pub fn get(&self) -> &Uart16550<u8> {
        unsafe { &*self.0 }
    }
}

impl rcore_console::Console for Console {
    #[inline]
    fn put_char(&self, c: u8) {
        unsafe { UART.get().write(core::slice::from_ref(&c)) };
    }

    #[inline]
    fn put_str(&self, s: &str) {
        unsafe { UART.get().write(s.as_bytes()) };
    }
}
