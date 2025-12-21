//! USB device implementation
//!
//! Initial inspiration: cbiffle's Rust [implementation](https://github.com/cbiffle/rp2040-usb-device-in-one-file/blob/main/src/main.rs)
//! Currently progressing towards adopting the TinyUSB like API

const std = @import("std");
const microzig = @import("microzig");
const assert = std.debug.assert;
const peripherals = microzig.chip.peripherals;
const usb = microzig.core.usb;
const types = usb.types;

const BufferControlMmio = microzig.mmio.Mmio(@TypeOf(microzig.chip.peripherals.USB_DPRAM.EP0_IN_BUFFER_CONTROL).underlying_type);
const EndpointControlMimo = microzig.mmio.Mmio(@TypeOf(peripherals.USB_DPRAM.EP1_IN_CONTROL).underlying_type);
const EndpointType = microzig.chip.types.peripherals.USB_DPRAM.EndpointType;

const HardwareEndpoint = struct {
    awaiting_rx: bool,
    max_packet_size: u11,
    data_buffer: []u8,

    const init: @This() = .{
        .awaiting_rx = false,
        .max_packet_size = 0,
        .data_buffer = "",
    };
};

const rp2xxx_buffers = struct {
    // Address 0x100-0xfff (3840 bytes) can be used for data buffers
    const USB_DPRAM_DATA_BUFFER_BASE = 0x50100100;

    const CTRL_EP_BUFFER_SIZE = 64;

    const USB_EP0_BUFFER0 = USB_DPRAM_DATA_BUFFER_BASE;
    const USB_EP0_BUFFER1 = USB_DPRAM_DATA_BUFFER_BASE + CTRL_EP_BUFFER_SIZE;

    const USB_DATA_BUFFER = USB_DPRAM_DATA_BUFFER_BASE + (2 * CTRL_EP_BUFFER_SIZE);
    const USB_DATA_BUFFER_SIZE = 3840 - (2 * CTRL_EP_BUFFER_SIZE);

    const ep0_buffer0: *[CTRL_EP_BUFFER_SIZE]u8 = @as(*[CTRL_EP_BUFFER_SIZE]u8, @ptrFromInt(USB_EP0_BUFFER0));
    const ep0_buffer1: *[CTRL_EP_BUFFER_SIZE]u8 = @as(*[CTRL_EP_BUFFER_SIZE]u8, @ptrFromInt(USB_EP0_BUFFER1));
    const data_buffer: *[USB_DATA_BUFFER_SIZE]u8 = @as(*[USB_DATA_BUFFER_SIZE]u8, @ptrFromInt(USB_DATA_BUFFER));

    fn data_offset(ep_data_buffer: []u8) u16 {
        const buf_base = @intFromPtr(&ep_data_buffer[0]);
        const dpram_base = @intFromPtr(peripherals.USB_DPRAM);
        return @as(u16, @intCast(buf_base - dpram_base));
    }
};

const rp2xxx_endpoints = struct {
    const USB_DPRAM_BASE = 0x50100000;
    const USB_DPRAM_BUFFERS_BASE = USB_DPRAM_BASE + 0x100;
    const USB_DPRAM_BUFFERS_CTRL_BASE = USB_DPRAM_BASE + 0x80;
    const USB_DPRAM_ENDPOINTS_CTRL_BASE = USB_DPRAM_BASE + 0x8;
};

pub fn get_ep_ctrl(ep: types.Endpoint) ?*EndpointControlMimo {
    const idx = std.math.sub(u4, @intFromEnum(ep.num), 1) catch return null;
    const ep_ctrl_base = @as([*][2]u32, @ptrFromInt(rp2xxx_endpoints.USB_DPRAM_ENDPOINTS_CTRL_BASE));
    return @ptrCast(&ep_ctrl_base[idx][@intFromBool(ep.dir == .Out)]);
}

pub fn get_buf_ctrl(ep: types.Endpoint) *BufferControlMmio {
    const buf_ctrl_base = @as([*][2]u32, @ptrFromInt(rp2xxx_endpoints.USB_DPRAM_BUFFERS_CTRL_BASE));
    return @ptrCast(&buf_ctrl_base[@intFromEnum(ep.num)][@intFromBool(ep.dir == .Out)]);
}

pub const Config = struct {
    sync_nops: comptime_int = 3,
};

pub fn Polled(
    controller_config: usb.Config,
    config: Config,
) type {
    return struct {
        const vtable: usb.DeviceInterface.VTable = .{
            .start_tx = start_tx,
            .start_rx = start_rx,
            .set_address = set_address,
            .endpoint_open = endpoint_open,
        };

        endpoints: [1 << @bitSizeOf(types.Endpoint.Num)][2]HardwareEndpoint,
        data_buffer: []u8,
        controller: usb.DeviceController(controller_config),
        interface: usb.DeviceInterface,

        pub fn poll(self: *@This()) void {
            // Check which interrupt flags are set.

            const ints = peripherals.USB.INTS.read();

            // Setup request received?
            if (ints.SETUP_REQ != 0) {
                // Reset PID to 1 for EP0 IN. Every DATA packet we send in response
                // to an IN on EP0 needs to use PID DATA1, and this line will ensure
                // that.

                const bufctrl_ptr = get_buf_ctrl(.in(.ep0));
                var bufctrl = bufctrl_ptr.read();
                bufctrl.PID_0 = 0;
                bufctrl_ptr.write(bufctrl);

                const setup = get_setup_packet();
                self.controller.on_setup_req(&self.interface, &setup);
            }

            // Events on one or more buffers? (In practice, always one.)
            if (ints.BUFF_STATUS != 0) {
                const bufbits_init = peripherals.USB.BUFF_STATUS.raw;
                var bufbits = bufbits_init;

                while (true) {
                    // Who's still outstanding? Find their bit index by counting how
                    // many LSBs are zero.
                    const lowbit_index = std.math.cast(u5, @ctz(bufbits)) orelse break;
                    // Remove their bit from our set.
                    bufbits ^= @as(u32, @intCast(1)) << lowbit_index;

                    // Here we exploit knowledge of the ordering of buffer control
                    // registers in the peripheral. Each endpoint has a pair of
                    // registers, so we can determine the endpoint number by:
                    const epnum = @as(u4, @intCast(lowbit_index >> 1));
                    // Of the pair, the IN endpoint comes first, followed by OUT, so
                    // we can get the direction by:
                    const dir = if (lowbit_index & 1 == 0) usb.types.Dir.In else usb.types.Dir.Out;

                    const ep: types.Endpoint = .{ .num = @enumFromInt(epnum), .dir = dir };
                    // Process the buffer-done event.

                    // Process the buffer-done event.
                    //
                    // Scan the device table to figure out which endpoint struct
                    // corresponds to this address. We could use a smarter
                    // method here, but in practice, the number of endpoints is
                    // small so a linear scan doesn't kill us.

                    const ep_hard = self.hardware_endpoint_get_by_address(ep);

                    // We should only get here if we've been notified that
                    // the buffer is ours again. This is indicated by the hw
                    // _clearing_ the AVAILABLE bit.
                    //
                    // This ensures that we can return a shared reference to
                    // the databuffer contents without races.
                    // TODO: if ((bc & (1 << 10)) == 1) return EPBError.NotAvailable;

                    // Cool. Checks out.

                    // Get the actual length of the data, which may be less
                    // than the buffer size.
                    const bufctrl_ptr = get_buf_ctrl(ep);
                    const len = bufctrl_ptr.read().LENGTH_0;

                    self.controller.on_buffer(&self.interface, ep, ep_hard.data_buffer[0..len]);

                    if (ep.dir == .Out)
                        ep_hard.awaiting_rx = false;
                }

                peripherals.USB.BUFF_STATUS.write_raw(bufbits_init);
            } // <-- END of buf status handling

            // Has the host signaled a bus reset?
            if (ints.BUS_RESET != 0) {
                // Acknowledge by writing the write-one-to-clear status bit.
                peripherals.USB.SIE_STATUS.modify(.{ .BUS_RESET = 1 });
                peripherals.USB.ADDR_ENDP.modify(.{ .ADDRESS = 0 });

                self.controller.on_bus_reset();
            }
        }

        pub fn init() @This() {
            const chip = microzig.hal.compatibility.chip;

            if (chip == .RP2350)
                peripherals.USB.MAIN_CTRL.modify(.{ .PHY_ISO = 0 });

            // Clear the control portion of DPRAM. This may not be necessary -- the
            // datasheet is ambiguous -- but the C examples do it, and so do we.
            const dpram_regs: [*]volatile u32 = @ptrCast(peripherals.USB_DPRAM);
            const dpram_regs_num = @divExact(@sizeOf(@TypeOf(peripherals.USB_DPRAM.*)), @sizeOf(u32));
            @memset(dpram_regs[0..dpram_regs_num], 0);

            // Mux the controller to the onboard USB PHY. I was surprised that there are
            // alternatives to this, but, there are.
            peripherals.USB.USB_MUXING.modify(.{
                .TO_PHY = 1,
                // This bit is also set in the SDK example, without any discussion. It's
                // undocumented (being named does not count as being documented).
                .SOFTCON = 1,
            });

            // Force VBUS detect. Not all RP2040 boards wire up VBUS detect, which would
            // let us detect being plugged into a host (the Pi Pico, to its credit,
            // does). For maximum compatibility, we'll set the hardware to always
            // pretend VBUS has been detected.
            peripherals.USB.USB_PWR.modify(.{
                .VBUS_DETECT = 1,
                .VBUS_DETECT_OVERRIDE_EN = 1,
            });

            // Enable controller in device mode.
            peripherals.USB.MAIN_CTRL.modify(.{
                .CONTROLLER_EN = 1,
                .HOST_NDEVICE = 0,
            });

            // Request to have an interrupt (which really just means setting a bit in
            // the `buff_status` register) every time a buffer moves through EP0.
            peripherals.USB.SIE_CTRL.modify(.{
                .EP0_INT_1BUF = 1,
            });

            // Enable interrupts (bits set in the `ints` register) for other conditions
            // we use:
            peripherals.USB.INTE.modify(.{
                // A buffer is done
                .BUFF_STATUS = 1,
                // The host has reset us
                .BUS_RESET = 1,
                // We've gotten a setup request on EP0
                .SETUP_REQ = 1,
            });

            var self: @This() = .{
                .endpoints = @splat(@splat(.init)),
                .data_buffer = rp2xxx_buffers.data_buffer,
                .interface = .{ .vtable = &vtable },
                .controller = .init,
            };

            endpoint_open(&self.interface, &.{ .endpoint = .in(.ep0), .max_packet_size = .from(64), .attributes = .{ .transfer_type = .Control, .usage = .data }, .interval = 0 });
            endpoint_open(&self.interface, &.{ .endpoint = .out(.ep0), .max_packet_size = .from(64), .attributes = .{ .transfer_type = .Control, .usage = .data }, .interval = 0 });

            // Present full-speed device by enabling pullup on DP. This is the point
            // where the host will notice our presence.
            peripherals.USB.SIE_CTRL.modify(.{ .PULLUP_EN = 1 });

            return self;
        }

        /// Configures a given endpoint to send data (device-to-host, IN) when the host
        /// next asks for it.
        ///
        /// The contents of `buffer` will be _copied_ into USB SRAM, so you can
        /// reuse `buffer` immediately after this returns. No need to wait for the
        /// packet to be sent.
        fn start_tx(
            itf: *usb.DeviceInterface,
            ep_num: types.Endpoint.Num,
            buffer: []const u8,
        ) ?usize {
            // It is technically possible to support longer buffers but this demo
            // doesn't bother.
            // TODO: assert!(buffer.len() <= 64);

            // Acquire buffer ownership
            const bufctrl_ptr = get_buf_ctrl(.in(ep_num));
            var bufctrl = bufctrl_ptr.read();
            if (bufctrl.AVAILABLE_0 == 1)
                return null;

            // Register may have been only written partially
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl = bufctrl_ptr.read();

            const self: *@This() = @fieldParentPtr("interface", itf);
            const ep = self.hardware_endpoint_get_by_address(.in(ep_num));
            // TODO: please fixme: https://github.com/ZigEmbeddedGroup/microzig/issues/452
            std.mem.copyForwards(u8, ep.data_buffer[0..buffer.len], buffer);

            // Configure buffer
            bufctrl.PID_0 ^= 1;
            bufctrl.FULL_0 = 1; // We have put data in
            bufctrl.LENGTH_0 = @as(u10, @intCast(buffer.len)); // There are this many bytes
            bufctrl_ptr.write(bufctrl);

            // Nop for some clock cycles for synchronization and transfer ownership
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl.AVAILABLE_0 = 1;
            bufctrl_ptr.write(bufctrl);

            return @min(buffer.len, 64);
        }

        fn start_rx(itf: *usb.DeviceInterface, ep_num: types.Endpoint.Num, len: usize) void {
            // It is technically possible to support longer buffers but this demo
            // doesn't bother.
            // TODO: assert!(len <= 64);

            // Acquire buffer ownership
            const bufctrl_ptr = get_buf_ctrl(.out(ep_num));
            var bufctrl = bufctrl_ptr.read();
            if (bufctrl.AVAILABLE_0 == 1)
                return;

            // Register may have been only written partially
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl = bufctrl_ptr.read();

            const self: *@This() = @fieldParentPtr("interface", itf);
            const ep = self.hardware_endpoint_get_by_address(.out(ep_num));
            if (ep.awaiting_rx)
                return;

            // Configure the OUT:
            bufctrl.PID_0 ^= 1; // Flip DATA0/1
            bufctrl.FULL_0 = 0; // Buffer is NOT full, we want the computer to fill it
            bufctrl.LENGTH_0 = @intCast(len); // Up tho this many bytes
            bufctrl_ptr.write(bufctrl);

            // Nop for some clock cycles for synchronization and release ownership
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl.AVAILABLE_0 = 1;
            bufctrl_ptr.write(bufctrl);

            ep.awaiting_rx = true;
        }

        /// Returns a received USB setup packet
        ///
        /// Side effect: The setup request status flag will be cleared
        ///
        /// One can assume that this function is only called if the
        /// setup request falg is set.
        fn get_setup_packet() usb.types.SetupPacket {
            // Clear the status flag (write-one-to-clear)
            peripherals.USB.SIE_STATUS.modify(.{ .SETUP_REC = 1 });

            // This assumes that the setup packet is arriving on EP0, our
            // control endpoint. Which it should be. We don't have any other
            // Control endpoints.

            // Copy the setup packet out of its dedicated buffer at the base of
            // USB SRAM. The PAC models this buffer as two 32-bit registers,
            // which is, like, not _wrong_ but slightly awkward since it means
            // we can't just treat it as bytes. Instead, copy it out to a byte
            // array.
            var setup_packet: [8]u8 = @splat(0);
            const spl: u32 = peripherals.USB_DPRAM.SETUP_PACKET_LOW.raw;
            const sph: u32 = peripherals.USB_DPRAM.SETUP_PACKET_HIGH.raw;
            @memcpy(setup_packet[0..4], std.mem.asBytes(&spl));
            @memcpy(setup_packet[4..8], std.mem.asBytes(&sph));
            // Reinterpret as setup packet
            return std.mem.bytesToValue(usb.types.SetupPacket, &setup_packet);
        }

        fn set_address(itf: *usb.DeviceInterface, addr: u7) void {
            const self: *@This() = @fieldParentPtr("interface", itf);
            _ = self;

            peripherals.USB.ADDR_ENDP.modify(.{ .ADDRESS = addr });
        }

        fn hardware_endpoint_get_by_address(self: *@This(), ep: types.Endpoint) *HardwareEndpoint {
            return &self.endpoints[@intFromEnum(ep.num)][@intFromEnum(ep.dir)];
        }

        fn endpoint_open(itf: *usb.DeviceInterface, desc: *const usb.descriptor.Endpoint) void {
            const self: *@This() = @fieldParentPtr("interface", itf);

            const ep = desc.endpoint;
            const ep_hard = self.hardware_endpoint_get_by_address(ep);

            ep_hard.max_packet_size = @intCast(desc.max_packet_size.into());
            ep_hard.awaiting_rx = false;

            if (get_ep_ctrl(ep)) |ep_ctrl_ptr| {
                // round up size to multiple of 64
                const size = 64 * (std.math.divCeil(
                    u16,
                    desc.max_packet_size.into(),
                    64,
                ) catch unreachable);
                std.debug.assert(self.data_buffer.len >= size);
                ep_hard.data_buffer = self.data_buffer[0..size];
                self.data_buffer = self.data_buffer[size..];

                var ep_ctrl = ep_ctrl_ptr.read();
                ep_ctrl.ENABLE = 1;
                ep_ctrl.INTERRUPT_PER_BUFF = 1;
                ep_ctrl.ENDPOINT_TYPE = @enumFromInt(@intFromEnum(desc.attributes.transfer_type));
                ep_ctrl.BUFFER_ADDRESS = rp2xxx_buffers.data_offset(ep_hard.data_buffer);
                ep_ctrl_ptr.write(ep_ctrl);
            } else {
                // ep0 has fixed data buffer
                ep_hard.data_buffer = rp2xxx_buffers.ep0_buffer0;
            }
        }
    };
}
