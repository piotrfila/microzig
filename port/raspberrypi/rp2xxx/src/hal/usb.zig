//! USB device implementation
//!
//! Initial inspiration: cbiffle's Rust [implementation](https://github.com/cbiffle/rp2040-usb-device-in-one-file/blob/main/src/main.rs)
//! Currently progressing towards adopting the TinyUSB like API

const std = @import("std");
const microzig = @import("microzig");
const assert = std.debug.assert;
const USB = microzig.chip.peripherals.USB;
const USB_DPRAM = microzig.chip.peripherals.USB_DPRAM;
const usb = microzig.core.usb;
const types = usb.types;

const HardwareBuffer = enum(u6) {
    const size = 64;
    const TagType = @typeInfo(@This()).@"enum".tag_type;

    none = 0,
    unused1 = 1,
    unused2 = 2,
    unused3 = 3,
    ep0_buf0 = 4,
    ep0_buf1 = 5,
    _,

    var last_alloc: TagType = @intFromEnum(@This().ep0_buf1);

    fn alloc() HardwareBuffer {
        const last = @atomicRmw(TagType, &last_alloc, .Add, 1, .monotonic);
        return @enumFromInt(last + 1);
    }

    fn to_offset(self: @This()) u16 {
        return @as(u16, @intFromEnum(self)) * size;
    }

    fn data(self: @This()) []u8 {
        const num_buffers = 1 << @bitSizeOf(@This());
        comptime assert(num_buffers * size == 4096);
        // It's ok to @volatileCast because ownership transfer does not happen during access.
        const buffers: *[num_buffers][size]u8 = @ptrCast(@volatileCast(USB_DPRAM));
        return &buffers[@intFromEnum(self)];
    }
};

// While it's tempting to store this data in the unused high 16 bits
// of buffer_control registers, errata RP2040-E4 makes this impossbile.
const EndpointData = packed struct(u16) {
    buffer: HardwareBuffer,
    max_len: u10,

    const init: @This() = .{ .buffer = .none, .max_len = 0 };
};

pub const Config = struct {
    sync_nops: comptime_int = 3,
    max_packet_size: comptime_int = 64,
    num_endpoints: comptime_int = 1 << @bitSizeOf(types.Endpoint.Num),
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

        ep_data: [config.num_endpoints][2]EndpointData,
        controller: usb.DeviceController(controller_config),
        interface: usb.DeviceInterface,

        fn get_ep_ctrl(ep: types.Endpoint) ?*volatile @TypeOf(USB_DPRAM.EP1_IN_CONTROL) {
            const idx = std.math.sub(u4, @intFromEnum(ep.num), 1) catch return null;
            const ep_ctrl: *volatile [config.num_endpoints - 1][2]u32 = @ptrCast(&USB_DPRAM.EP1_IN_CONTROL);
            return @ptrCast(&ep_ctrl[idx][@intFromBool(ep.dir == .Out)]);
        }

        fn get_bufctrl(ep: types.Endpoint) *volatile @TypeOf(USB_DPRAM.EP0_IN_BUFFER_CONTROL) {
            const buf_ctrl: *volatile [config.num_endpoints][2]u32 = @ptrCast(&USB_DPRAM.EP0_IN_BUFFER_CONTROL);
            return @ptrCast(&buf_ctrl[@intFromEnum(ep.num)][@intFromBool(ep.dir == .Out)]);
        }

        fn get_ep_data(self: *@This(), ep: types.Endpoint) *EndpointData {
            return &self.ep_data[@intFromEnum(ep.num)][@intFromEnum(ep.dir)];
        }

        pub fn poll(self: *@This()) void {
            // Check which interrupt flags are set.
            const ints = USB.INTS.read();

            // Setup request received?
            if (ints.SETUP_REQ != 0) {
                // Reset PID to 1 for EP0 IN. Every DATA packet we send in response
                // to an IN on EP0 needs to use PID DATA1.
                get_bufctrl(.in(.ep0)).modify(.{ .PID_0 = 0 });

                // Copy the setup packet out of its dedicated buffer.
                const setup: types.SetupPacket = @bitCast([2]u32{
                    USB_DPRAM.SETUP_PACKET_LOW.raw,
                    USB_DPRAM.SETUP_PACKET_HIGH.raw,
                });

                // Clear the status flag (write-one-to-clear)
                USB.SIE_STATUS.configure(.{ .SETUP_REC = 1 });

                self.controller.on_setup_req(&self.interface, setup);
            }

            // Events on one or more buffers? (In practice, always one.)
            if (ints.BUFF_STATUS != 0) {
                const bufbits_init = USB.BUFF_STATUS.raw;
                var bufbits = bufbits_init;

                while (bufbits != 0) {
                    // Who's still outstanding? Find their bit index by counting how
                    // many LSBs are zero.
                    const lowbit_index = std.math.cast(u5, @ctz(bufbits)) orelse unreachable;
                    // Remove their bit from our set.
                    bufbits ^= @as(u32, @intCast(1)) << lowbit_index;

                    // Here we exploit knowledge of the ordering of buffer control
                    // registers in the peripheral. Each endpoint has a pair of
                    // registers, so we can determine the endpoint number by:
                    const epnum: u4 = @intCast(lowbit_index >> 1);
                    // Of the pair, the IN endpoint comes first, followed by OUT, so
                    // we can get the direction by:
                    const dir = if (lowbit_index & 1 == 0) usb.types.Dir.In else usb.types.Dir.Out;

                    const ep: types.Endpoint = .{ .num = @enumFromInt(epnum), .dir = dir };

                    // Process the buffer-done event.
                    const data = self.get_ep_data(ep).buffer.data();
                    const len = get_bufctrl(ep).read().LENGTH_0;
                    self.controller.on_buffer(&self.interface, ep, data[0..len]);
                }

                USB.BUFF_STATUS.write_raw(bufbits_init);
            }

            // Has the host signaled a bus reset?
            if (ints.BUS_RESET != 0) {
                // Acknowledge by writing the write-one-to-clear status bit.
                USB.SIE_STATUS.configure(.{ .BUS_RESET = 1 });
                USB.ADDR_ENDP.configure(.{ .ADDRESS = 0 });

                self.controller.on_bus_reset();
            }
        }

        pub fn init() @This() {
            // Clear the control portion of DPRAM. This may not be necessary -- the
            // datasheet is ambiguous -- but the C examples do it, and so do we.
            const dpram_regs: [*]volatile u32 = @ptrCast(USB_DPRAM);
            const dpram_regs_num = @divExact(@sizeOf(@TypeOf(USB_DPRAM.*)), @sizeOf(u32));
            @memset(dpram_regs[0..dpram_regs_num], 0);

            // Mux the controller to the onboard USB PHY. I was surprised that there are
            // alternatives to this, but, there are.
            USB.USB_MUXING.configure(.{
                .TO_PHY = 1,
                // This bit is also set in the SDK example, without any discussion. It's
                // undocumented (being named does not count as being documented).
                .SOFTCON = 1,
            });

            // Force VBUS detect. Not all RP2040 boards wire up VBUS detect, which would
            // let us detect being plugged into a host (the Pi Pico, to its credit,
            // does). For maximum compatibility, we'll set the hardware to always
            // pretend VBUS has been detected.
            USB.USB_PWR.configure(.{
                .VBUS_DETECT = 1,
                .VBUS_DETECT_OVERRIDE_EN = 1,
            });

            // Enable controller in device mode.
            USB.MAIN_CTRL.configure(.{
                .CONTROLLER_EN = 1,
                .HOST_NDEVICE = 0,
            });

            // Request to have an interrupt (which really just means setting a bit in
            // the `buff_status` register) every time a buffer moves through EP0.
            USB.SIE_CTRL.configure(.{
                .EP0_INT_1BUF = 1,
            });

            // Enable interrupts (bits set in the `ints` register) for other conditions
            // we use:
            USB.INTE.configure(.{
                // A buffer is done
                .BUFF_STATUS = 1,
                // The host has reset us
                .BUS_RESET = 1,
                // We've gotten a setup request on EP0
                .SETUP_REQ = 1,
            });

            var self: @This() = .{
                .ep_data = @splat(@splat(.init)),
                .interface = .{ .vtable = &vtable },
                .controller = .init,
            };

            const ep0_data: EndpointData = .{
                .buffer = .ep0_buf0,
                .max_len = config.max_packet_size,
            };
            self.get_ep_data(.in(.ep0)).* = ep0_data;
            self.get_ep_data(.out(.ep0)).* = ep0_data;

            // Present full-speed device by enabling pullup on DP. This is the point
            // where the host will notice our presence.
            USB.SIE_CTRL.modify(.{ .PULLUP_EN = 1 });

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
            data: []const u8,
        ) ?usize {
            // It is technically possible to support longer buffers but this demo
            // doesn't bother.
            // TODO: assert!(buffer.len() <= 64);

            // Acquire buffer ownership
            const bufctrl_ptr = get_bufctrl(.in(ep_num));
            var bufctrl = bufctrl_ptr.read();
            if (bufctrl.AVAILABLE_0 == 1)
                return null;

            // Register may have been only written partially
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl = bufctrl_ptr.read();

            const self: *@This() = @fieldParentPtr("interface", itf);
            const ep_data = self.get_ep_data(.in(ep_num));
            const len = @min(ep_data.max_len, data.len);
            // TODO: please fixme: https://github.com/ZigEmbeddedGroup/microzig/issues/452
            std.mem.copyForwards(u8, ep_data.buffer.data()[0..len], data[0..len]);

            // Configure buffer
            bufctrl.PID_0 ^= 1;
            bufctrl.FULL_0 = 1; // We have put data in
            bufctrl.LENGTH_0 = @intCast(len); // There are this many bytes
            bufctrl_ptr.write(bufctrl);

            // Nop for some clock cycles for synchronization and transfer ownership
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl.AVAILABLE_0 = 1;
            bufctrl_ptr.write(bufctrl);

            return len;
        }

        fn start_rx(
            itf: *usb.DeviceInterface,
            ep_num: types.Endpoint.Num,
            data: []u8,
            request_len: ?u10,
        ) ?usize {
            // Acquire buffer ownership
            const bufctrl_ptr = get_bufctrl(.out(ep_num));
            var bufctrl = bufctrl_ptr.read();
            if (bufctrl.AVAILABLE_0 == 1) return null;

            // Register may have been only written partially
            asm volatile ("nop\n" ** config.sync_nops);
            bufctrl = bufctrl_ptr.read();

            // Configure the OUT:
            bufctrl.PID_0 ^= 1; // Flip DATA0/1
            const self: *@This() = @fieldParentPtr("interface", itf);
            const ep_data = self.get_ep_data(.out(ep_num));
            const ret = if (bufctrl.FULL_0 == 0)
                0
            else blk: {
                const len = @min(data.len, bufctrl.LENGTH_0);
                // TODO: Partial reads
                if (data.len < len)
                    std.log.err("discarded rx data", .{});
                std.mem.copyForwards(u8, data[0..len], ep_data.buffer.data()[0..len]);
                break :blk len;
            };
            bufctrl.FULL_0 = 0; // Buffer is NOT full, we want the computer to fill it
            if (request_len) |len| {
                bufctrl.LENGTH_0 = @min(len, ep_data.max_len); // Up tho this many bytes
                bufctrl_ptr.write(bufctrl);

                // Nop for some clock cycles for synchronization and release ownership
                asm volatile ("nop\n" ** config.sync_nops);
                bufctrl.AVAILABLE_0 = 1;
            }
            bufctrl_ptr.write(bufctrl);
            return ret;
        }

        fn set_address(itf: *usb.DeviceInterface, addr: u7) void {
            _ = itf;
            USB.ADDR_ENDP.configure(.{ .ADDRESS = addr });
        }

        fn endpoint_open(itf: *usb.DeviceInterface, desc: *const usb.descriptor.Endpoint) void {
            const self: *@This() = @fieldParentPtr("interface", itf);

            const ep = desc.endpoint;
            const ep_data = self.get_ep_data(ep);

            if (ep_data.buffer != .none)
                std.debug.panic("endpoint configured twice:\n{any}\n", .{desc.*});

            ep_data.* = .{
                .buffer = .alloc(),
                .max_len = desc.max_packet_size.into_len(),
            };
            assert(ep_data.buffer.data().len >= desc.max_packet_size.into_len());

            const EndpointType = microzig.chip.types.peripherals.USB_DPRAM.EndpointType;

            get_ep_ctrl(ep).?.configure(.{
                .ENABLE = 1,
                .INTERRUPT_PER_BUFF = 1,
                .ENDPOINT_TYPE = @as(EndpointType, @enumFromInt(@intFromEnum(desc.attributes.transfer_type))),
                .BUFFER_ADDRESS = ep_data.buffer.to_offset(),
            });
        }
    };
}
