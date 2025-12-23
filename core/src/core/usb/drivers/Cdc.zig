const std = @import("std");
const usb = @import("../../usb.zig");
const assert = std.debug.assert;
const descriptor = usb.descriptor;
const types = usb.types;

const utilities = @import("../../../utilities.zig");

pub const ManagementRequestType = enum(u8) {
    SetLineCoding = 0x20,
    GetLineCoding = 0x21,
    SetControlLineState = 0x22,
    SendBreak = 0x23,
};

pub const LineCoding = extern struct {
    bit_rate: u32 align(1),
    stop_bits: u8,
    parity: u8,
    data_bits: u8,

    pub const init: @This() = .{
        .bit_rate = 115200,
        .stop_bits = 0,
        .parity = 0,
        .data_bits = 8,
    };
};

const options_max_packet_size = 64;

const FIFO = utilities.Queue;

pub const Descriptor = extern struct {
    itf_assoc: descriptor.InterfaceAssociation,
    itf_notifi: descriptor.Interface,
    cdc_header: descriptor.cdc.Header,
    cdc_call_mgmt: descriptor.cdc.CallManagement,
    cdc_acm: descriptor.cdc.AbstractControlModel,
    cdc_union: descriptor.cdc.Union,
    ep_notifi: descriptor.Endpoint,
    itf_data: descriptor.Interface,
    ep_out: descriptor.Endpoint,
    ep_in: descriptor.Endpoint,

    pub fn create(
        first_interface: u8,
        first_string: u8,
        first_endpoint_in: u4,
        first_endpoint_out: u4,
    ) @This() {
        const endpoint_notifi_size = 8;
        return .{
            .itf_assoc = .{
                .first_interface = first_interface,
                .interface_count = 2,
                .function_class = 2,
                .function_subclass = 2,
                .function_protocol = 0,
                .function = 0,
            },
            .itf_notifi = .{
                .interface_number = first_interface,
                .alternate_setting = 0,
                .num_endpoints = 1,
                .interface_class = 2,
                .interface_subclass = 2,
                .interface_protocol = 0,
                .interface_s = first_string,
            },
            .cdc_header = .{ .bcd_cdc = .from(0x0120) },
            .cdc_call_mgmt = .{
                .capabilities = 0,
                .data_interface = first_interface + 1,
            },
            .cdc_acm = .{ .capabilities = 6 },
            .cdc_union = .{
                .master_interface = first_interface,
                .slave_interface_0 = first_interface + 1,
            },
            .ep_notifi = .{
                .endpoint = .in(@enumFromInt(first_endpoint_in)),
                .attributes = .{ .transfer_type = .Interrupt, .usage = .data },
                .max_packet_size = .from(endpoint_notifi_size),
                .interval = 16,
            },
            .itf_data = .{
                .interface_number = first_interface + 1,
                .alternate_setting = 0,
                .num_endpoints = 2,
                .interface_class = 10,
                .interface_subclass = 0,
                .interface_protocol = 0,
                .interface_s = 0,
            },
            .ep_out = .{
                .endpoint = .out(@enumFromInt(first_endpoint_out)),
                .attributes = .{ .transfer_type = .Bulk, .usage = .data },
                .max_packet_size = .from(options_max_packet_size),
                .interval = 0,
            },
            .ep_in = .{
                .endpoint = .in(@enumFromInt(first_endpoint_in + 1)),
                .attributes = .{ .transfer_type = .Bulk, .usage = .data },
                .max_packet_size = .from(options_max_packet_size),
                .interval = 0,
            },
        };
    }
};

device: *usb.DeviceInterface,
desc: *const Descriptor,
line_coding: LineCoding align(4),

tx: FIFO,

epin_buf: [options_max_packet_size]u8 = undefined,

// TODO: this is terrible
var tx_buffer: [64]u8 align(64) = undefined;

pub fn read(self: *@This(), dst: []u8) usize {
    return self.device.ep_read(self.desc.ep_out.endpoint.num, dst, 64) orelse 0;
}

pub fn write(self: *@This(), data: []const u8) []const u8 {
    const write_count = @min(self.tx.get_writable_len(), data.len);

    if (write_count > 0) {
        self.tx.write(data[0..write_count]) catch unreachable;
    } else {
        return data[0..];
    }

    if (self.tx.get_writable_len() == 0) {
        _ = self.write_flush();
    }

    return data[write_count..];
}

pub fn write_flush(self: *@This()) usize {
    if (self.tx.get_readable_len() == 0) {
        return 0;
    }
    std.log.info("{any}", .{self.tx});
    const len = self.tx.read(&self.epin_buf);
    // TODO: wait instead of discard
    if (self.device.ep_write(self.desc.ep_in.endpoint.num, self.epin_buf[0..len]) != len)
        std.log.err("data discarded", .{});
    std.log.info("{any} {}", .{ self.tx, len });
    return len;
}

pub fn init(desc: *const Descriptor, device: *usb.DeviceInterface) @This() {
    assert(device.ep_read(
        desc.ep_out.endpoint.num,
        "",
        desc.ep_out.max_packet_size.into_len(),
    ) == 0);
    return .{
        .device = device,
        .desc = desc,
        .tx = .init(tx_buffer.len, &tx_buffer),
        .line_coding = .{
            .bit_rate = 115200,
            .stop_bits = 0,
            .parity = 0,
            .data_bits = 8,
        },
    };
}

pub fn class_control(self: *@This(), stage: types.ControlStage, setup: types.SetupPacket) ?[]const u8 {
    if (std.meta.intToEnum(ManagementRequestType, setup.request)) |request| {
        if (stage == .Setup) switch (request) {
            .SetLineCoding => return usb.ack, // HACK, we should handle data phase somehow to read sent line_coding
            .GetLineCoding => return std.mem.asBytes(&self.line_coding),
            .SetControlLineState => {
                // const DTR_BIT = 1;
                // self.is_ready = (setup.value & DTR_BIT) != 0;
                // self.line_state = @intCast(setup.value & 0xFF);
                return usb.ack;
            },
            .SendBreak => return usb.ack,
        };
    } else |_| {}

    return usb.nak;
}

pub fn transfer(self: *@This(), ep: types.Endpoint, data: []u8) void {
    if (ep == self.desc.ep_in.endpoint) {
        if (self.write_flush() == 0) {
            // If there is no data left, a empty packet should be sent if
            // data len is multiple of EP Packet size and not zero
            if (self.tx.get_readable_len() == 0 and data.len > 0 and data.len == options_max_packet_size) {
                assert(self.device.ep_write(self.desc.ep_in.endpoint.num, usb.ack) == 0);
            }
        }
    }
}
