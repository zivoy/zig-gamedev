const std = @import("std");
const common = @import("common.zig");

pub const identity: common.Quaternion = .{ .w = 1, .x = 0, .y = 0, .z = 0 };

pub const right: common.Vector3 = .{ .v = [3]f32{ 1, 0, 0 } };
pub const left: common.Vector3 = .{ .v = [3]f32{ -1, 0, 0 } };
pub const up: common.Vector3 = .{ .v = [3]f32{ 0, 1, 0 } };
pub const down: common.Vector3 = .{ .v = [3]f32{ 0, -1, 0 } };
pub const forward: common.Vector3 = .{ .v = [3]f32{ 0, 0, -1 } };
pub const backward: common.Vector3 = .{ .v = [3]f32{ 0, 0, 1 } };

pub fn quaternionFromMatrix(matrix: anytype) common.Quaternion {
    comptime {
        const T = @TypeOf(matrix);
        if (!(T == common.Matrix34 or T == common.Matrix33)) @compileError("only matrix34 and matrix33 supported");
    }

    var quaternion = common.Quaternion{
        .w = @sqrt(@max(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2,
        .x = @sqrt(@max(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2,
        .y = @sqrt(@max(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2,
        .z = @sqrt(@max(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2,
    };

    quaternion.x = std.math.copysign(quaternion.x, matrix.m[2][1] - matrix.m[1][2]);
    quaternion.y = std.math.copysign(quaternion.y, matrix.m[0][2] - matrix.m[2][0]);
    quaternion.z = std.math.copysign(quaternion.z, matrix.m[1][0] - matrix.m[0][1]);

    return quaternion;
}

pub fn quaternionFromSwingTwist(swing: common.Vector2, twist: f32) common.Quaternion {
    var quaternion: common.Quaternion = undefined;
    const swing_squared: f32 = swing.v[0] * swing.v[0] + swing.v[1] * swing.v[1];

    const cos_half_twist = @cos(twist / 2);
    const sin_half_twist = @sin(twist / 2);
    if (swing_squared > 0) {
        const theta_swing = @sqrt(swing_squared);
        const cos_half_theta_swing = @cos(theta_swing / 2);
        const sin_half_theta_swing_over_theta = @sin(theta_swing / 2) / theta_swing;

        quaternion.w = cos_half_theta_swing * cos_half_twist;
        quaternion.x = cos_half_theta_swing * sin_half_twist;
        quaternion.y = (swing.v[1] * cos_half_twist * sin_half_theta_swing_over_theta) - (swing.v[0] * sin_half_twist * sin_half_theta_swing_over_theta);
        quaternion.z = (swing.v[0] * cos_half_twist * sin_half_theta_swing_over_theta) + (swing.v[1] * sin_half_twist * sin_half_theta_swing_over_theta);
    } else {
        const sin_half_theta_over_theta = 0.5;

        quaternion.w = cos_half_twist;
        quaternion.x = sin_half_twist;
        quaternion.y = (swing.v[1] * cos_half_twist * sin_half_theta_over_theta) - (swing.v[0] * sin_half_twist * sin_half_theta_over_theta);
        quaternion.z = (swing.v[0] * cos_half_twist * sin_half_theta_over_theta) + (swing.v[1] * sin_half_twist * sin_half_theta_over_theta);
    }

    return quaternion;
}

fn enforeceQuaternionType(comptime T: type) void {
    if (!(T == common.Quaternion or T == common.Quaternionf)) {
        @compileError("A quaternion type (common.Quaternion, common.Quaternionf) is required here");
    }
}

pub fn normalizeAnyQuaternion(quaternion: anytype) @TypeOf(quaternion) {
    enforeceQuaternionType(@TypeOf(quaternion));

    const norm = @sqrt(quaternion.w * quaternion.w + quaternion.x * quaternion.x + quaternion.y * quaternion.y + quaternion.z * quaternion.z);
    return .{
        .w = quaternion.w / norm,
        .x = quaternion.x / norm,
        .y = quaternion.y / norm,
        .z = quaternion.z / norm,
    };
}
pub fn normalize(quaternion: common.Quaternion) common.Quaternion {
    return normalizeAnyQuaternion(quaternion);
}

pub fn quaternionFromEulerAngles(x: f64, y: f64, z: f64) common.Quaternion {
    const cx = @cos(x / 2);
    const sx = @sin(x / 2);
    const cy = @cos(y / 2);
    const sy = @sin(y / 2);
    const cz = @cos(z / 2);
    const sz = @sin(z / 2);

    return .{
        .w = cx * cy * cz + sx * sy * sz,
        .x = cx * sy * cz + sx * cy * sz,
        .y = cx * cy * sz - sx * sy * cz,
        .z = sx * cy * cz - cx * sy * sz,
    };
}

pub fn inverseAnyQuaterion(quaterion: anytype) @TypeOf(quaterion) {
    enforeceQuaternionType(@TypeOf(quaterion));

    return .{ .w = quaterion.w, .x = -quaterion.x, .y = -quaterion.y, .z = -quaterion.z };
}
pub fn inverseQuaterion(quaternion: common.Quaternion) common.Quaternion {
    return inverseAnyQuaterion(quaternion);
}

test "90 degree rotation on y" {
    const quat = quaternionFromMatrix(common.Matrix34{ .m = .{
        .{ 0, 0, 1, 0 },
        .{ 0, 1, 0, 0 },
        .{ 1, 0, 0, 0 },
    } });

    const expectEqual = std.testing.expectEqual;
    const expectApprox = std.testing.expectApproxEqAbs;

    const sqrt05 = std.math.sqrt(0.5);
    try expectApprox(sqrt05, quat.w, 1e-6);
    try expectEqual(0, quat.x);
    try expectApprox(sqrt05, quat.y, 1e-6);
    try expectEqual(0, quat.z);
}

// test "swing and twist" {
//     const quat = quaternionFromSwingTwist(.{ .v = .{ 1.0, 0.0 } }, std.math.pi / 2.0); //std.math.degreesToRadians(45));
//     std.debug.print("{}\n", .{quat});
// }
