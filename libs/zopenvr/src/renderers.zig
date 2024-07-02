const config = @import("rendermodesConfig");

pub const d3d12 = if (config.d3d12) @import("zwin32").d3d12 else struct {
    pub const ICommandQueue = anyopaque;
    pub const IResource = anyopaque;
};

pub const d3d11 = if (config.d3d11) @import("zwin32").d3d11 else struct {
    pub const IShaderResourceView = anyopaque;
    pub const IResource = anyopaque;
    pub const ITexture2D = anyopaque;
};