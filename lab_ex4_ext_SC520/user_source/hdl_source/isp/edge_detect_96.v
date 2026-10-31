// ==========================================================================
// Backward-compatible RGB wrapper for the Sobel core.
// --------------------------------------------------------------------------
// The actual datapath now lives in edge_detect_y96.v and consumes the shared
// 4-pixel luma stream.  This wrapper keeps the original RGB port list so the
// existing testbenches (tb_frontend, tb_det_frontend_awb) still instantiate
// exactly the same RTL block.  It computes the same Y = R/4 + G/2 + B/4 as
// before and therefore produces bit-identical edge pixels.
// ==========================================================================
module edge_detect_96 #(
    parameter IMG_WIDTH  = 1280,
    parameter IMG_HEIGHT = 720,
    parameter EDGE_TH    = 11'd80
)(
    input               I_clk,
    input               I_rst_n,
    input               I_tlast,
    input               I_tuser,
    input      [95:0]   I_tdata,
    input               I_tvalid,
    input               I_tready,
    output wire         O_tlast,
    output wire         O_tuser,
    output wire [95:0]  O_tdata,
    output wire         O_tvalid,
    output              O_tready
);

function [7:0] y_of;
    input [23:0] px;
    begin
        y_of = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
    end
endfunction

wire [31:0] y4 = { y_of(I_tdata[95:72]),
                   y_of(I_tdata[71:48]),
                   y_of(I_tdata[47:24]),
                   y_of(I_tdata[23:0]) };

edge_detect_y96 #(
    .IMG_WIDTH  (IMG_WIDTH ),
    .IMG_HEIGHT (IMG_HEIGHT),
    .EDGE_TH    (EDGE_TH   )
) u_core (
    .I_clk   (I_clk),
    .I_rst_n (I_rst_n),
    .I_tlast (I_tlast),
    .I_tuser (I_tuser),
    .I_y4    (y4),
    .I_tvalid(I_tvalid),
    .I_tready(I_tready),
    .O_tlast (O_tlast),
    .O_tuser (O_tuser),
    .O_tdata (O_tdata),
    .O_tvalid(O_tvalid),
    .O_tready(O_tready)
);

endmodule
