// ==========================================================================
// Shared RGB -> luma front end + 4-pixel gray stream tap.
// --------------------------------------------------------------------------
// Phase-1 goal: keep ONE luma computation and reuse it for
//   * GRAY display,
//   * display EDGE (4 px/beat -> 2 horizontal samples/beat),
//   * POP brightness bands,
//   * the colour-independent foreground Sobel,
//   * a reserved 4 px/beat gray stream for a future circle detector.
//
// Luma is the same fixed-point approximation used by the baseline RTL:
//   Y = (R >> 2) + (G >> 1) + (B >> 2)      // 0..253, no 8-bit overflow
// This is NOT a new color space and must not be described as one.
//
// isp_rgb2y96  : pure combinational, 4 px/beat, O_y4 = {Y3,Y2,Y1,Y0}
//                where Y0 is the LSB (right-most) pixel of I_rgb.
//
// isp_gray_tap96: exactly one axi4s_video_aclk register on the shared luma
//                stream.  It is the ONLY place where the detection gray
//                stream is registered, so EDGE, display, and the reserved
//                detector port see the same pixel beat.
//
// Sampling coordinates (O_x/O_y):
//   O_x = x of the FIRST pixel in O_y4 (0,4,...,IMG_WIDTH-4)
//   O_y = row (0..IMG_HEIGHT-1)
//   O_tuser_det = 1 marks (0,0) and restarts the coordinate counters.
//   Counters advance only on I_valid; tlast is passed through unchanged.
//   IMG_WIDTH must be a multiple of 4.
//
// Two tuser inputs are intentional:
//   I_tuser_disp is the legacy early display tuser (AWB O_tuser, arrives
//   before valid).  It must stay on the display path so RAW/EDGE/GRAY/POP
//   keep the existing on-screen alignment.
//   I_tuser_det is the valid-aligned detection tuser (AWB O_tuser_alg).
//   The reserved gray stream exports this one, never the display tuser.
//
// Latency: 1 clock (I_* -> O_*).  No display-mode signal enters this module.
// ==========================================================================
module isp_rgb2y96 (
    input  wire [95:0] I_rgb,
    output wire [31:0] O_y4
);
    function [7:0] y_of;
        input [23:0] px;
        begin
            y_of = {2'b0,px[23:18]} + {1'b0,px[15:9]} + {2'b0,px[7:2]};
        end
    endfunction

    assign O_y4 = { y_of(I_rgb[95:72]),
                    y_of(I_rgb[71:48]),
                    y_of(I_rgb[47:24]),
                    y_of(I_rgb[23:0]) };
endmodule


module isp_gray_tap96 #(
    parameter integer IMG_WIDTH  = 1280,
    parameter integer IMG_HEIGHT = 720
)(
    input  wire        I_clk,
    input  wire        I_rst_n,
    input  wire [31:0] I_y4,
    input  wire        I_valid,
    input  wire        I_tlast,
    input  wire        I_tuser_disp,
    input  wire        I_tuser_det,
    output reg  [31:0] O_y4,
    output reg         O_valid,
    output reg         O_tlast,
    output reg         O_tuser_disp,
    output reg         O_tuser_det,
    output reg  [11:0] O_x,
    output reg  [11:0] O_y
);
    localparam integer LAST_X = IMG_WIDTH  - 4;
    localparam integer LAST_Y = IMG_HEIGHT - 1;

    reg [11:0] cx, cy;

    always @(posedge I_clk or negedge I_rst_n) begin
        if (!I_rst_n) begin
            O_y4        <= 32'd0;
            O_valid     <= 1'b0;
            O_tlast     <= 1'b0;
            O_tuser_disp<= 1'b0;
            O_tuser_det <= 1'b0;
            O_x         <= 12'd0;
            O_y         <= 12'd0;
            cx          <= 12'd0;
            cy          <= 12'd0;
        end
        else begin
            O_y4         <= I_y4;
            O_valid      <= I_valid;
            O_tlast      <= I_tlast;
            O_tuser_disp <= I_tuser_disp;
            O_tuser_det  <= I_tuser_det;

            if (I_valid) begin
                if (I_tuser_det) begin
                    // This beat is (0,0); the next beat is (4,0).
                    O_x <= 12'd0;
                    O_y <= 12'd0;
                    cx  <= 12'd4;
                    cy  <= 12'd0;
                end
                else begin
                    O_x <= cx;
                    O_y <= cy;
                    if (cx >= LAST_X) begin
                        cx <= 12'd0;
                        cy <= (cy >= LAST_Y) ? 12'd0 : cy + 12'd1;
                    end
                    else begin
                        cx <= cx + 12'd4;
                    end
                end
            end
        end
    end
endmodule
