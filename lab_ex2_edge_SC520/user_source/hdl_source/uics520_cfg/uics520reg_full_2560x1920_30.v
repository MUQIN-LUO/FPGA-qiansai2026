`timescale 1ns / 1ps

/*
 * SC520CS 2560x1920 @ 30 fps 备用完整寄存器配置
 * ------------------------------------------------------------
 * 本文件逐项转换自原厂0x9F INI：19.2 MHz MCLK、2-Lane MIPI、
 * RAW10、896 Mbps/Lane，Sensor直接输出2560x1920，约30 fps。
 * 输出尺寸寄存器0x3208~0x320B为0x0A00 x 0x0780；模式值
 * 0x301F=0x9F。寄存器的顺序、重复写入及数值均按原INI保留。
 * 原INI中的0x550F=0x42也原样保留；若使用SC520CS-03料号，
 * 应依据原厂料号说明确认是否需要改为0x8A。
 *
 * 当前工程默认使用uics520reg_720p60.v，本文件仅作为Full模式
 * 备用Setting，不应与720p60配置混合使用。启用本文件时，必须
 * 同步修改/检查：Sensor MCLK改为19.2 MHz、MIPI 896 Mbps/Lane
 * 接收约束、uicfg实例、RAW10行长度、uial2axis及ISP宽高、DDR
 * 帧缓存尺寸和带宽，以及最终HDMI裁剪/缩放与显示时序。2560x1920
 * 不是常用HDMI标准时序，不能只替换寄存器表后直接完整显示。
 */

module uics520reg_full_2560x1920_30 (
    input  wire [8:0]  REG_INDEX,
    output reg  [23:0] REG_DATA,
    output wire [8:0]  REG_SIZE
);
    assign REG_SIZE = 9'd159;

    always @(*) begin
        case (REG_INDEX)
              9'd0: REG_DATA = {16'h0103, 8'h01};
              9'd1: REG_DATA = {16'h36e9, 8'h80};
              9'd2: REG_DATA = {16'h37f9, 8'h80};
              9'd3: REG_DATA = {16'h36ea, 8'h0e};
              9'd4: REG_DATA = {16'h36eb, 8'h0c};
              9'd5: REG_DATA = {16'h36ec, 8'h4a};
              9'd6: REG_DATA = {16'h36ed, 8'h28};
              9'd7: REG_DATA = {16'h37fa, 8'h13};
              9'd8: REG_DATA = {16'h37fb, 8'h64};
              9'd9: REG_DATA = {16'h37fc, 8'h10};
             9'd10: REG_DATA = {16'h37fd, 8'h28};
             9'd11: REG_DATA = {16'h36e9, 8'h44};
             9'd12: REG_DATA = {16'h37f9, 8'h20};
             9'd13: REG_DATA = {16'h301f, 8'h9f};
             9'd14: REG_DATA = {16'h3200, 8'h00};
             9'd15: REG_DATA = {16'h3201, 8'h00};
             9'd16: REG_DATA = {16'h3202, 8'h00};
             9'd17: REG_DATA = {16'h3203, 8'h0c};
             9'd18: REG_DATA = {16'h3204, 8'h0a};
             9'd19: REG_DATA = {16'h3205, 8'h27};
             9'd20: REG_DATA = {16'h3206, 8'h07};
             9'd21: REG_DATA = {16'h3207, 8'h9b};
             9'd22: REG_DATA = {16'h3208, 8'h0a};
             9'd23: REG_DATA = {16'h3209, 8'h00};
             9'd24: REG_DATA = {16'h320a, 8'h07};
             9'd25: REG_DATA = {16'h320b, 8'h80};
             9'd26: REG_DATA = {16'h320c, 8'h05};
             9'd27: REG_DATA = {16'h320d, 8'hf0};
             9'd28: REG_DATA = {16'h3210, 8'h00};
             9'd29: REG_DATA = {16'h3211, 8'h14};
             9'd30: REG_DATA = {16'h3212, 8'h00};
             9'd31: REG_DATA = {16'h3213, 8'h08};
             9'd32: REG_DATA = {16'h3301, 8'h08};
             9'd33: REG_DATA = {16'h3306, 8'h60};
             9'd34: REG_DATA = {16'h3308, 8'h10};
             9'd35: REG_DATA = {16'h3309, 8'h80};
             9'd36: REG_DATA = {16'h330b, 8'hd0};
             9'd37: REG_DATA = {16'h3314, 8'h15};
             9'd38: REG_DATA = {16'h331f, 8'h71};
             9'd39: REG_DATA = {16'h3333, 8'h10};
             9'd40: REG_DATA = {16'h3334, 8'h40};
             9'd41: REG_DATA = {16'h3364, 8'h56};
             9'd42: REG_DATA = {16'h336c, 8'hcf};
             9'd43: REG_DATA = {16'h3390, 8'h09};
             9'd44: REG_DATA = {16'h3391, 8'h0f};
             9'd45: REG_DATA = {16'h3392, 8'h1f};
             9'd46: REG_DATA = {16'h3393, 8'h0e};
             9'd47: REG_DATA = {16'h3394, 8'h15};
             9'd48: REG_DATA = {16'h3395, 8'h19};
             9'd49: REG_DATA = {16'h33ad, 8'h1c};
             9'd50: REG_DATA = {16'h33af, 8'h3f};
             9'd51: REG_DATA = {16'h33b3, 8'h30};
             9'd52: REG_DATA = {16'h33b5, 8'h90};
             9'd53: REG_DATA = {16'h349f, 8'h1e};
             9'd54: REG_DATA = {16'h34a6, 8'h09};
             9'd55: REG_DATA = {16'h34a7, 8'h0b};
             9'd56: REG_DATA = {16'h34a8, 8'h20};
             9'd57: REG_DATA = {16'h34a9, 8'h20};
             9'd58: REG_DATA = {16'h34f8, 8'h0f};
             9'd59: REG_DATA = {16'h34f9, 8'h10};
             9'd60: REG_DATA = {16'h3630, 8'h88};
             9'd61: REG_DATA = {16'h3633, 8'h34};
             9'd62: REG_DATA = {16'h3635, 8'hc0};
             9'd63: REG_DATA = {16'h3637, 8'h52};
             9'd64: REG_DATA = {16'h3638, 8'hc1};
             9'd65: REG_DATA = {16'h363a, 8'h89};
             9'd66: REG_DATA = {16'h3670, 8'h4a};
             9'd67: REG_DATA = {16'h3674, 8'h48};
             9'd68: REG_DATA = {16'h3675, 8'h28};
             9'd69: REG_DATA = {16'h3676, 8'h18};
             9'd70: REG_DATA = {16'h367c, 8'h0f};
             9'd71: REG_DATA = {16'h367d, 8'h1f};
             9'd72: REG_DATA = {16'h3690, 8'h43};
             9'd73: REG_DATA = {16'h3691, 8'h43};
             9'd74: REG_DATA = {16'h3692, 8'h43};
             9'd75: REG_DATA = {16'h3698, 8'h89};
             9'd76: REG_DATA = {16'h3699, 8'h92};
             9'd77: REG_DATA = {16'h369a, 8'ha5};
             9'd78: REG_DATA = {16'h369b, 8'hca};
             9'd79: REG_DATA = {16'h369c, 8'h0b};
             9'd80: REG_DATA = {16'h369d, 8'h0f};
             9'd81: REG_DATA = {16'h36a2, 8'h09};
             9'd82: REG_DATA = {16'h36a3, 8'h0b};
             9'd83: REG_DATA = {16'h36a4, 8'h0f};
             9'd84: REG_DATA = {16'h36b1, 8'h38};
             9'd85: REG_DATA = {16'h36b2, 8'hc1};
             9'd86: REG_DATA = {16'h370f, 8'h01};
             9'd87: REG_DATA = {16'h3724, 8'hb1};
             9'd88: REG_DATA = {16'h3771, 8'h02};
             9'd89: REG_DATA = {16'h3772, 8'h03};
             9'd90: REG_DATA = {16'h3773, 8'h03};
             9'd91: REG_DATA = {16'h377a, 8'h0b};
             9'd92: REG_DATA = {16'h377b, 8'h0f};
             9'd93: REG_DATA = {16'h3901, 8'h04};
             9'd94: REG_DATA = {16'h3905, 8'h8d};
             9'd95: REG_DATA = {16'h391d, 8'h01};
             9'd96: REG_DATA = {16'h3926, 8'h23};
             9'd97: REG_DATA = {16'h3c04, 8'h01};
             9'd98: REG_DATA = {16'h3e00, 8'h00};
             9'd99: REG_DATA = {16'h3e01, 8'hf9};
            9'd100: REG_DATA = {16'h3e02, 8'h70};
            9'd101: REG_DATA = {16'h3e09, 8'h00};
            9'd102: REG_DATA = {16'h4401, 8'h13};
            9'd103: REG_DATA = {16'h4402, 8'h02};
            9'd104: REG_DATA = {16'h4403, 8'h0a};
            9'd105: REG_DATA = {16'h4404, 8'h1c};
            9'd106: REG_DATA = {16'h4405, 8'h24};
            9'd107: REG_DATA = {16'h4407, 8'h0c};
            9'd108: REG_DATA = {16'h440c, 8'h2e};
            9'd109: REG_DATA = {16'h440d, 8'h2e};
            9'd110: REG_DATA = {16'h440e, 8'h23};
            9'd111: REG_DATA = {16'h440f, 8'h39};
            9'd112: REG_DATA = {16'h4412, 8'h01};
            9'd113: REG_DATA = {16'h4424, 8'h01};
            9'd114: REG_DATA = {16'h442d, 8'h00};
            9'd115: REG_DATA = {16'h442e, 8'h00};
            9'd116: REG_DATA = {16'h4509, 8'h28};
            9'd117: REG_DATA = {16'h450d, 8'h19};
            9'd118: REG_DATA = {16'h451d, 8'h28};
            9'd119: REG_DATA = {16'h4526, 8'h05};
            9'd120: REG_DATA = {16'h4819, 8'h0e};
            9'd121: REG_DATA = {16'h481b, 8'h08};
            9'd122: REG_DATA = {16'h481d, 8'h1e};
            9'd123: REG_DATA = {16'h481f, 8'h06};
            9'd124: REG_DATA = {16'h4821, 8'h0e};
            9'd125: REG_DATA = {16'h4823, 8'h07};
            9'd126: REG_DATA = {16'h4825, 8'h06};
            9'd127: REG_DATA = {16'h4827, 8'h07};
            9'd128: REG_DATA = {16'h4829, 8'h0c};
            9'd129: REG_DATA = {16'h5000, 8'h0e};
            9'd130: REG_DATA = {16'h5007, 8'h00};
            9'd131: REG_DATA = {16'h550e, 8'h00};
            9'd132: REG_DATA = {16'h550f, 8'h42};
            9'd133: REG_DATA = {16'h5780, 8'h66};
            9'd134: REG_DATA = {16'h5787, 8'h08};
            9'd135: REG_DATA = {16'h5788, 8'h04};
            9'd136: REG_DATA = {16'h5789, 8'h01};
            9'd137: REG_DATA = {16'h578d, 8'h40};
            9'd138: REG_DATA = {16'h5790, 8'h08};
            9'd139: REG_DATA = {16'h5791, 8'h04};
            9'd140: REG_DATA = {16'h5792, 8'h01};
            9'd141: REG_DATA = {16'h5799, 8'h46};
            9'd142: REG_DATA = {16'h57aa, 8'heb};
            9'd143: REG_DATA = {16'h59f0, 8'h00};
            9'd144: REG_DATA = {16'h5ae0, 8'hfe};
            9'd145: REG_DATA = {16'h5ae1, 8'h40};
            9'd146: REG_DATA = {16'h5ae2, 8'h38};
            9'd147: REG_DATA = {16'h5ae3, 8'h30};
            9'd148: REG_DATA = {16'h5ae4, 8'h0c};
            9'd149: REG_DATA = {16'h5ae5, 8'h38};
            9'd150: REG_DATA = {16'h5ae6, 8'h30};
            9'd151: REG_DATA = {16'h5ae7, 8'h28};
            9'd152: REG_DATA = {16'h5ae8, 8'h3f};
            9'd153: REG_DATA = {16'h5ae9, 8'h34};
            9'd154: REG_DATA = {16'h5aea, 8'h2c};
            9'd155: REG_DATA = {16'h5aeb, 8'h3f};
            9'd156: REG_DATA = {16'h5aec, 8'h34};
            9'd157: REG_DATA = {16'h5aed, 8'h2c};
            9'd158: REG_DATA = {16'h0100, 8'h01};
            default: REG_DATA = {16'h0000, 8'h00};
        endcase
    end
endmodule


