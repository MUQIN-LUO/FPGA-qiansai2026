// ==========================================================================
// Runtime clock DD:HH:MM:SS with an atomic CDC transfer.
// --------------------------------------------------------------------------
// Source domain: stable 50 MHz system clock.  A fixed CLK_DIV divider makes a
// 1 Hz tick, independent of the video frame rate.  Second/minute/hour/day are
// incremented with rollover and saturate at 99:23:59:59.  Reset clears.
//
// The whole {day,hour,min,sec} group is copied into a snapshot and moved to
// the HDMI pixel-clock domain with a request/ack toggle handshake: the source
// never rewrites the snapshot until the destination has acknowledged, so the
// destination always sees one coherent timestamp (no mixed 59/00 fields).
// ==========================================================================
module isp_rtc_cdc #(
    parameter integer CLK_DIV = 50_000_000,
    parameter [7:0]   INIT_DAY = 8'd0,
    parameter [7:0]   INIT_HOUR = 8'd0,
    parameter [7:0]   INIT_MIN = 8'd0,
    parameter [7:0]   INIT_SEC = 8'd0
)(
    input  wire       src_clk,
    input  wire       src_rst_n,
    input  wire       dst_clk,
    input  wire       dst_rst_n,
    output wire [7:0] day,
    output wire [7:0] hour,
    output wire [7:0] minute,
    output wire [7:0] second
);
    // ---------------- source domain (50 MHz) ---------------------------
    reg [31:0] div_cnt;
    reg [7:0]  s_day, s_hour, s_min, s_sec;
    reg        tick_d;
    reg [7:0]  sn_day, sn_hour, sn_min, sn_sec;
    reg        req;
    reg        pending;
    reg        ack_s1, ack_s2;
    reg        ack;             // destination-domain ack, 2FF-synced above

    wire tick = (div_cnt == (CLK_DIV-1));

    always @(posedge src_clk or negedge src_rst_n) begin
        if (!src_rst_n) begin
            div_cnt<=32'd0; tick_d<=1'b0;
            s_day<=INIT_DAY; s_hour<=INIT_HOUR;
            s_min<=INIT_MIN; s_sec<=INIT_SEC;
            sn_day<=8'd0; sn_hour<=8'd0; sn_min<=8'd0; sn_sec<=8'd0;
            req<=1'b0; pending<=1'b0; ack_s1<=1'b0; ack_s2<=1'b0;
        end
        else begin
            tick_d <= tick;
            ack_s1 <= ack;
            ack_s2 <= ack_s1;

            if (tick) begin
                div_cnt <= 32'd0;
                if (!((s_day==8'd99) && (s_hour==8'd23) &&
                      (s_min==8'd59) && (s_sec==8'd59))) begin
                    if (s_sec != 8'd59) s_sec <= s_sec + 8'd1;
                    else begin
                        s_sec <= 8'd0;
                        if (s_min != 8'd59) s_min <= s_min + 8'd1;
                        else begin
                            s_min <= 8'd0;
                            if (s_hour != 8'd23) s_hour <= s_hour + 8'd1;
                            else begin
                                s_hour <= 8'd0;
                                if (s_day != 8'd99) s_day <= s_day + 8'd1;
                            end
                        end
                    end
                end
            end
            else div_cnt <= div_cnt + 32'd1;

            // tick_d is one cycle after the 1 Hz increment, so the snapshot
            // sees the new fields.
            if ((req == ack_s2) && (tick_d || pending)) begin
                sn_day<=s_day; sn_hour<=s_hour;
                sn_min<=s_min; sn_sec<=s_sec;
                req<=~req; pending<=1'b0;
            end
            else if (tick_d && (req != ack_s2)) begin
                pending<=1'b1;
            end
        end
    end

    // ---------------- destination domain (HDMI pixel clock) -----------
    reg        req_s1, req_s2;
    reg [7:0]  d_day, d_hour, d_min, d_sec;

    always @(posedge dst_clk or negedge dst_rst_n) begin
        if (!dst_rst_n) begin
            req_s1<=1'b0; req_s2<=1'b0; ack<=1'b0;
            d_day<=8'd0; d_hour<=8'd0; d_min<=8'd0; d_sec<=8'd0;
        end
        else begin
            req_s1 <= req;
            req_s2 <= req_s1;
            if (req_s2 != ack) begin
                d_day<=sn_day; d_hour<=sn_hour;
                d_min<=sn_min; d_sec<=sn_sec;
                ack<=req_s2;
            end
        end
    end

    assign day    = d_day;
    assign hour   = d_hour;
    assign minute = d_min;
    assign second = d_sec;
endmodule
