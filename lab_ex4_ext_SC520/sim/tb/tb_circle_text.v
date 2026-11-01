`timescale 1ns/1ps
// ==========================================================================
// CIRCLE text + optional GREEN + runtime DD:HH:MM:SS overlay test.
// Real hdmi_mixer, synthetic 1280x720 raster, output pixels captured.
//   - dynamic 18/24-char CIRCLE text (GREEN only when I_det_green=1)
//   - 8 px glyphs, green + black outline, bottom/above placement, edge clamp
//   - positive/negative/zero coordinates, four corners, invalid-box removal,
//     multi-frame A->B->invalid
//   - runtime text: colon glyph, bottom-right position, 01:02:03:04 sequence,
//     box-over-runtime conflict (runtime fields must stay coherent)
// ==========================================================================
module tb_circle_text;
    localparam integer W = 1280, H = 720;
    localparam integer TXT_H = 10;
    localparam integer RT_X0 = 926, RT_Y0 = 666, RT_W = 330, RT_H = 30;

    reg clk = 0, rst_n = 0;
    always #5 clk = ~clk;

    reg        v_vsync=0, v_hsync=0, v_de=0, v_user=0, v_last=0;
    reg [23:0] v_data = 24'h202020;
    reg [11:0] d_minx=0,d_maxx=0,d_miny=0,d_maxy=0,d_cx=640,d_cy=360;
    reg        d_valid=0, d_green=0;
    reg [7:0]  rt_day=0, rt_hour=0, rt_min=0, rt_sec=0;

    wire O_rd_en, O_vsync, O_hsync, O_de;
    wire [23:0] O_data;

    hdmi_mixer #(.H_OFFSET(0), .V_OFFSET(0),
                 .IMG_WIDTH(W), .IMG_HEIGHT(H), .DEBUG_MODE(0)) u_dut (
        .I_clk(clk), .I_rst_n(rst_n),
        .I_video_vsync(v_vsync), .I_video_hsync(v_hsync),
        .I_video_de(v_de), .I_video_user(v_user), .I_video_last(v_last),
        .I_debug_status(4'd0), .I_fps_value(10'd0), .I_mode(2'd0),
        .I_lat(10'd0),
        .I_det_minx(d_minx), .I_det_maxx(d_maxx),
        .I_det_miny(d_miny), .I_det_maxy(d_maxy),
        .I_det_cx(d_cx), .I_det_cy(d_cy),
        .I_det_green(d_green), .I_det_valid(d_valid), .I_det_cand(1'b0),
        .I_det_cnt(10'd0), .I_det_flags(8'd0),
        .I_rt_day(rt_day), .I_rt_hour(rt_hour),
        .I_rt_min(rt_min), .I_rt_sec(rt_sec),
        .I_red_valid(1'b0), .I_red_minx(12'd0), .I_red_maxx(12'd0),
        .I_red_miny(12'd0), .I_red_maxy(12'd0),
        .I_avg_r(8'd0), .I_avg_g(8'd0), .I_avg_b(8'd0),
        .I_gr100(10'd0), .I_gb100(10'd0),
        .I_ae(16'd0), .I_ag(16'd0), .I_cal_st(2'd0),
        .O_video_rd_en(O_rd_en), .I_video_rd_data(v_data),
        .O_hdmi_vsync(O_vsync), .O_hdmi_hsync(O_hsync),
        .O_hdmi_de(O_de), .O_hdmi_data(O_data)
    );

    reg [23:0] cap [0:W*H-1];
    integer cap_ox=0, cap_oy=0;
    reg cap_en=0;
    always @(posedge clk) begin
        if (cap_en) begin
            if (!O_de) cap_ox = 0;
            else begin
                if (cap_oy < H) cap[cap_oy*W + cap_ox] = O_data;
                if (cap_ox == W-1) begin cap_ox=0; cap_oy=cap_oy+1; end
                else cap_ox=cap_ox+1;
            end
        end
    end

    reg [4:0] seq [0:23], exp_seq [0:23];
    reg [4:0] rt_seq [0:10];
    integer si;
    always @(posedge clk) begin
        if (cap_en && u_dut.S_txt_box_q &&
            (u_dut.S_txt_celly_q==4'd4) && (u_dut.S_txt_cellx_q==4'd5)) begin
            si = u_dut.S_txt_slot;
            if (si >= 0 && si < 24) seq[si] = u_dut.S_txt_ch_q;
        end
        if (cap_en && u_dut.S_rt_box_q &&
            (u_dut.S_rt_celly_q==4'd4) && (u_dut.S_rt_cellx_q==4'd5)) begin
            si = u_dut.S_rt_slot_d2;
            if (si >= 0 && si < 11) rt_seq[si] = u_dut.S_rt_ch_q;
        end
    end

    integer errors=0;
    task check;
        input cond;
        input [8*80-1:0] msg;
        begin
            if (cond) $display("  [ok]   %0s", msg);
            else begin $display("  [FAIL] %0s", msg); errors=errors+1; end
        end
    endtask

    task clear_seq;
        integer i;
        begin for (i=0;i<24;i=i+1) seq[i]=5'd15; end
    endtask
    task clear_rt;
        integer i;
        begin for (i=0;i<11;i=i+1) rt_seq[i]=5'd15; end
    endtask

    task set_exp;
        input [4:0] a0,a1,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12,a13,a14,a15,a16,a17,a18,a19,a20,a21,a22,a23;
        begin
            exp_seq[0]=a0; exp_seq[1]=a1; exp_seq[2]=a2; exp_seq[3]=a3;
            exp_seq[4]=a4; exp_seq[5]=a5; exp_seq[6]=a6; exp_seq[7]=a7;
            exp_seq[8]=a8; exp_seq[9]=a9; exp_seq[10]=a10; exp_seq[11]=a11;
            exp_seq[12]=a12; exp_seq[13]=a13; exp_seq[14]=a14; exp_seq[15]=a15;
            exp_seq[16]=a16; exp_seq[17]=a17; exp_seq[18]=a18; exp_seq[19]=a19;
            exp_seq[20]=a20; exp_seq[21]=a21; exp_seq[22]=a22; exp_seq[23]=a23;
        end
    endtask
    task check_seq;
        input integer n;
        input [8*40-1:0] name;
        integer i; reg ok;
        begin
            ok=1;
            for (i=0;i<n;i=i+1) if (seq[i]!==exp_seq[i]) begin
                ok=0; $display("  seq[%0d]=%0d exp %0d",i,seq[i],exp_seq[i]);
            end
            check(ok,name);
        end
    endtask

    task set_det;
        input [11:0] mnx,mxx,mny,mxy,cx,cy;
        input v, g;
        begin
            d_minx=mnx; d_maxx=mxx; d_miny=mny; d_maxy=mxy;
            d_cx=cx; d_cy=cy; d_valid=v; d_green=g;
        end
    endtask
    task set_rt;
        input [7:0] d,h,m,s;
        begin rt_day=d; rt_hour=h; rt_min=m; rt_sec=s; end
    endtask

    task drive_frame;
        integer yy,xx,tmo;
        begin
            cap_ox=0; cap_oy=0; cap_en=1; clear_seq; clear_rt;
            for (yy=0; yy<H; yy=yy+1) begin
                for (xx=0; xx<W; xx=xx+1) begin
                    @(negedge clk);
                    v_de=1; v_user=(yy==0 && xx==0);
                    v_last=(xx==W-1); v_data=24'h202020;
                end
                @(negedge clk); v_de=0; v_user=0; v_last=0;
                repeat (4) @(negedge clk);
            end
            tmo=0;
            while ((cap_oy<H) && (tmo<2000)) begin @(posedge clk); tmo=tmo+1; end
            cap_en=0;
            if (cap_oy<H) begin $display("  [FAIL] capture timeout"); errors=errors+1; end
            repeat (4) @(negedge clk);
        end
    endtask

    integer gcnt,bcnt,gminx,gmaxx,gminy,gmaxy,bminx,bminy,bmaxx,bmaxy;
    task scan_region;
        input integer x0,y0,w,h;
        integer xx,yy;
        begin
            gcnt=0; bcnt=0;
            gminx=W-1; gmaxx=0; gminy=H-1; gmaxy=0;
            bminx=W-1; bmaxx=0; bminy=H-1; bmaxy=0;
            for (yy=y0; yy<y0+h; yy=yy+1)
                for (xx=x0; xx<x0+w; xx=xx+1)
                    if (xx>=0 && xx<W && yy>=0 && yy<H) begin
                        if (cap[yy*W+xx]==24'h00FF00) begin
                            gcnt=gcnt+1;
                            if (xx<gminx) gminx=xx; if (xx>gmaxx) gmaxx=xx;
                            if (yy<gminy) gminy=yy; if (yy>gmaxy) gmaxy=yy;
                        end
                        if (cap[yy*W+xx]==24'h000000) begin
                            bcnt=bcnt+1;
                            if (xx<bminx) bminx=xx; if (xx>bmaxx) bmaxx=xx;
                            if (yy<bminy) bminy=yy; if (yy>bmaxy) bmaxy=yy;
                        end
                    end
        end
    endtask

    task dump_region;
        input integer x0,y0,w,h;
        input [8*32-1:0] tag;
        input integer mode;       // 0=text, 1=append text
        integer f,xx,yy;
        begin
            if (mode==0) f=$fopen("logs/text_render.txt","w");
            else         f=$fopen("logs/text_render.txt","a");
            $fwrite(f,"=== %0s ===\n",tag);
            for (yy=y0; yy<y0+h; yy=yy+1) begin
                for (xx=x0; xx<x0+w; xx=xx+1) begin
                    if (xx>=0 && xx<W && yy>=0 && yy<H) begin
                        if (cap[yy*W+xx]==24'h00FF00)      $fwrite(f,"G");
                        else if (cap[yy*W+xx]==24'h000000) $fwrite(f,"k");
                        else                               $fwrite(f,".");
                    end else $fwrite(f,".");
                end
                $fwrite(f,"\n");
            end
            $fclose(f);
        end
    endtask

    task dump_ppm;
        input integer x0,y0,w,h;
        input [8*40-1:0] name;
        integer f,xx,yy;
        begin
            f=$fopen(name,"w");
            $fwrite(f,"P3\n%0d %0d\n255\n",w,h);
            for (yy=y0; yy<y0+h; yy=yy+1)
                for (xx=x0; xx<x0+w; xx=xx+1) begin
                    if (xx>=0 && xx<W && yy>=0 && yy<H) begin
                        if (cap[yy*W+xx]==24'h00FF00)      $fwrite(f,"0 255 0\n");
                        else if (cap[yy*W+xx]==24'h000000) $fwrite(f,"0 0 0\n");
                        else                               $fwrite(f,"32 32 32\n");
                    end else $fwrite(f,"32 32 32\n");
                end
            $fclose(f);
        end
    endtask

    localparam [4:0] G_C=28,G_I=29,G_R=16,G_L=24,G_E=19,G_SP=15,
                     G_X=14,G_Y=22,G_PLUS=30,G_MINUS=31,
                     G_0=4,G_1=5,G_2=6,G_3=7,G_4=8,G_5=9,
                     G_G=25,G_N=26,G_COLON=3;
    integer x0,y0,w;
    initial begin
        repeat (10) @(negedge clk); rst_n=1; repeat (5) @(negedge clk);

        // ---------------- 1. centre, non-green X+123 Y-045 --------------
        $display("\n-- centre non-green: CIRCLE X+123 Y-045 --");
        set_det(500,700,400,600,763,315,1'b1,1'b0);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_PLUS,
                G_1,G_2,G_3,G_SP,G_Y,G_MINUS,G_0,G_4,G_5,
                G_SP,G_SP,G_SP,G_SP,G_SP,G_SP);
        drive_frame();
        x0=600-90; y0=400-14; w=180;
        check(u_dut.S_txt_x0_q==x0 && u_dut.S_txt_y0_q==y0 &&
              u_dut.S_txt_w_q==w, "non-green placement/size");
        check_seq(18, "non-green glyph sequence X+123 Y-045");
        scan_region(x0,y0,w,TXT_H);
        $display("  text: green=%0d black=%0d gbox=(%0d,%0d)-(%0d,%0d)",gcnt,bcnt,gminx,gminy,gmaxx,gmaxy);
        check(gcnt>100 && bcnt>100, "green glyphs + black outline");
        check((gmaxy-gminy+1)==8, "glyph height 8 px");
        check(bminx>=x0 && bminx<=x0+8 && bminy>=y0 && bminy<=y0+2,
              "text aligned to placement");
        dump_region(x0,y0,w,TXT_H,"centre non-green",0);
        dump_ppm(x0,y0,w,TXT_H,"logs/text_render.ppm");

        // ---------------- 2. zero ---------------------------------------
        $display("\n-- zero: CIRCLE X+000 Y+000 --");
        set_det(600,680,300,380,640,360,1'b1,1'b0);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_PLUS,
                G_0,G_0,G_0,G_SP,G_Y,G_PLUS,G_0,G_0,G_0,
                G_SP,G_SP,G_SP,G_SP,G_SP,G_SP);
        drive_frame();
        x0=640-90; y0=300-14; w=180;
        check_seq(18,"zero glyph sequence X+000 Y+000");
        scan_region(x0,y0,w,TXT_H);
        check(gcnt>100 && bcnt>100,"zero text rendered");

        // ---------------- 3. negative X ---------------------------------
        $display("\n-- negative X: CIRCLE X-045 Y+123 --");
        set_det(495,695,383,583,595,483,1'b1,1'b0);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_MINUS,
                G_0,G_4,G_5,G_SP,G_Y,G_PLUS,G_1,G_2,G_3,
                G_SP,G_SP,G_SP,G_SP,G_SP,G_SP);
        drive_frame();
        x0=595-90; y0=383-14; w=180;
        check_seq(18,"negative glyph sequence X-045 Y+123");
        scan_region(x0,y0,w,TXT_H);
        check(gcnt>100 && bcnt>100,"negative text rendered");

        // ---------------- 4. GREEN label --------------------------------
        $display("\n-- green label: CIRCLE GREEN X+123 Y-045 --");
        set_det(500,700,400,600,763,315,1'b1,1'b1);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_G,G_R,G_E,G_E,G_N,
                G_SP,G_X,G_PLUS,G_1,G_2,G_3,G_SP,G_Y,G_MINUS,
                G_0,G_4,G_5);
        drive_frame();
        x0=600-120; y0=400-14; w=240;
        check(u_dut.S_txt_x0_q==x0 && u_dut.S_txt_w_q==w,
              "green placement/size");
        check_seq(24,"GREEN glyph sequence");
        scan_region(x0,y0,w,TXT_H);
        check(gcnt>200 && bcnt>200,"GREEN text green/black pixels");
        dump_region(x0,y0,w,TXT_H,"green label",1);
        dump_ppm(x0,y0,w,TXT_H,"logs/text_render_green.ppm");

        // ---------------- 5. four corners (non-green) -------------------
        $display("\n-- four corners --");
        set_det(0,60,0,60,30,30,1'b1,1'b0);
        drive_frame();
        check(u_dut.S_txt_x0_q==0 && u_dut.S_txt_y0_q==64,
              "top-left clamp/place");
        scan_region(0,64,180,TXT_H);
        check(gmaxx<W && gmaxy<H,"top-left inside frame");

        set_det(1219,1279,0,60,1249,30,1'b1,1'b0);
        drive_frame();
        check(u_dut.S_txt_x0_q==1096 && u_dut.S_txt_y0_q==64,
              "top-right clamp");
        scan_region(1096,64,180,TXT_H);
        check(gmaxx<W && gmaxy<H,"top-right inside frame");

        set_det(0,60,659,719,30,689,1'b1,1'b0);
        drive_frame();
        check(u_dut.S_txt_x0_q==0 && u_dut.S_txt_y0_q==645,
              "bottom-left clamp/place");
        scan_region(0,645,180,TXT_H);
        check(gmaxx<W && gmaxy<H,"bottom-left inside frame");

        set_det(1219,1279,659,719,1249,689,1'b1,1'b0);
        drive_frame();
        check(u_dut.S_txt_x0_q==1096 && u_dut.S_txt_y0_q==645,
              "bottom-right clamp/place");
        scan_region(1096,645,180,TXT_H);
        check(gmaxx<W && gmaxy<H,"bottom-right inside frame");

        // ---------------- 6. invalid box: text disappears ---------------
        $display("\n-- invalid box --");
        set_det(500,700,400,600,763,315,1'b0,1'b0);
        drive_frame();
        scan_region(510,386,180,TXT_H);
        check(gcnt==0 && bcnt==0,"no text when invalid");
        check(u_dut.S_txt_on_q==0,"text enable cleared");

        // ---------------- 7. multi-frame A(green)->B(non-green)->invalid -
        $display("\n-- multi-frame A(green)->B(non-green)->invalid --");
        set_det(500,700,400,600,763,315,1'b1,1'b1);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_G,G_R,G_E,G_E,G_N,
                G_SP,G_X,G_PLUS,G_1,G_2,G_3,G_SP,G_Y,G_MINUS,G_0,G_4,G_5);
        drive_frame();
        check_seq(24,"frame A GREEN");
        set_det(495,695,383,583,595,483,1'b1,1'b0);
        set_exp(G_C,G_I,G_R,G_C,G_L,G_E,G_SP,G_X,G_MINUS,
                G_0,G_4,G_5,G_SP,G_Y,G_PLUS,G_1,G_2,G_3,
                G_SP,G_SP,G_SP,G_SP,G_SP,G_SP);
        drive_frame();
        check_seq(18,"frame B non-green (GREEN gone)");
        check(u_dut.S_txt_x0_q==(595-90), "frame B width back to 180");
        set_det(500,700,400,600,763,315,1'b0,1'b0);
        drive_frame();
        scan_region(505,369,180,TXT_H);
        check(gcnt==0,"frame B region empty after invalid");

        // ---------------- 8. runtime DD:HH:MM:SS ------------------------
        $display("\n-- runtime 01:02:03:04 --");
        set_det(0,0,0,0,640,360,1'b0,1'b0);
        set_rt(8'd1,8'd2,8'd3,8'd4);
        drive_frame();
        check(rt_seq[0]==G_0 && rt_seq[1]==G_1 && rt_seq[2]==G_COLON &&
              rt_seq[3]==G_0 && rt_seq[4]==G_2 && rt_seq[5]==G_COLON &&
              rt_seq[6]==G_0 && rt_seq[7]==G_3 && rt_seq[8]==G_COLON &&
              rt_seq[9]==G_0 && rt_seq[10]==G_4,
              "runtime glyph sequence 01:02:03:04");
        scan_region(RT_X0,RT_Y0,RT_W,RT_H);
        check(gcnt>500 && bcnt>500,"runtime green glyphs + black outline");
        check(gminx>=RT_X0 && gmaxx<RT_X0+RT_W &&
              gminy>=RT_Y0 && gmaxy<RT_Y0+RT_H,
              "runtime inside bottom-right region");
        dump_region(RT_X0,RT_Y0,RT_W,RT_H,"runtime 01:02:03:04",1);
        dump_ppm(RT_X0,RT_Y0,RT_W,RT_H,"logs/rt_render.ppm");

        // ---------------- 9. conflict: box edge over runtime -------------
        $display("\n-- conflict: box over runtime, fields coherent --");
        set_det(950,1150,650,719,1050,685,1'b1,1'b0);
        set_rt(8'd99,8'd23,8'd59,8'd59);
        drive_frame();
        check(rt_seq[0]==5'd13 && rt_seq[1]==5'd13 &&  // day 99 -> glyph 13
              rt_seq[3]==5'd6 && rt_seq[4]==5'd7 &&    // hour 23
              rt_seq[6]==5'd9 && rt_seq[7]==5'd13 &&   // min 59
              rt_seq[9]==5'd9 && rt_seq[10]==5'd13,    // sec 59
              "runtime fields coherent with a box overlapping");
        scan_region(RT_X0,RT_Y0,RT_W,RT_H);
        check(gcnt>200,"runtime still visible under box conflict");

        $display("\n================================================");
        if (errors==0) $display("RESULT: PASS (0 errors)");
        else           $display("RESULT: FAIL (%0d errors)",errors);
        $display("================================================");
        $finish;
    end
endmodule
