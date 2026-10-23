/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Sep 07 2026 13:20:27
**	TD version	:	6.2.168116
************************************************************/
module ChipWatcher_mipi
(
  input   [15:0]                probe0,
  input   [0:0]                 probe1,
  input   [0:0]                 probe2,
  input   [8:0]                 probe3,
  input   [10:0]                probe4,
  input   [0:0]                 probe5,
  input   [0:0]                 probe6,
  input   [0:0]                 probe7,
  input   [0:0]                 probe8,
  input                         clk
);

  ChipWatcher_aef86b9bb9fc  ChipWatcher_aef86b9bb9fc_Inst
  (
      .probe0(probe0),
      .probe1(probe1),
      .probe2(probe2),
      .probe3(probe3),
      .probe4(probe4),
      .probe5(probe5),
      .probe6(probe6),
      .probe7(probe7),
      .probe8(probe8),
      .clk(clk)
  );
endmodule
