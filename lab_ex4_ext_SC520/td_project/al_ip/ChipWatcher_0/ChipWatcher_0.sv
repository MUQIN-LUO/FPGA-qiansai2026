/************************************************************\
**	Copyright (c) 2012-2025 Anlogic Inc.
**	All Right Reserved.
\************************************************************/
/************************************************************\
**	Build time: Sep 07 2026 13:02:56
**	TD version	:	6.2.168116
************************************************************/
module ChipWatcher_0
(
  input   [15:0]                probe0,
  input   [0:0]                 probe1,
  input   [10:0]                probe2,
  input   [0:0]                 probe3,
  input   [0:0]                 probe4,
  input   [0:0]                 probe5,
  input                         clk
);

  ChipWatcher_16ce29dbbb24  ChipWatcher_16ce29dbbb24_Inst
  (
      .probe0(probe0),
      .probe1(probe1),
      .probe2(probe2),
      .probe3(probe3),
      .probe4(probe4),
      .probe5(probe5),
      .clk(clk)
  );
endmodule
