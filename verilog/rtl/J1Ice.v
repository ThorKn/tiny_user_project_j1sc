// Generator : SpinalHDL v1.4.2    git head : 804c7bd7b7feaddcc1d25ecef6c208fd5f776f79
// Component : J1Ice
// Git hash  : 7a0056e38fc4ed1d0e62e373fd997deb12e593d6


`define UartStopType_defaultEncoding_type [0:0]
`define UartStopType_defaultEncoding_ONE 1'b0
`define UartStopType_defaultEncoding_TWO 1'b1

`define UartParityType_defaultEncoding_type [1:0]
`define UartParityType_defaultEncoding_NONE 2'b00
`define UartParityType_defaultEncoding_EVEN 2'b01
`define UartParityType_defaultEncoding_ODD 2'b10

`define jtagFSM_enumDefinition_defaultEncoding_type [4:0]
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_BOOT 5'b00000
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset 5'b00001
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle 5'b00010
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan 5'b00011
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR 5'b00100
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR 5'b00101
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR 5'b00110
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR 5'b00111
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR 5'b01000
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR 5'b01001
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan 5'b01010
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR 5'b01011
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR 5'b01100
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR 5'b01101
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR 5'b01110
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR 5'b01111
`define jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR 5'b10000

`define UartCtrlTxState_defaultEncoding_type [2:0]
`define UartCtrlTxState_defaultEncoding_IDLE 3'b000
`define UartCtrlTxState_defaultEncoding_START 3'b001
`define UartCtrlTxState_defaultEncoding_DATA 3'b010
`define UartCtrlTxState_defaultEncoding_PARITY 3'b011
`define UartCtrlTxState_defaultEncoding_STOP 3'b100

`define UartCtrlRxState_defaultEncoding_type [2:0]
`define UartCtrlRxState_defaultEncoding_IDLE 3'b000
`define UartCtrlRxState_defaultEncoding_START 3'b001
`define UartCtrlRxState_defaultEncoding_DATA 3'b010
`define UartCtrlRxState_defaultEncoding_PARITY 3'b011
`define UartCtrlRxState_defaultEncoding_STOP 3'b100

module thorkn_j1sc_top (

  input [7:0] in,
  output [2:0] out,
);

  J1Ice J1Ice (in[0], in[1], in[2], in[3], in[4], out[0], out[1], in[5], out[2], in[6], in[7]);
  /*********** rst,   clk,   clkL,  eInt , rx   , tx    , uLed  , tdi  , tdo   , tms  , tck   ****/ 
endmodule

module J1Ice (
  input               reset,
  input               boardClk,
  input               boardClkLocked,
  input      [0:0]    extInt,
  input               rx,
  output              tx,
  output              uartLed,
  input               tdi,
  output              tdo,
  input               tms,
  input               tck
);
  wire                _zz_20;
  wire       [7:0]    _zz_21;
  wire       [15:0]   _zz_22;
  wire       [15:0]   _zz_23;
  wire       [15:0]   _zz_24;
  wire       [15:0]   _zz_25;
  wire       [15:0]   _zz_26;
  wire       [15:0]   _zz_27;
  wire                _zz_28;
  reg                 _zz_29;
  wire                _zz_30;
  reg        [3:0]    _zz_31;
  reg        [3:0]    _zz_32;
  wire       [15:0]   _zz_33;
  wire                jtagIface_jtagArea_jtag_tdo;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_valid;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory;
  wire       [7:0]    jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr;
  wire       [15:0]   jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord;
  wire                jtagIface_jtagArea_jtag_jtagReset;
  wire                bufferCC_4_io_dataOut;
  wire                coreArea_cpu_cpuBus_enable;
  wire                coreArea_cpu_cpuBus_writeMode;
  wire       [15:0]   coreArea_cpu_cpuBus_address;
  wire       [15:0]   coreArea_cpu_cpuBus_writeData;
  wire                flowCCByToggle_1_io_output_valid;
  wire                flowCCByToggle_1_io_output_payload_jtagDataValid;
  wire                flowCCByToggle_1_io_output_payload_jtagStall;
  wire                flowCCByToggle_1_io_output_payload_jtagCaptureMemory;
  wire       [7:0]    flowCCByToggle_1_io_output_payload_jtagCPUAdr;
  wire       [15:0]   flowCCByToggle_1_io_output_payload_jtagCPUWord;
  wire       [15:0]   coreArea_timerA_enableState;
  wire       [15:0]   coreArea_timerA_highState;
  wire       [15:0]   coreArea_timerA_lowState;
  wire                coreArea_timerA_interrupt;
  wire       [15:0]   coreArea_timerB_enableState;
  wire       [15:0]   coreArea_timerB_highState;
  wire       [15:0]   coreArea_timerB_lowState;
  wire                coreArea_timerB_interrupt;
  wire                coreArea_uartCtrl_io_write_ready;
  wire                coreArea_uartCtrl_io_read_valid;
  wire       [7:0]    coreArea_uartCtrl_io_read_payload;
  wire                coreArea_uartCtrl_io_uart_txd;
  wire                coreArea_uartCtrl_io_readError;
  wire                coreArea_uartCtrl_io_readBreak;
  wire                coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  wire                coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload;
  wire       [3:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy;
  wire       [3:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_availability;
  wire                coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready;
  wire                coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload;
  wire       [3:0]    coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy;
  wire       [3:0]    coreArea_uartCtrl_io_read_queueWithOccupancy_io_availability;
  wire       [15:0]   coreArea_intCtrl_irqGetMask;
  wire       [15:0]   coreArea_intCtrl_irqVectors_0;
  wire       [15:0]   coreArea_intCtrl_irqVectors_1;
  wire       [15:0]   coreArea_intCtrl_irqVectors_2;
  wire       [15:0]   coreArea_intCtrl_irqVectors_3;
  wire       [15:0]   coreArea_intCtrl_intVec;
  wire                coreArea_intCtrl_irq;
  wire                _zz_34;
  wire       [0:0]    _zz_35;
  wire       [0:0]    _zz_36;
  wire       [0:0]    _zz_37;
  wire       [0:0]    _zz_38;
  wire       [0:0]    _zz_39;
  wire       [0:0]    _zz_40;
  wire       [0:0]    _zz_41;
  wire       [0:0]    _zz_42;
  wire       [0:0]    _zz_43;
  wire       [0:0]    _zz_44;
  wire       [0:0]    _zz_45;
  wire       [19:0]   _zz_46;
  wire       [3:0]    _zz_47;
  wire       [0:0]    _zz_48;
  wire       [0:0]    _zz_49;
  wire                core_clk;
  wire                core_reset;
  wire                _zz_1;
  reg                 _zz_2;
  reg                 flowCCByToggle_1_io_output_payload_regNextWhen_jtagDataValid;
  reg                 flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall;
  reg                 flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory;
  reg        [7:0]    flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr;
  reg        [15:0]   flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord;
  wire                coreArea_peripheralBus_enable;
  wire                coreArea_peripheralBus_writeMode;
  wire       [15:0]   coreArea_peripheralBus_address;
  wire       [15:0]   coreArea_peripheralBus_writeData;
  reg        [15:0]   coreArea_peripheralBus_readData;
  reg        [15:0]   coreArea_cpu_cpuBus_address_delay_1;
  reg                 coreArea_cpu_cpuBus_enable_delay_1;
  reg                 coreArea_cpu_cpuBus_writeMode_delay_1;
  reg        [15:0]   coreArea_cpu_cpuBus_writeData_delay_1;
  wire                coreArea_peripheralBusCtrl_askWrite;
  wire                coreArea_peripheralBusCtrl_askRead;
  reg                 _zz_3;
  reg                 _zz_4;
  reg                 _zz_5;
  reg                 _zz_6;
  reg                 _zz_7;
  reg                 _zz_8;
  wire       [2:0]    coreArea_uartBridge_uartConfigReg_frame_dataLength;
  wire       `UartStopType_defaultEncoding_type coreArea_uartBridge_uartConfigReg_frame_stop;
  wire       `UartParityType_defaultEncoding_type coreArea_uartBridge_uartConfigReg_frame_parity;
  reg        [19:0]   coreArea_uartBridge_uartConfigReg_clockDivider;
  reg                 _zz_9;
  wire                coreArea_uartBridge_write_streamUnbuffered_valid;
  wire                coreArea_uartBridge_write_streamUnbuffered_ready;
  wire       [7:0]    coreArea_uartBridge_write_streamUnbuffered_payload;
  reg                 coreArea_uartBridge_read_streamBreaked_valid;
  reg                 coreArea_uartBridge_read_streamBreaked_ready;
  wire       [7:0]    coreArea_uartBridge_read_streamBreaked_payload;
  reg                 coreArea_uartBridge_interruptCtrl_writeIntEnable;
  reg                 coreArea_uartBridge_interruptCtrl_readIntEnable;
  wire                coreArea_uartBridge_interruptCtrl_readInt;
  wire                coreArea_uartBridge_interruptCtrl_writeInt;
  wire                coreArea_uartBridge_interruptCtrl_interrupt;
  reg                 coreArea_uartBridge_misc_readError;
  reg                 _zz_10;
  reg                 coreArea_uartBridge_misc_readOverflowError;
  reg                 _zz_11;
  reg                 coreArea_uartBridge_misc_breakDetected;
  reg                 coreArea_uartCtrl_io_readBreak_regNext;
  reg                 _zz_12;
  reg                 coreArea_uartBridge_misc_doBreak;
  reg                 _zz_13;
  reg                 _zz_14;
  reg                 coreArea_uartTimeOut_state;
  reg                 coreArea_uartTimeOut_stateRise;
  wire                coreArea_uartTimeOut_counter_willIncrement;
  reg                 coreArea_uartTimeOut_counter_willClear;
  reg        [19:0]   coreArea_uartTimeOut_counter_valueNext;
  reg        [19:0]   coreArea_uartTimeOut_counter_value;
  wire                coreArea_uartTimeOut_counter_willOverflowIfInc;
  wire                coreArea_uartTimeOut_counter_willOverflow;
  reg                 _zz_15;
  reg                 _zz_16;
  reg                 _zz_17;
  reg                 _zz_18;
  reg                 _zz_19;
  `ifndef SYNTHESIS
  reg [23:0] coreArea_uartBridge_uartConfigReg_frame_stop_string;
  reg [31:0] coreArea_uartBridge_uartConfigReg_frame_parity_string;
  `endif

  function [19:0] zz_coreArea_uartBridge_uartConfigReg_clockDivider(input dummy);
    begin
      zz_coreArea_uartBridge_uartConfigReg_clockDivider = 20'h0;
      zz_coreArea_uartBridge_uartConfigReg_clockDivider = 20'h00012;
    end
  endfunction
  wire [19:0] _zz_50;

  assign _zz_34 = (! (tx && rx));
  assign _zz_35 = coreArea_peripheralBus_writeData[0 : 0];
  assign _zz_36 = 1'b0;
  assign _zz_37 = coreArea_peripheralBus_writeData[1 : 1];
  assign _zz_38 = 1'b0;
  assign _zz_39 = coreArea_peripheralBus_writeData[9 : 9];
  assign _zz_40 = 1'b0;
  assign _zz_41 = coreArea_peripheralBus_writeData[10 : 10];
  assign _zz_42 = 1'b1;
  assign _zz_43 = coreArea_peripheralBus_writeData[11 : 11];
  assign _zz_44 = 1'b0;
  assign _zz_45 = coreArea_uartTimeOut_counter_willIncrement;
  assign _zz_46 = {19'd0, _zz_45};
  assign _zz_47 = (4'b1000 - coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy);
  assign _zz_48 = coreArea_peripheralBus_writeData[0 : 0];
  assign _zz_49 = coreArea_peripheralBus_writeData[1 : 1];
  J1Jtag jtagIface_jtagArea_jtag (
    .tdi                                       (tdi                                                             ), //i
    .tdo                                       (jtagIface_jtagArea_jtag_tdo                                     ), //o
    .tms                                       (tms                                                             ), //i
    .jtagDataFlow_valid                        (jtagIface_jtagArea_jtag_jtagDataFlow_valid                      ), //o
    .jtagDataFlow_payload_jtagDataValid        (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid      ), //o
    .jtagDataFlow_payload_jtagStall            (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall          ), //o
    .jtagDataFlow_payload_jtagCaptureMemory    (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory  ), //o
    .jtagDataFlow_payload_jtagCPUAdr           (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr[7:0]    ), //o
    .jtagDataFlow_payload_jtagCPUWord          (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord[15:0]  ), //o
    .jtagReset                                 (jtagIface_jtagArea_jtag_jtagReset                               ), //o
    .tck                                       (tck                                                             ), //i
    .reset                                     (reset                                                           )  //i
  );
  BufferCC_3 bufferCC_4 (
    .io_dataIn     (_zz_20                 ), //i
    .io_dataOut    (bufferCC_4_io_dataOut  ), //o
    .core_clk      (core_clk               ), //i
    ._zz_1         (_zz_1                  )  //i
  );
  J1 coreArea_cpu (
    .stall               (flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall          ), //i
    .irq                 (coreArea_intCtrl_irq                                              ), //i
    .intVec              (_zz_21[7:0]                                                       ), //i
    .captureMemory       (flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory  ), //i
    .jtagMemAdr          (flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr[7:0]    ), //i
    .jtagMemWord         (flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord[15:0]  ), //i
    .cpuBus_enable       (coreArea_cpu_cpuBus_enable                                        ), //o
    .cpuBus_writeMode    (coreArea_cpu_cpuBus_writeMode                                     ), //o
    .cpuBus_address      (coreArea_cpu_cpuBus_address[15:0]                                 ), //o
    .cpuBus_writeData    (coreArea_cpu_cpuBus_writeData[15:0]                               ), //o
    .cpuBus_readData     (coreArea_peripheralBus_readData[15:0]                             ), //i
    .core_reset          (core_reset                                                        ), //i
    .core_clk            (core_clk                                                          )  //i
  );
  FlowCCByToggle flowCCByToggle_1 (
    .io_input_valid                         (jtagIface_jtagArea_jtag_jtagDataFlow_valid                      ), //i
    .io_input_payload_jtagDataValid         (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid      ), //i
    .io_input_payload_jtagStall             (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall          ), //i
    .io_input_payload_jtagCaptureMemory     (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory  ), //i
    .io_input_payload_jtagCPUAdr            (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr[7:0]    ), //i
    .io_input_payload_jtagCPUWord           (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord[15:0]  ), //i
    .io_output_valid                        (flowCCByToggle_1_io_output_valid                                ), //o
    .io_output_payload_jtagDataValid        (flowCCByToggle_1_io_output_payload_jtagDataValid                ), //o
    .io_output_payload_jtagStall            (flowCCByToggle_1_io_output_payload_jtagStall                    ), //o
    .io_output_payload_jtagCaptureMemory    (flowCCByToggle_1_io_output_payload_jtagCaptureMemory            ), //o
    .io_output_payload_jtagCPUAdr           (flowCCByToggle_1_io_output_payload_jtagCPUAdr[7:0]              ), //o
    .io_output_payload_jtagCPUWord          (flowCCByToggle_1_io_output_payload_jtagCPUWord[15:0]            ), //o
    .tck                                    (tck                                                             ), //i
    .reset                                  (reset                                                           ), //i
    .core_clk                               (core_clk                                                        ), //i
    .core_reset                             (core_reset                                                      )  //i
  );
  Timer coreArea_timerA (
    .loadHigh             (_zz_4                              ), //i
    .loadLow              (_zz_3                              ), //i
    .cmpHigh              (_zz_22[15:0]                       ), //i
    .cmpLow               (_zz_23[15:0]                       ), //i
    .enable               (_zz_24[15:0]                       ), //i
    .accessEnableWrite    (_zz_5                              ), //i
    .enableState          (coreArea_timerA_enableState[15:0]  ), //o
    .highState            (coreArea_timerA_highState[15:0]    ), //o
    .lowState             (coreArea_timerA_lowState[15:0]     ), //o
    .interrupt            (coreArea_timerA_interrupt          ), //o
    .core_clk             (core_clk                           ), //i
    .core_reset           (core_reset                         )  //i
  );
  Timer coreArea_timerB (
    .loadHigh             (_zz_7                              ), //i
    .loadLow              (_zz_6                              ), //i
    .cmpHigh              (_zz_25[15:0]                       ), //i
    .cmpLow               (_zz_26[15:0]                       ), //i
    .enable               (_zz_27[15:0]                       ), //i
    .accessEnableWrite    (_zz_8                              ), //i
    .enableState          (coreArea_timerB_enableState[15:0]  ), //o
    .highState            (coreArea_timerB_highState[15:0]    ), //o
    .lowState             (coreArea_timerB_lowState[15:0]     ), //o
    .interrupt            (coreArea_timerB_interrupt          ), //o
    .core_clk             (core_clk                           ), //i
    .core_reset           (core_reset                         )  //i
  );
  UartCtrl coreArea_uartCtrl (
    .io_config_frame_dataLength    (coreArea_uartBridge_uartConfigReg_frame_dataLength[2:0]                            ), //i
    .io_config_frame_stop          (coreArea_uartBridge_uartConfigReg_frame_stop                                       ), //i
    .io_config_frame_parity        (coreArea_uartBridge_uartConfigReg_frame_parity[1:0]                                ), //i
    .io_config_clockDivider        (coreArea_uartBridge_uartConfigReg_clockDivider[19:0]                               ), //i
    .io_write_valid                (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid         ), //i
    .io_write_ready                (coreArea_uartCtrl_io_write_ready                                                   ), //o
    .io_write_payload              (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0]  ), //i
    .io_read_valid                 (coreArea_uartCtrl_io_read_valid                                                    ), //o
    .io_read_ready                 (coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready                         ), //i
    .io_read_payload               (coreArea_uartCtrl_io_read_payload[7:0]                                             ), //o
    .io_uart_txd                   (coreArea_uartCtrl_io_uart_txd                                                      ), //o
    .io_uart_rxd                   (rx                                                                                 ), //i
    .io_readError                  (coreArea_uartCtrl_io_readError                                                     ), //o
    .io_writeBreak                 (coreArea_uartBridge_misc_doBreak                                                   ), //i
    .io_readBreak                  (coreArea_uartCtrl_io_readBreak                                                     ), //o
    .core_clk                      (core_clk                                                                           ), //i
    .core_reset                    (core_reset                                                                         )  //i
  );
  StreamFifo coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy (
    .io_push_valid      (coreArea_uartBridge_write_streamUnbuffered_valid                                    ), //i
    .io_push_ready      (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready         ), //o
    .io_push_payload    (coreArea_uartBridge_write_streamUnbuffered_payload[7:0]                             ), //i
    .io_pop_valid       (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid          ), //o
    .io_pop_ready       (coreArea_uartCtrl_io_write_ready                                                    ), //i
    .io_pop_payload     (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0]   ), //o
    .io_flush           (_zz_28                                                                              ), //i
    .io_occupancy       (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy[3:0]     ), //o
    .io_availability    (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_availability[3:0]  ), //o
    .core_clk           (core_clk                                                                            ), //i
    .core_reset         (core_reset                                                                          )  //i
  );
  StreamFifo coreArea_uartCtrl_io_read_queueWithOccupancy (
    .io_push_valid      (coreArea_uartCtrl_io_read_valid                                    ), //i
    .io_push_ready      (coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready         ), //o
    .io_push_payload    (coreArea_uartCtrl_io_read_payload[7:0]                             ), //i
    .io_pop_valid       (coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid          ), //o
    .io_pop_ready       (_zz_29                                                             ), //i
    .io_pop_payload     (coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload[7:0]   ), //o
    .io_flush           (_zz_30                                                             ), //i
    .io_occupancy       (coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy[3:0]     ), //o
    .io_availability    (coreArea_uartCtrl_io_read_queueWithOccupancy_io_availability[3:0]  ), //o
    .core_clk           (core_clk                                                           ), //i
    .core_reset         (core_reset                                                         )  //i
  );
  InterruptCtrl coreArea_intCtrl (
    .irqReqs               (_zz_31[3:0]                          ), //i
    .enableWriteNewMask    (_zz_15                               ), //i
    .enableWriteIrqVec     (_zz_32[3:0]                          ), //i
    .irqSetData            (_zz_33[15:0]                         ), //i
    .irqGetMask            (coreArea_intCtrl_irqGetMask[15:0]    ), //o
    .irqVectors_0          (coreArea_intCtrl_irqVectors_0[15:0]  ), //o
    .irqVectors_1          (coreArea_intCtrl_irqVectors_1[15:0]  ), //o
    .irqVectors_2          (coreArea_intCtrl_irqVectors_2[15:0]  ), //o
    .irqVectors_3          (coreArea_intCtrl_irqVectors_3[15:0]  ), //o
    .intVec                (coreArea_intCtrl_intVec[15:0]        ), //o
    .irq                   (coreArea_intCtrl_irq                 ), //o
    .core_clk              (core_clk                             ), //i
    .core_reset            (core_reset                           )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(coreArea_uartBridge_uartConfigReg_frame_stop)
      `UartStopType_defaultEncoding_ONE : coreArea_uartBridge_uartConfigReg_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : coreArea_uartBridge_uartConfigReg_frame_stop_string = "TWO";
      default : coreArea_uartBridge_uartConfigReg_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(coreArea_uartBridge_uartConfigReg_frame_parity)
      `UartParityType_defaultEncoding_NONE : coreArea_uartBridge_uartConfigReg_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : coreArea_uartBridge_uartConfigReg_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : coreArea_uartBridge_uartConfigReg_frame_parity_string = "ODD ";
      default : coreArea_uartBridge_uartConfigReg_frame_parity_string = "????";
    endcase
  end
  `endif

  assign tdo = jtagIface_jtagArea_jtag_tdo;
  assign core_clk = boardClk;
  assign _zz_1 = ((reset || jtagIface_jtagArea_jtag_jtagReset) || (! boardClkLocked));
  assign _zz_20 = 1'b0;
  assign core_reset = _zz_2;
  assign coreArea_peripheralBus_address = coreArea_cpu_cpuBus_address_delay_1;
  assign coreArea_peripheralBus_enable = coreArea_cpu_cpuBus_enable_delay_1;
  assign coreArea_peripheralBus_writeMode = coreArea_cpu_cpuBus_writeMode_delay_1;
  assign coreArea_peripheralBus_writeData = coreArea_cpu_cpuBus_writeData_delay_1;
  assign coreArea_peripheralBusCtrl_askWrite = (coreArea_peripheralBus_enable && coreArea_peripheralBus_writeMode);
  assign coreArea_peripheralBusCtrl_askRead = (coreArea_peripheralBus_enable && (! coreArea_peripheralBus_writeMode));
  always @ (*) begin
    _zz_3 = 1'b0;
    _zz_4 = 1'b0;
    _zz_5 = 1'b0;
    _zz_6 = 1'b0;
    _zz_7 = 1'b0;
    _zz_8 = 1'b0;
    _zz_9 = 1'b0;
    coreArea_uartBridge_read_streamBreaked_ready = 1'b0;
    _zz_10 = 1'b0;
    _zz_11 = 1'b0;
    _zz_12 = 1'b0;
    _zz_13 = 1'b0;
    _zz_14 = 1'b0;
    _zz_15 = 1'b0;
    _zz_16 = 1'b0;
    _zz_17 = 1'b0;
    _zz_18 = 1'b0;
    _zz_19 = 1'b0;
    coreArea_peripheralBus_readData = 16'h0;
    case(coreArea_peripheralBus_address)
      16'h00c0 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_3 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_lowState;
      end
      16'h00c1 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_4 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_highState;
      end
      16'h00c2 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_5 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_enableState;
      end
      16'h00d0 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_6 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_lowState;
      end
      16'h00d1 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_7 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_highState;
      end
      16'h00d2 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_8 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_enableState;
      end
      16'h00f0 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_9 = 1'b1;
        end
        if(coreArea_peripheralBusCtrl_askRead)begin
          coreArea_uartBridge_read_streamBreaked_ready = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 15] = (coreArea_uartBridge_read_streamBreaked_valid ^ 1'b0);
        coreArea_peripheralBus_readData[7 : 0] = coreArea_uartBridge_read_streamBreaked_payload;
      end
      16'h00f6 : begin
        coreArea_peripheralBus_readData[3 : 0] = _zz_47;
        coreArea_peripheralBus_readData[11 : 8] = coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy;
      end
      16'h00f4 : begin
        coreArea_peripheralBus_readData[15 : 15] = coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
        coreArea_peripheralBus_readData[0 : 0] = coreArea_uartBridge_interruptCtrl_writeIntEnable;
        coreArea_peripheralBus_readData[1 : 1] = coreArea_uartBridge_interruptCtrl_readIntEnable;
        coreArea_peripheralBus_readData[8 : 8] = coreArea_uartBridge_interruptCtrl_writeInt;
        coreArea_peripheralBus_readData[9 : 9] = coreArea_uartBridge_interruptCtrl_readInt;
      end
      16'h0100 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_10 = 1'b1;
          _zz_11 = 1'b1;
          _zz_12 = 1'b1;
          _zz_13 = 1'b1;
          _zz_14 = 1'b1;
        end
        coreArea_peripheralBus_readData[0 : 0] = coreArea_uartBridge_misc_readError;
        coreArea_peripheralBus_readData[1 : 1] = coreArea_uartBridge_misc_readOverflowError;
        coreArea_peripheralBus_readData[8 : 8] = coreArea_uartCtrl_io_readBreak;
        coreArea_peripheralBus_readData[9 : 9] = coreArea_uartBridge_misc_breakDetected;
      end
      16'h00e4 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_15 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqGetMask;
      end
      16'h00e0 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_16 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_0;
      end
      16'h00e1 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_17 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_1;
      end
      16'h00e2 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_18 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_2;
      end
      16'h00e3 : begin
        if(coreArea_peripheralBusCtrl_askWrite)begin
          _zz_19 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_3;
      end
      default : begin
      end
    endcase
  end

  assign _zz_50 = zz_coreArea_uartBridge_uartConfigReg_clockDivider(1'b0);
  always @ (*) coreArea_uartBridge_uartConfigReg_clockDivider = _zz_50;
  assign coreArea_uartBridge_uartConfigReg_frame_dataLength = 3'b111;
  assign coreArea_uartBridge_uartConfigReg_frame_parity = `UartParityType_defaultEncoding_NONE;
  assign coreArea_uartBridge_uartConfigReg_frame_stop = `UartStopType_defaultEncoding_ONE;
  assign coreArea_uartBridge_write_streamUnbuffered_valid = _zz_9;
  assign coreArea_uartBridge_write_streamUnbuffered_payload = coreArea_peripheralBus_writeData[7 : 0];
  assign coreArea_uartBridge_write_streamUnbuffered_ready = coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  always @ (*) begin
    coreArea_uartBridge_read_streamBreaked_valid = coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid;
    _zz_29 = coreArea_uartBridge_read_streamBreaked_ready;
    if(coreArea_uartCtrl_io_readBreak)begin
      coreArea_uartBridge_read_streamBreaked_valid = 1'b0;
      _zz_29 = 1'b1;
    end
  end

  assign coreArea_uartBridge_read_streamBreaked_payload = coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload;
  assign coreArea_uartBridge_interruptCtrl_readInt = (coreArea_uartBridge_interruptCtrl_readIntEnable && coreArea_uartBridge_read_streamBreaked_valid);
  assign coreArea_uartBridge_interruptCtrl_writeInt = (coreArea_uartBridge_interruptCtrl_writeIntEnable && (! coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid));
  assign coreArea_uartBridge_interruptCtrl_interrupt = (coreArea_uartBridge_interruptCtrl_readInt || coreArea_uartBridge_interruptCtrl_writeInt);
  assign tx = coreArea_uartCtrl_io_uart_txd;
  always @ (*) begin
    coreArea_uartTimeOut_stateRise = 1'b0;
    coreArea_uartTimeOut_counter_willClear = 1'b0;
    if(coreArea_uartTimeOut_counter_willOverflow)begin
      coreArea_uartTimeOut_stateRise = (! coreArea_uartTimeOut_state);
    end
    if(_zz_34)begin
      coreArea_uartTimeOut_counter_willClear = 1'b1;
      coreArea_uartTimeOut_stateRise = 1'b0;
    end
  end

  assign coreArea_uartTimeOut_counter_willOverflowIfInc = (coreArea_uartTimeOut_counter_value == 20'hdbb9f);
  assign coreArea_uartTimeOut_counter_willOverflow = (coreArea_uartTimeOut_counter_willOverflowIfInc && coreArea_uartTimeOut_counter_willIncrement);
  always @ (*) begin
    if(coreArea_uartTimeOut_counter_willOverflow)begin
      coreArea_uartTimeOut_counter_valueNext = 20'h0;
    end else begin
      coreArea_uartTimeOut_counter_valueNext = (coreArea_uartTimeOut_counter_value + _zz_46);
    end
    if(coreArea_uartTimeOut_counter_willClear)begin
      coreArea_uartTimeOut_counter_valueNext = 20'h0;
    end
  end

  assign coreArea_uartTimeOut_counter_willIncrement = 1'b1;
  assign uartLed = coreArea_uartTimeOut_state;
  always @ (*) begin
    _zz_32[0] = _zz_16;
    _zz_32[1] = _zz_17;
    _zz_32[2] = _zz_18;
    _zz_32[3] = _zz_19;
  end

  always @ (*) begin
    _zz_31[3 : 3] = extInt;
    _zz_31[0] = coreArea_uartBridge_interruptCtrl_readInt;
    _zz_31[1] = coreArea_timerA_interrupt;
    _zz_31[2] = coreArea_timerB_interrupt;
  end

  assign _zz_21 = coreArea_intCtrl_intVec[7:0];
  assign _zz_23 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_22 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_24 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_26 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_25 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_27 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_33 = coreArea_peripheralBus_writeData[15 : 0];
  assign _zz_28 = 1'b0;
  assign _zz_30 = 1'b0;
  always @ (posedge core_clk) begin
    _zz_2 <= bufferCC_4_io_dataOut;
    if(flowCCByToggle_1_io_output_payload_jtagDataValid)begin
      flowCCByToggle_1_io_output_payload_regNextWhen_jtagDataValid <= flowCCByToggle_1_io_output_payload_jtagDataValid;
      flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall <= flowCCByToggle_1_io_output_payload_jtagStall;
      flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory <= flowCCByToggle_1_io_output_payload_jtagCaptureMemory;
      flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr <= flowCCByToggle_1_io_output_payload_jtagCPUAdr;
      flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord <= flowCCByToggle_1_io_output_payload_jtagCPUWord;
    end
    coreArea_cpu_cpuBus_address_delay_1 <= coreArea_cpu_cpuBus_address;
    coreArea_cpu_cpuBus_enable_delay_1 <= coreArea_cpu_cpuBus_enable;
    coreArea_cpu_cpuBus_writeMode_delay_1 <= coreArea_cpu_cpuBus_writeMode;
    coreArea_cpu_cpuBus_writeData_delay_1 <= coreArea_cpu_cpuBus_writeData;
    coreArea_uartCtrl_io_readBreak_regNext <= coreArea_uartCtrl_io_readBreak;
  end

  always @ (posedge core_clk) begin
    if(core_reset) begin
      coreArea_uartBridge_interruptCtrl_writeIntEnable <= 1'b0;
      coreArea_uartBridge_interruptCtrl_readIntEnable <= 1'b0;
      coreArea_uartBridge_misc_readError <= 1'b0;
      coreArea_uartBridge_misc_readOverflowError <= 1'b0;
      coreArea_uartBridge_misc_breakDetected <= 1'b0;
      coreArea_uartBridge_misc_doBreak <= 1'b0;
      coreArea_uartTimeOut_state <= 1'b0;
      coreArea_uartTimeOut_counter_value <= 20'h0;
    end else begin
      if(_zz_10)begin
        if(_zz_35[0])begin
          coreArea_uartBridge_misc_readError <= _zz_36[0];
        end
      end
      if(coreArea_uartCtrl_io_readError)begin
        coreArea_uartBridge_misc_readError <= 1'b1;
      end
      if(_zz_11)begin
        if(_zz_37[0])begin
          coreArea_uartBridge_misc_readOverflowError <= _zz_38[0];
        end
      end
      if((coreArea_uartCtrl_io_read_valid && (! coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready)))begin
        coreArea_uartBridge_misc_readOverflowError <= 1'b1;
      end
      if((coreArea_uartCtrl_io_readBreak && (! coreArea_uartCtrl_io_readBreak_regNext)))begin
        coreArea_uartBridge_misc_breakDetected <= 1'b1;
      end
      if(_zz_12)begin
        if(_zz_39[0])begin
          coreArea_uartBridge_misc_breakDetected <= _zz_40[0];
        end
      end
      if(_zz_13)begin
        if(_zz_41[0])begin
          coreArea_uartBridge_misc_doBreak <= _zz_42[0];
        end
      end
      if(_zz_14)begin
        if(_zz_43[0])begin
          coreArea_uartBridge_misc_doBreak <= _zz_44[0];
        end
      end
      coreArea_uartBridge_interruptCtrl_readIntEnable <= 1'b1;
      coreArea_uartTimeOut_counter_value <= coreArea_uartTimeOut_counter_valueNext;
      if(coreArea_uartTimeOut_counter_willOverflow)begin
        coreArea_uartTimeOut_state <= 1'b1;
      end
      if(_zz_34)begin
        coreArea_uartTimeOut_state <= 1'b0;
      end
      case(coreArea_peripheralBus_address)
        16'h00f4 : begin
          if(coreArea_peripheralBusCtrl_askWrite)begin
            coreArea_uartBridge_interruptCtrl_writeIntEnable <= _zz_48[0];
            coreArea_uartBridge_interruptCtrl_readIntEnable <= _zz_49[0];
          end
        end
        default : begin
        end
      endcase
    end
  end


endmodule

module InterruptCtrl (
  input      [3:0]    irqReqs,
  input               enableWriteNewMask,
  input      [3:0]    enableWriteIrqVec,
  input      [15:0]   irqSetData,
  output     [15:0]   irqGetMask,
  output     [15:0]   irqVectors_0,
  output     [15:0]   irqVectors_1,
  output     [15:0]   irqVectors_2,
  output     [15:0]   irqVectors_3,
  output     [15:0]   intVec,
  output              irq,
  input               core_clk,
  input               core_reset
);
  reg        [7:0]    _zz_7;
  wire       [3:0]    irqReqs_buffercc_io_dataOut;
  wire       [14:0]   _zz_8;
  wire       [14:0]   _zz_9;
  wire       [14:0]   _zz_10;
  wire       [14:0]   _zz_11;
  wire       [3:0]    _zz_12;
  reg        [3:0]    irqMask;
  wire       [3:0]    irqVecWriteEnable;
  reg        [7:0]    irqVectors_0_1;
  reg        [7:0]    irqVectors_1_1;
  reg        [7:0]    irqVectors_2_1;
  reg        [7:0]    irqVectors_3_1;
  wire       [3:0]    irqSync;
  wire       [3:0]    _zz_1;
  wire       [3:0]    _zz_2;
  wire                _zz_3;
  wire                _zz_4;
  wire                _zz_5;
  wire       [1:0]    intNo;
  wire                _zz_6;
  reg                 _zz_6_regNext;

  assign _zz_8 = (irqSetData >>> 1);
  assign _zz_9 = (irqSetData >>> 1);
  assign _zz_10 = (irqSetData >>> 1);
  assign _zz_11 = (irqSetData >>> 1);
  assign _zz_12 = (_zz_1 - 4'b0001);
  BufferCC_2 irqReqs_buffercc (
    .io_dataIn     (irqReqs[3:0]                      ), //i
    .io_dataOut    (irqReqs_buffercc_io_dataOut[3:0]  ), //o
    .core_clk      (core_clk                          ), //i
    .core_reset    (core_reset                        )  //i
  );
  always @(*) begin
    case(intNo)
      2'b00 : begin
        _zz_7 = irqVectors_0_1;
      end
      2'b01 : begin
        _zz_7 = irqVectors_1_1;
      end
      2'b10 : begin
        _zz_7 = irqVectors_2_1;
      end
      default : begin
        _zz_7 = irqVectors_3_1;
      end
    endcase
  end

  assign irqGetMask = {12'd0, irqMask};
  assign irqVecWriteEnable = enableWriteIrqVec;
  assign irqVectors_0 = {8'd0, irqVectors_0_1};
  assign irqVectors_1 = {8'd0, irqVectors_1_1};
  assign irqVectors_2 = {8'd0, irqVectors_2_1};
  assign irqVectors_3 = {8'd0, irqVectors_3_1};
  assign irqSync = irqReqs_buffercc_io_dataOut;
  assign _zz_1 = irqSync;
  assign _zz_2 = (_zz_1 & (~ _zz_12));
  assign _zz_3 = _zz_2[3];
  assign _zz_4 = (_zz_2[1] || _zz_3);
  assign _zz_5 = (_zz_2[2] || _zz_3);
  assign intNo = {_zz_5,_zz_4};
  assign intVec = {8'd0, _zz_7};
  assign _zz_6 = ((irqSync & irqMask) != 4'b0000);
  assign irq = (_zz_6 && (! _zz_6_regNext));
  always @ (posedge core_clk) begin
    if(core_reset) begin
      irqMask <= (1'b0 ? 4'b1111 : 4'b0000);
      irqVectors_0_1 <= (1'b0 ? 8'hff : 8'h0);
      irqVectors_1_1 <= (1'b0 ? 8'hff : 8'h0);
      irqVectors_2_1 <= (1'b0 ? 8'hff : 8'h0);
      irqVectors_3_1 <= (1'b0 ? 8'hff : 8'h0);
      _zz_6_regNext <= 1'b0;
    end else begin
      if(enableWriteNewMask)begin
        irqMask <= irqSetData[3:0];
      end
      if(irqVecWriteEnable[0])begin
        irqVectors_0_1 <= _zz_8[7:0];
      end
      if(irqVecWriteEnable[1])begin
        irqVectors_1_1 <= _zz_9[7:0];
      end
      if(irqVecWriteEnable[2])begin
        irqVectors_2_1 <= _zz_10[7:0];
      end
      if(irqVecWriteEnable[3])begin
        irqVectors_3_1 <= _zz_11[7:0];
      end
      _zz_6_regNext <= _zz_6;
    end
  end


endmodule

//StreamFifo replaced by StreamFifo

module StreamFifo (
  input               io_push_valid,
  output              io_push_ready,
  input      [7:0]    io_push_payload,
  output              io_pop_valid,
  input               io_pop_ready,
  output     [7:0]    io_pop_payload,
  input               io_flush,
  output     [3:0]    io_occupancy,
  output     [3:0]    io_availability,
  input               core_clk,
  input               core_reset
);
  reg        [7:0]    _zz_3;
  wire       [0:0]    _zz_4;
  wire       [2:0]    _zz_5;
  wire       [0:0]    _zz_6;
  wire       [2:0]    _zz_7;
  wire       [2:0]    _zz_8;
  wire                _zz_9;
  reg                 _zz_1;
  reg                 logic_pushPtr_willIncrement;
  reg                 logic_pushPtr_willClear;
  reg        [2:0]    logic_pushPtr_valueNext;
  reg        [2:0]    logic_pushPtr_value;
  wire                logic_pushPtr_willOverflowIfInc;
  wire                logic_pushPtr_willOverflow;
  reg                 logic_popPtr_willIncrement;
  reg                 logic_popPtr_willClear;
  reg        [2:0]    logic_popPtr_valueNext;
  reg        [2:0]    logic_popPtr_value;
  wire                logic_popPtr_willOverflowIfInc;
  wire                logic_popPtr_willOverflow;
  wire                logic_ptrMatch;
  reg                 logic_risingOccupancy;
  wire                logic_pushing;
  wire                logic_popping;
  wire                logic_empty;
  wire                logic_full;
  reg                 _zz_2;
  wire       [2:0]    logic_ptrDif;
  reg [7:0] logic_ram [0:7];

  assign _zz_4 = logic_pushPtr_willIncrement;
  assign _zz_5 = {2'd0, _zz_4};
  assign _zz_6 = logic_popPtr_willIncrement;
  assign _zz_7 = {2'd0, _zz_6};
  assign _zz_8 = (logic_popPtr_value - logic_pushPtr_value);
  assign _zz_9 = 1'b1;
  always @ (posedge core_clk) begin
    if(_zz_9) begin
      _zz_3 <= logic_ram[logic_popPtr_valueNext];
    end
  end

  always @ (posedge core_clk) begin
    if(_zz_1) begin
      logic_ram[logic_pushPtr_value] <= io_push_payload;
    end
  end

  always @ (*) begin
    _zz_1 = 1'b0;
    logic_pushPtr_willIncrement = 1'b0;
    if(logic_pushing)begin
      _zz_1 = 1'b1;
      logic_pushPtr_willIncrement = 1'b1;
    end
  end

  always @ (*) begin
    logic_pushPtr_willClear = 1'b0;
    logic_popPtr_willClear = 1'b0;
    if(io_flush)begin
      logic_pushPtr_willClear = 1'b1;
      logic_popPtr_willClear = 1'b1;
    end
  end

  assign logic_pushPtr_willOverflowIfInc = (logic_pushPtr_value == 3'b111);
  assign logic_pushPtr_willOverflow = (logic_pushPtr_willOverflowIfInc && logic_pushPtr_willIncrement);
  always @ (*) begin
    logic_pushPtr_valueNext = (logic_pushPtr_value + _zz_5);
    if(logic_pushPtr_willClear)begin
      logic_pushPtr_valueNext = 3'b000;
    end
  end

  always @ (*) begin
    logic_popPtr_willIncrement = 1'b0;
    if(logic_popping)begin
      logic_popPtr_willIncrement = 1'b1;
    end
  end

  assign logic_popPtr_willOverflowIfInc = (logic_popPtr_value == 3'b111);
  assign logic_popPtr_willOverflow = (logic_popPtr_willOverflowIfInc && logic_popPtr_willIncrement);
  always @ (*) begin
    logic_popPtr_valueNext = (logic_popPtr_value + _zz_7);
    if(logic_popPtr_willClear)begin
      logic_popPtr_valueNext = 3'b000;
    end
  end

  assign logic_ptrMatch = (logic_pushPtr_value == logic_popPtr_value);
  assign logic_pushing = (io_push_valid && io_push_ready);
  assign logic_popping = (io_pop_valid && io_pop_ready);
  assign logic_empty = (logic_ptrMatch && (! logic_risingOccupancy));
  assign logic_full = (logic_ptrMatch && logic_risingOccupancy);
  assign io_push_ready = (! logic_full);
  assign io_pop_valid = ((! logic_empty) && (! (_zz_2 && (! logic_full))));
  assign io_pop_payload = _zz_3;
  assign logic_ptrDif = (logic_pushPtr_value - logic_popPtr_value);
  assign io_occupancy = {(logic_risingOccupancy && logic_ptrMatch),logic_ptrDif};
  assign io_availability = {((! logic_risingOccupancy) && logic_ptrMatch),_zz_8};
  always @ (posedge core_clk) begin
    if(core_reset) begin
      logic_pushPtr_value <= 3'b000;
      logic_popPtr_value <= 3'b000;
      logic_risingOccupancy <= 1'b0;
      _zz_2 <= 1'b0;
    end else begin
      logic_pushPtr_value <= logic_pushPtr_valueNext;
      logic_popPtr_value <= logic_popPtr_valueNext;
      _zz_2 <= (logic_popPtr_valueNext == logic_pushPtr_value);
      if((logic_pushing != logic_popping))begin
        logic_risingOccupancy <= logic_pushing;
      end
      if(io_flush)begin
        logic_risingOccupancy <= 1'b0;
      end
    end
  end


endmodule

module UartCtrl (
  input      [2:0]    io_config_frame_dataLength,
  input      `UartStopType_defaultEncoding_type io_config_frame_stop,
  input      `UartParityType_defaultEncoding_type io_config_frame_parity,
  input      [19:0]   io_config_clockDivider,
  input               io_write_valid,
  output reg          io_write_ready,
  input      [7:0]    io_write_payload,
  output              io_read_valid,
  input               io_read_ready,
  output     [7:0]    io_read_payload,
  output              io_uart_txd,
  input               io_uart_rxd,
  output              io_readError,
  input               io_writeBreak,
  output              io_readBreak,
  input               core_clk,
  input               core_reset
);
  wire                _zz_1;
  wire                tx_io_write_ready;
  wire                tx_io_txd;
  wire                rx_io_read_valid;
  wire       [7:0]    rx_io_read_payload;
  wire                rx_io_rts;
  wire                rx_io_error;
  wire                rx_io_break;
  reg        [19:0]   clockDivider_counter;
  wire                clockDivider_tick;
  reg                 io_write_thrown_valid;
  wire                io_write_thrown_ready;
  wire       [7:0]    io_write_thrown_payload;
  `ifndef SYNTHESIS
  reg [23:0] io_config_frame_stop_string;
  reg [31:0] io_config_frame_parity_string;
  `endif


  UartCtrlTx tx (
    .io_configFrame_dataLength    (io_config_frame_dataLength[2:0]  ), //i
    .io_configFrame_stop          (io_config_frame_stop             ), //i
    .io_configFrame_parity        (io_config_frame_parity[1:0]      ), //i
    .io_samplingTick              (clockDivider_tick                ), //i
    .io_write_valid               (io_write_thrown_valid            ), //i
    .io_write_ready               (tx_io_write_ready                ), //o
    .io_write_payload             (io_write_thrown_payload[7:0]     ), //i
    .io_cts                       (_zz_1                            ), //i
    .io_txd                       (tx_io_txd                        ), //o
    .io_break                     (io_writeBreak                    ), //i
    .core_clk                     (core_clk                         ), //i
    .core_reset                   (core_reset                       )  //i
  );
  UartCtrlRx rx (
    .io_configFrame_dataLength    (io_config_frame_dataLength[2:0]  ), //i
    .io_configFrame_stop          (io_config_frame_stop             ), //i
    .io_configFrame_parity        (io_config_frame_parity[1:0]      ), //i
    .io_samplingTick              (clockDivider_tick                ), //i
    .io_read_valid                (rx_io_read_valid                 ), //o
    .io_read_ready                (io_read_ready                    ), //i
    .io_read_payload              (rx_io_read_payload[7:0]          ), //o
    .io_rxd                       (io_uart_rxd                      ), //i
    .io_rts                       (rx_io_rts                        ), //o
    .io_error                     (rx_io_error                      ), //o
    .io_break                     (rx_io_break                      ), //o
    .core_clk                     (core_clk                         ), //i
    .core_reset                   (core_reset                       )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_config_frame_stop)
      `UartStopType_defaultEncoding_ONE : io_config_frame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_config_frame_stop_string = "TWO";
      default : io_config_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_config_frame_parity)
      `UartParityType_defaultEncoding_NONE : io_config_frame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_config_frame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_config_frame_parity_string = "ODD ";
      default : io_config_frame_parity_string = "????";
    endcase
  end
  `endif

  assign clockDivider_tick = (clockDivider_counter == 20'h0);
  always @ (*) begin
    io_write_thrown_valid = io_write_valid;
    io_write_ready = io_write_thrown_ready;
    if(rx_io_break)begin
      io_write_thrown_valid = 1'b0;
      io_write_ready = 1'b1;
    end
  end

  assign io_write_thrown_payload = io_write_payload;
  assign io_write_thrown_ready = tx_io_write_ready;
  assign io_read_valid = rx_io_read_valid;
  assign io_read_payload = rx_io_read_payload;
  assign io_uart_txd = tx_io_txd;
  assign io_readError = rx_io_error;
  assign _zz_1 = 1'b0;
  assign io_readBreak = rx_io_break;
  always @ (posedge core_clk) begin
    if(core_reset) begin
      clockDivider_counter <= 20'h0;
    end else begin
      clockDivider_counter <= (clockDivider_counter - 20'h00001);
      if(clockDivider_tick)begin
        clockDivider_counter <= io_config_clockDivider;
      end
    end
  end


endmodule

//Timer replaced by Timer

module Timer (
  input               loadHigh,
  input               loadLow,
  input      [15:0]   cmpHigh,
  input      [15:0]   cmpLow,
  input      [15:0]   enable,
  input               accessEnableWrite,
  output     [15:0]   enableState,
  output     [15:0]   highState,
  output     [15:0]   lowState,
  output              interrupt,
  input               core_clk,
  input               core_reset
);
  reg        [31:0]   cnt;
  reg        [31:0]   cmp;
  reg                 isEnabled;
  wire       [31:0]   maxCnt;

  assign highState = cmp[31 : 16];
  assign lowState = cmp[15 : 0];
  assign enableState = (isEnabled ? 16'hffff : 16'h0);
  assign maxCnt = (cmp - 32'h00000001);
  assign interrupt = ((isEnabled && (cnt == maxCnt)) && (! (loadHigh || loadLow)));
  always @ (posedge core_clk) begin
    if(core_reset) begin
      cnt <= 32'h0;
      cmp <= 32'h0;
      isEnabled <= 1'b0;
    end else begin
      if(accessEnableWrite)begin
        isEnabled <= (enable != 16'h0);
      end
      if(loadLow)begin
        cmp[15 : 0] <= cmpLow;
        isEnabled <= 1'b0;
        cnt <= 32'h0;
      end
      if(loadHigh)begin
        cmp[31 : 16] <= cmpHigh;
        isEnabled <= 1'b0;
        cnt <= 32'h0;
      end
      if((isEnabled && (! (loadHigh || loadLow))))begin
        if((cnt < maxCnt))begin
          cnt <= (cnt + 32'h00000001);
        end else begin
          cnt <= 32'h0;
        end
      end
    end
  end


endmodule

module FlowCCByToggle (
  input               io_input_valid,
  input               io_input_payload_jtagDataValid,
  input               io_input_payload_jtagStall,
  input               io_input_payload_jtagCaptureMemory,
  input      [7:0]    io_input_payload_jtagCPUAdr,
  input      [15:0]   io_input_payload_jtagCPUWord,
  output              io_output_valid,
  output              io_output_payload_jtagDataValid,
  output              io_output_payload_jtagStall,
  output              io_output_payload_jtagCaptureMemory,
  output     [7:0]    io_output_payload_jtagCPUAdr,
  output     [15:0]   io_output_payload_jtagCPUWord,
  input               tck,
  input               reset,
  input               core_clk,
  input               core_reset
);
  wire                inputArea_target_buffercc_io_dataOut;
  wire                outHitSignal;
  reg                 inputArea_target;
  reg                 inputArea_data_jtagDataValid;
  reg                 inputArea_data_jtagStall;
  reg                 inputArea_data_jtagCaptureMemory;
  reg        [7:0]    inputArea_data_jtagCPUAdr;
  reg        [15:0]   inputArea_data_jtagCPUWord;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire                outputArea_flow_payload_jtagDataValid;
  wire                outputArea_flow_payload_jtagStall;
  wire                outputArea_flow_payload_jtagCaptureMemory;
  wire       [7:0]    outputArea_flow_payload_jtagCPUAdr;
  wire       [15:0]   outputArea_flow_payload_jtagCPUWord;
  reg                 outputArea_flow_regNext_valid;
  reg                 outputArea_flow_regNext_payload_jtagDataValid;
  reg                 outputArea_flow_regNext_payload_jtagStall;
  reg                 outputArea_flow_regNext_payload_jtagCaptureMemory;
  reg        [7:0]    outputArea_flow_regNext_payload_jtagCPUAdr;
  reg        [15:0]   outputArea_flow_regNext_payload_jtagCPUWord;

  BufferCC inputArea_target_buffercc (
    .io_dataIn     (inputArea_target                      ), //i
    .io_dataOut    (inputArea_target_buffercc_io_dataOut  ), //o
    .core_clk      (core_clk                              ), //i
    .core_reset    (core_reset                            )  //i
  );
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_jtagDataValid = inputArea_data_jtagDataValid;
  assign outputArea_flow_payload_jtagStall = inputArea_data_jtagStall;
  assign outputArea_flow_payload_jtagCaptureMemory = inputArea_data_jtagCaptureMemory;
  assign outputArea_flow_payload_jtagCPUAdr = inputArea_data_jtagCPUAdr;
  assign outputArea_flow_payload_jtagCPUWord = inputArea_data_jtagCPUWord;
  assign io_output_valid = outputArea_flow_regNext_valid;
  assign io_output_payload_jtagDataValid = outputArea_flow_regNext_payload_jtagDataValid;
  assign io_output_payload_jtagStall = outputArea_flow_regNext_payload_jtagStall;
  assign io_output_payload_jtagCaptureMemory = outputArea_flow_regNext_payload_jtagCaptureMemory;
  assign io_output_payload_jtagCPUAdr = outputArea_flow_regNext_payload_jtagCPUAdr;
  assign io_output_payload_jtagCPUWord = outputArea_flow_regNext_payload_jtagCPUWord;
  always @ (posedge tck or posedge reset) begin
    if (reset) begin
      inputArea_target <= 1'b0;
    end else begin
      if(io_input_valid)begin
        inputArea_target <= (! inputArea_target);
      end
    end
  end

  always @ (posedge tck) begin
    if(io_input_valid)begin
      inputArea_data_jtagDataValid <= io_input_payload_jtagDataValid;
      inputArea_data_jtagStall <= io_input_payload_jtagStall;
      inputArea_data_jtagCaptureMemory <= io_input_payload_jtagCaptureMemory;
      inputArea_data_jtagCPUAdr <= io_input_payload_jtagCPUAdr;
      inputArea_data_jtagCPUWord <= io_input_payload_jtagCPUWord;
    end
  end

  always @ (posedge core_clk) begin
    if(core_reset) begin
      outputArea_flow_regNext_valid <= 1'b0;
      outputArea_hit <= 1'b0;
    end else begin
      outputArea_hit <= outputArea_target;
      outputArea_flow_regNext_valid <= outputArea_flow_valid;
    end
  end

  always @ (posedge core_clk) begin
    outputArea_flow_regNext_payload_jtagDataValid <= outputArea_flow_payload_jtagDataValid;
    outputArea_flow_regNext_payload_jtagStall <= outputArea_flow_payload_jtagStall;
    outputArea_flow_regNext_payload_jtagCaptureMemory <= outputArea_flow_payload_jtagCaptureMemory;
    outputArea_flow_regNext_payload_jtagCPUAdr <= outputArea_flow_payload_jtagCPUAdr;
    outputArea_flow_regNext_payload_jtagCPUWord <= outputArea_flow_payload_jtagCPUWord;
  end


endmodule

module J1 (
  input               stall,
  input               irq,
  input      [7:0]    intVec,
  input               captureMemory,
  input      [7:0]    jtagMemAdr,
  input      [15:0]   jtagMemWord,
  output              cpuBus_enable,
  output              cpuBus_writeMode,
  output     [15:0]   cpuBus_address,
  output     [15:0]   cpuBus_writeData,
  input      [15:0]   cpuBus_readData,
  input               core_reset,
  input               core_clk
);
  reg                 _zz_1;
  reg        [7:0]    _zz_2;
  reg        [15:0]   _zz_3;
  wire                coreJ1CPU_memWriteMode;
  wire                coreJ1CPU_ioWriteMode;
  wire                coreJ1CPU_ioReadMode;
  wire       [15:0]   coreJ1CPU_extAdr;
  wire       [15:0]   coreJ1CPU_extToWrite;
  wire       [7:0]    coreJ1CPU_nextInstrAdr;
  wire       [15:0]   mainMem_readData;
  wire       [15:0]   coreMemRead;
  reg        [15:0]   coreJ1CPU_extAdr_delay_1;

  J1Core coreJ1CPU (
    .memWriteMode    (coreJ1CPU_memWriteMode       ), //o
    .ioWriteMode     (coreJ1CPU_ioWriteMode        ), //o
    .ioReadMode      (coreJ1CPU_ioReadMode         ), //o
    .extAdr          (coreJ1CPU_extAdr[15:0]       ), //o
    .extToWrite      (coreJ1CPU_extToWrite[15:0]   ), //o
    .toRead          (coreMemRead[15:0]            ), //i
    .stall           (stall                        ), //i
    .irq             (irq                          ), //i
    .intVec          (intVec[7:0]                  ), //i
    .nextInstrAdr    (coreJ1CPU_nextInstrAdr[7:0]  ), //o
    .memInstr        (mainMem_readData[15:0]       ), //i
    .clrActive       (core_reset                   ), //i
    .core_clk        (core_clk                     )  //i
  );
  MainMemory mainMem (
    .readDataAdr     (coreJ1CPU_nextInstrAdr[7:0]  ), //i
    .readData        (mainMem_readData[15:0]       ), //o
    .writeEnable     (_zz_1                        ), //i
    .writeDataAdr    (_zz_2[7:0]                   ), //i
    .writeData       (_zz_3[15:0]                  ), //i
    .core_clk        (core_clk                     ), //i
    .core_reset      (core_reset                   )  //i
  );
  always @ (*) begin
    if(stall)begin
      _zz_1 = captureMemory;
      _zz_2 = jtagMemAdr;
      _zz_3 = jtagMemWord;
    end else begin
      _zz_1 = coreJ1CPU_memWriteMode;
      _zz_2 = coreJ1CPU_extAdr[8 : 1];
      _zz_3 = coreJ1CPU_extToWrite;
    end
  end

  assign coreMemRead = (coreJ1CPU_ioReadMode ? cpuBus_readData : 16'h0);
  assign cpuBus_enable = (coreJ1CPU_ioWriteMode || coreJ1CPU_ioReadMode);
  assign cpuBus_writeMode = coreJ1CPU_ioWriteMode;
  assign cpuBus_address = coreJ1CPU_extAdr_delay_1;
  assign cpuBus_writeData = coreJ1CPU_extToWrite;
  always @ (posedge core_clk) begin
    coreJ1CPU_extAdr_delay_1 <= coreJ1CPU_extAdr;
  end


endmodule

module BufferCC_3 (
  input               io_dataIn,
  output              io_dataOut,
  input               core_clk,
  input               _zz_1
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge core_clk or posedge _zz_1) begin
    if (_zz_1) begin
      buffers_0 <= 1'b1;
      buffers_1 <= 1'b1;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module J1Jtag (
  input               tdi,
  output reg          tdo,
  input               tms,
  output              jtagDataFlow_valid,
  output              jtagDataFlow_payload_jtagDataValid,
  output              jtagDataFlow_payload_jtagStall,
  output              jtagDataFlow_payload_jtagCaptureMemory,
  output     [7:0]    jtagDataFlow_payload_jtagCPUAdr,
  output     [15:0]   jtagDataFlow_payload_jtagCPUWord,
  output              jtagReset,
  input               tck,
  input               reset
);
  wire       [4:0]    _zz_8;
  reg                 jtagDataValid;
  wire       [4:0]    _zz_1;
  wire       [4:0]    _zz_2;
  wire       [4:0]    _zz_3;
  wire       [4:0]    _zz_4;
  wire       [4:0]    _zz_5;
  wire       [4:0]    _zz_6;
  wire       [4:0]    _zz_7;
  reg        [4:0]    instructionShiftReg = 5'b00000;
  reg        [4:0]    instructionHoldReg = 5'b00000;
  reg        [0:0]    dataHoldRegs_0;
  reg        [31:0]   dataHoldRegs_1;
  reg        [0:0]    dataHoldRegs_2;
  reg        [0:0]    dataHoldRegs_3;
  reg        [0:0]    dataHoldRegs_4;
  reg        [7:0]    dataHoldRegs_5;
  reg        [15:0]   dataHoldRegs_6;
  reg        [0:0]    dataShiftRegs_0;
  reg        [31:0]   dataShiftRegs_1;
  reg        [0:0]    dataShiftRegs_2;
  reg        [0:0]    dataShiftRegs_3;
  reg        [0:0]    dataShiftRegs_4;
  reg        [7:0]    dataShiftRegs_5;
  reg        [15:0]   dataShiftRegs_6;
  wire                jtagFSM_wantExit;
  reg                 jtagFSM_wantStart;
  wire                jtagDataBundle_jtagDataValid;
  wire                jtagDataBundle_jtagStall;
  wire                jtagDataBundle_jtagCaptureMemory;
  wire       [7:0]    jtagDataBundle_jtagCPUAdr;
  wire       [15:0]   jtagDataBundle_jtagCPUWord;
  reg        `jtagFSM_enumDefinition_defaultEncoding_type jtagFSM_stateReg;
  reg        `jtagFSM_enumDefinition_defaultEncoding_type jtagFSM_stateNext;
  `ifndef SYNTHESIS
  reg [175:0] jtagFSM_stateReg_string;
  reg [175:0] jtagFSM_stateNext_string;
  `endif


  assign _zz_8 = 5'h19;
  `ifndef SYNTHESIS
  always @(*) begin
    case(jtagFSM_stateReg)
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_BOOT : jtagFSM_stateReg_string = "jtagFSM_BOOT          ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset : jtagFSM_stateReg_string = "jtagFSM_testLogicReset";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle : jtagFSM_stateReg_string = "jtagFSM_runTestIdle   ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan : jtagFSM_stateReg_string = "jtagFSM_selectDRScan  ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR : jtagFSM_stateReg_string = "jtagFSM_captureDR     ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR : jtagFSM_stateReg_string = "jtagFSM_shiftDR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR : jtagFSM_stateReg_string = "jtagFSM_exit1DR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR : jtagFSM_stateReg_string = "jtagFSM_pauseDR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR : jtagFSM_stateReg_string = "jtagFSM_exit2DR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR : jtagFSM_stateReg_string = "jtagFSM_updateDR      ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan : jtagFSM_stateReg_string = "jtagFSM_selectIRScan  ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR : jtagFSM_stateReg_string = "jtagFSM_captureIR     ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR : jtagFSM_stateReg_string = "jtagFSM_shiftIR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR : jtagFSM_stateReg_string = "jtagFSM_exit1IR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR : jtagFSM_stateReg_string = "jtagFSM_pauseIR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR : jtagFSM_stateReg_string = "jtagFSM_exit2IR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR : jtagFSM_stateReg_string = "jtagFSM_updateIR      ";
      default : jtagFSM_stateReg_string = "??????????????????????";
    endcase
  end
  always @(*) begin
    case(jtagFSM_stateNext)
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_BOOT : jtagFSM_stateNext_string = "jtagFSM_BOOT          ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset : jtagFSM_stateNext_string = "jtagFSM_testLogicReset";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle : jtagFSM_stateNext_string = "jtagFSM_runTestIdle   ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan : jtagFSM_stateNext_string = "jtagFSM_selectDRScan  ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR : jtagFSM_stateNext_string = "jtagFSM_captureDR     ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR : jtagFSM_stateNext_string = "jtagFSM_shiftDR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR : jtagFSM_stateNext_string = "jtagFSM_exit1DR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR : jtagFSM_stateNext_string = "jtagFSM_pauseDR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR : jtagFSM_stateNext_string = "jtagFSM_exit2DR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR : jtagFSM_stateNext_string = "jtagFSM_updateDR      ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan : jtagFSM_stateNext_string = "jtagFSM_selectIRScan  ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR : jtagFSM_stateNext_string = "jtagFSM_captureIR     ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR : jtagFSM_stateNext_string = "jtagFSM_shiftIR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR : jtagFSM_stateNext_string = "jtagFSM_exit1IR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR : jtagFSM_stateNext_string = "jtagFSM_pauseIR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR : jtagFSM_stateNext_string = "jtagFSM_exit2IR       ";
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR : jtagFSM_stateNext_string = "jtagFSM_updateIR      ";
      default : jtagFSM_stateNext_string = "??????????????????????";
    endcase
  end
  `endif

  assign _zz_1 = (1'b1 ? 5'h1f : 5'h0);
  assign _zz_2 = 5'h05;
  assign _zz_3 = 5'h09;
  assign _zz_4 = 5'h0d;
  assign _zz_5 = 5'h11;
  assign _zz_6 = 5'h15;
  assign _zz_7 = 5'h19;
  assign jtagFSM_wantExit = 1'b0;
  always @ (*) begin
    jtagFSM_wantStart = 1'b0;
    jtagDataValid = 1'b0;
    tdo = dataShiftRegs_0[0];
    if((instructionHoldReg == _zz_1))begin
      tdo = dataShiftRegs_0[0];
    end
    if((instructionHoldReg == _zz_7))begin
      tdo = dataShiftRegs_1[0];
    end
    if((instructionHoldReg == _zz_2))begin
      tdo = dataShiftRegs_2[0];
    end
    if((instructionHoldReg == _zz_3))begin
      tdo = dataShiftRegs_3[0];
    end
    if((instructionHoldReg == _zz_4))begin
      tdo = dataShiftRegs_4[0];
    end
    if((instructionHoldReg == _zz_5))begin
      tdo = dataShiftRegs_5[0];
    end
    if((instructionHoldReg == _zz_6))begin
      tdo = dataShiftRegs_6[0];
    end
    jtagFSM_stateNext = jtagFSM_stateReg;
    case(jtagFSM_stateReg)
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset : begin
        jtagDataValid = 1'b1;
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle : begin
        jtagDataValid = 1'b1;
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR : begin
        tdo = instructionShiftReg[0];
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR : begin
        if(tms)begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan;
        end else begin
          jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle;
        end
      end
      default : begin
        jtagFSM_wantStart = 1'b1;
      end
    endcase
    if(((jtagFSM_stateReg == `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR) && (! (jtagFSM_stateNext == `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR))))begin
      jtagDataValid = 1'b1;
    end
    if(((jtagFSM_stateReg == `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR) && (! (jtagFSM_stateNext == `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR))))begin
      jtagDataValid = 1'b1;
    end
    if(jtagFSM_wantStart)begin
      jtagFSM_stateNext = `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset;
    end
  end

  assign jtagDataBundle_jtagStall = dataHoldRegs_2[0];
  assign jtagReset = dataHoldRegs_3[0];
  assign jtagDataBundle_jtagDataValid = jtagDataValid;
  assign jtagDataBundle_jtagCaptureMemory = dataHoldRegs_4[0];
  assign jtagDataBundle_jtagCPUAdr = dataHoldRegs_5;
  assign jtagDataBundle_jtagCPUWord = dataHoldRegs_6;
  assign jtagDataFlow_payload_jtagDataValid = jtagDataBundle_jtagDataValid;
  assign jtagDataFlow_payload_jtagStall = jtagDataBundle_jtagStall;
  assign jtagDataFlow_payload_jtagCaptureMemory = jtagDataBundle_jtagCaptureMemory;
  assign jtagDataFlow_payload_jtagCPUAdr = jtagDataBundle_jtagCPUAdr;
  assign jtagDataFlow_payload_jtagCPUWord = jtagDataBundle_jtagCPUWord;
  assign jtagDataFlow_valid = jtagDataValid;
  always @ (posedge tck or posedge reset) begin
    if (reset) begin
      dataHoldRegs_2 <= 1'b0;
      dataHoldRegs_3 <= 1'b0;
      dataHoldRegs_4 <= 1'b0;
      jtagFSM_stateReg <= `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_BOOT;
    end else begin
      jtagFSM_stateReg <= jtagFSM_stateNext;
      case(jtagFSM_stateReg)
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR : begin
          if((instructionHoldReg == _zz_2))begin
            dataHoldRegs_2 <= dataShiftRegs_2;
          end
          if((instructionHoldReg == _zz_3))begin
            dataHoldRegs_3 <= dataShiftRegs_3;
          end
          if((instructionHoldReg == _zz_4))begin
            dataHoldRegs_4 <= dataShiftRegs_4;
          end
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR : begin
        end
        `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @ (posedge tck) begin
    case(jtagFSM_stateReg)
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_testLogicReset : begin
        instructionHoldReg <= 5'h19;
        dataHoldRegs_1 <= {27'd0, _zz_8};
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_runTestIdle : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectDRScan : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureDR : begin
        if((instructionHoldReg == _zz_1))begin
          dataShiftRegs_0 <= dataHoldRegs_0;
        end
        if((instructionHoldReg == _zz_7))begin
          dataShiftRegs_1 <= 32'h01234567;
        end
        if((instructionHoldReg == _zz_2))begin
          dataShiftRegs_2 <= dataHoldRegs_2;
        end
        if((instructionHoldReg == _zz_3))begin
          dataShiftRegs_3 <= dataHoldRegs_3;
        end
        if((instructionHoldReg == _zz_4))begin
          dataShiftRegs_4 <= dataHoldRegs_4;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftDR : begin
        if((instructionHoldReg == _zz_1))begin
          dataShiftRegs_0 <= ({tdi,dataShiftRegs_0} >>> 1);
        end
        if((instructionHoldReg == _zz_7))begin
          dataShiftRegs_1 <= ({tdi,dataShiftRegs_1} >>> 1);
        end
        if((instructionHoldReg == _zz_2))begin
          dataShiftRegs_2 <= ({tdi,dataShiftRegs_2} >>> 1);
        end
        if((instructionHoldReg == _zz_3))begin
          dataShiftRegs_3 <= ({tdi,dataShiftRegs_3} >>> 1);
        end
        if((instructionHoldReg == _zz_4))begin
          dataShiftRegs_4 <= ({tdi,dataShiftRegs_4} >>> 1);
        end
        if((instructionHoldReg == _zz_5))begin
          dataShiftRegs_5 <= ({tdi,dataShiftRegs_5} >>> 1);
        end
        if((instructionHoldReg == _zz_6))begin
          dataShiftRegs_6 <= ({tdi,dataShiftRegs_6} >>> 1);
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1DR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseDR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2DR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateDR : begin
        if((instructionHoldReg == _zz_1))begin
          dataHoldRegs_0 <= dataShiftRegs_0;
        end
        if((instructionHoldReg == _zz_5))begin
          dataHoldRegs_5 <= dataShiftRegs_5;
        end
        if((instructionHoldReg == _zz_6))begin
          dataHoldRegs_6 <= dataShiftRegs_6;
        end
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_selectIRScan : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_captureIR : begin
        instructionShiftReg <= (1'b1 ? 5'h1f : 5'h0);
        instructionShiftReg[0] <= 1'b1;
        instructionShiftReg[1] <= 1'b0;
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_shiftIR : begin
        instructionShiftReg <= ({tdi,instructionShiftReg} >>> 1);
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit1IR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_pauseIR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_exit2IR : begin
      end
      `jtagFSM_enumDefinition_defaultEncoding_jtagFSM_updateIR : begin
        instructionHoldReg <= instructionShiftReg;
      end
      default : begin
      end
    endcase
  end


endmodule

module BufferCC_2 (
  input      [3:0]    io_dataIn,
  output     [3:0]    io_dataOut,
  input               core_clk,
  input               core_reset
);
  reg        [3:0]    buffers_0;
  reg        [3:0]    buffers_1;
  reg        [3:0]    buffers_2;

  assign io_dataOut = buffers_2;
  always @ (posedge core_clk) begin
    if(core_reset) begin
      buffers_0 <= 4'b0000;
      buffers_1 <= 4'b0000;
      buffers_2 <= 4'b0000;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
      buffers_2 <= buffers_1;
    end
  end


endmodule

module UartCtrlRx (
  input      [2:0]    io_configFrame_dataLength,
  input      `UartStopType_defaultEncoding_type io_configFrame_stop,
  input      `UartParityType_defaultEncoding_type io_configFrame_parity,
  input               io_samplingTick,
  output              io_read_valid,
  input               io_read_ready,
  output     [7:0]    io_read_payload,
  input               io_rxd,
  output              io_rts,
  output reg          io_error,
  output              io_break,
  input               core_clk,
  input               core_reset
);
  wire                io_rxd_buffercc_io_dataOut;
  wire                _zz_2;
  wire                _zz_3;
  wire                _zz_4;
  wire                _zz_5;
  wire       [0:0]    _zz_6;
  wire       [2:0]    _zz_7;
  wire                _zz_8;
  wire                _zz_9;
  wire                _zz_10;
  wire                _zz_11;
  wire                _zz_12;
  wire                _zz_13;
  wire                _zz_14;
  reg                 _zz_1;
  wire                sampler_synchroniser;
  wire                sampler_samples_0;
  reg                 sampler_samples_1;
  reg                 sampler_samples_2;
  reg                 sampler_samples_3;
  reg                 sampler_samples_4;
  reg                 sampler_value;
  reg                 sampler_tick;
  reg        [2:0]    bitTimer_counter;
  reg                 bitTimer_tick;
  reg        [2:0]    bitCounter_value;
  reg        [6:0]    break_counter;
  wire                break_valid;
  reg        `UartCtrlRxState_defaultEncoding_type stateMachine_state;
  reg                 stateMachine_parity;
  reg        [7:0]    stateMachine_shifter;
  reg                 stateMachine_validReg;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif


  assign _zz_2 = (stateMachine_parity == sampler_value);
  assign _zz_3 = (! sampler_value);
  assign _zz_4 = ((sampler_tick && (! sampler_value)) && (! break_valid));
  assign _zz_5 = (bitCounter_value == io_configFrame_dataLength);
  assign _zz_6 = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? 1'b0 : 1'b1);
  assign _zz_7 = {2'd0, _zz_6};
  assign _zz_8 = ((((1'b0 || ((_zz_13 && sampler_samples_1) && sampler_samples_2)) || (((_zz_14 && sampler_samples_0) && sampler_samples_1) && sampler_samples_3)) || (((1'b1 && sampler_samples_0) && sampler_samples_2) && sampler_samples_3)) || (((1'b1 && sampler_samples_1) && sampler_samples_2) && sampler_samples_3));
  assign _zz_9 = (((1'b1 && sampler_samples_0) && sampler_samples_1) && sampler_samples_4);
  assign _zz_10 = ((1'b1 && sampler_samples_0) && sampler_samples_2);
  assign _zz_11 = (1'b1 && sampler_samples_1);
  assign _zz_12 = 1'b1;
  assign _zz_13 = (1'b1 && sampler_samples_0);
  assign _zz_14 = 1'b1;
  BufferCC io_rxd_buffercc (
    .io_dataIn     (io_rxd                      ), //i
    .io_dataOut    (io_rxd_buffercc_io_dataOut  ), //o
    .core_clk      (core_clk                    ), //i
    .core_reset    (core_reset                  )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlRxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlRxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlRxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlRxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    io_error = 1'b0;
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlRxState_defaultEncoding_START : begin
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          if(! _zz_2) begin
            io_error = 1'b1;
          end
        end
      end
      default : begin
        if(bitTimer_tick)begin
          if(_zz_3)begin
            io_error = 1'b1;
          end
        end
      end
    endcase
  end

  assign io_rts = _zz_1;
  assign sampler_synchroniser = io_rxd_buffercc_io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @ (*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick)begin
      if((bitTimer_counter == 3'b000))begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign break_valid = (break_counter == 7'h68);
  assign io_break = break_valid;
  assign io_read_valid = stateMachine_validReg;
  assign io_read_payload = stateMachine_shifter;
  always @ (posedge core_clk) begin
    if(core_reset) begin
      _zz_1 <= 1'b0;
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_samples_3 <= 1'b1;
      sampler_samples_4 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      break_counter <= 7'h0;
      stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      _zz_1 <= (! io_read_ready);
      if(io_samplingTick)begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick)begin
        sampler_samples_2 <= sampler_samples_1;
      end
      if(io_samplingTick)begin
        sampler_samples_3 <= sampler_samples_2;
      end
      if(io_samplingTick)begin
        sampler_samples_4 <= sampler_samples_3;
      end
      sampler_value <= ((((((_zz_8 || _zz_9) || (_zz_10 && sampler_samples_4)) || ((_zz_11 && sampler_samples_2) && sampler_samples_4)) || (((_zz_12 && sampler_samples_0) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_1) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_2) && sampler_samples_3) && sampler_samples_4));
      sampler_tick <= io_samplingTick;
      if(sampler_value)begin
        break_counter <= 7'h0;
      end else begin
        if((io_samplingTick && (! break_valid)))begin
          break_counter <= (break_counter + 7'h01);
        end
      end
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        `UartCtrlRxState_defaultEncoding_IDLE : begin
          if(_zz_4)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_START;
          end
        end
        `UartCtrlRxState_defaultEncoding_START : begin
          if(bitTimer_tick)begin
            stateMachine_state <= `UartCtrlRxState_defaultEncoding_DATA;
            if((sampler_value == 1'b1))begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_DATA : begin
          if(bitTimer_tick)begin
            if(_zz_5)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlRxState_defaultEncoding_PARITY : begin
          if(bitTimer_tick)begin
            if(_zz_2)begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick)begin
            if(_zz_3)begin
              stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
            end else begin
              if((bitCounter_value == _zz_7))begin
                stateMachine_state <= `UartCtrlRxState_defaultEncoding_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @ (posedge core_clk) begin
    if(sampler_tick)begin
      bitTimer_counter <= (bitTimer_counter - 3'b001);
    end
    if(bitTimer_tick)begin
      bitCounter_value <= (bitCounter_value + 3'b001);
    end
    if(bitTimer_tick)begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      `UartCtrlRxState_defaultEncoding_IDLE : begin
        if(_zz_4)begin
          bitTimer_counter <= 3'b010;
        end
      end
      `UartCtrlRxState_defaultEncoding_START : begin
        if(bitTimer_tick)begin
          bitCounter_value <= 3'b000;
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
        end
      end
      `UartCtrlRxState_defaultEncoding_DATA : begin
        if(bitTimer_tick)begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(_zz_5)begin
            bitCounter_value <= 3'b000;
          end
        end
      end
      `UartCtrlRxState_defaultEncoding_PARITY : begin
        if(bitTimer_tick)begin
          bitCounter_value <= 3'b000;
        end
      end
      default : begin
      end
    endcase
  end


endmodule

module UartCtrlTx (
  input      [2:0]    io_configFrame_dataLength,
  input      `UartStopType_defaultEncoding_type io_configFrame_stop,
  input      `UartParityType_defaultEncoding_type io_configFrame_parity,
  input               io_samplingTick,
  input               io_write_valid,
  output reg          io_write_ready,
  input      [7:0]    io_write_payload,
  input               io_cts,
  output              io_txd,
  input               io_break,
  input               core_clk,
  input               core_reset
);
  wire                _zz_2;
  wire       [0:0]    _zz_3;
  wire       [2:0]    _zz_4;
  wire       [0:0]    _zz_5;
  wire       [2:0]    _zz_6;
  reg                 clockDivider_counter_willIncrement;
  wire                clockDivider_counter_willClear;
  reg        [2:0]    clockDivider_counter_valueNext;
  reg        [2:0]    clockDivider_counter_value;
  wire                clockDivider_counter_willOverflowIfInc;
  wire                clockDivider_counter_willOverflow;
  reg        [2:0]    tickCounter_value;
  reg        `UartCtrlTxState_defaultEncoding_type stateMachine_state;
  reg                 stateMachine_parity;
  reg                 stateMachine_txd;
  reg                 _zz_1;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif


  assign _zz_2 = (tickCounter_value == io_configFrame_dataLength);
  assign _zz_3 = clockDivider_counter_willIncrement;
  assign _zz_4 = {2'd0, _zz_3};
  assign _zz_5 = ((io_configFrame_stop == `UartStopType_defaultEncoding_ONE) ? 1'b0 : 1'b1);
  assign _zz_6 = {2'd0, _zz_5};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      `UartStopType_defaultEncoding_ONE : io_configFrame_stop_string = "ONE";
      `UartStopType_defaultEncoding_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      `UartParityType_defaultEncoding_NONE : io_configFrame_parity_string = "NONE";
      `UartParityType_defaultEncoding_EVEN : io_configFrame_parity_string = "EVEN";
      `UartParityType_defaultEncoding_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : stateMachine_state_string = "IDLE  ";
      `UartCtrlTxState_defaultEncoding_START : stateMachine_state_string = "START ";
      `UartCtrlTxState_defaultEncoding_DATA : stateMachine_state_string = "DATA  ";
      `UartCtrlTxState_defaultEncoding_PARITY : stateMachine_state_string = "PARITY";
      `UartCtrlTxState_defaultEncoding_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @ (*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick)begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == 3'b111);
  assign clockDivider_counter_willOverflow = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @ (*) begin
    clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_4);
    if(clockDivider_counter_willClear)begin
      clockDivider_counter_valueNext = 3'b000;
    end
  end

  always @ (*) begin
    stateMachine_txd = 1'b1;
    io_write_ready = io_break;
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        stateMachine_txd = 1'b0;
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
        if(clockDivider_counter_willOverflow)begin
          if(_zz_2)begin
            io_write_ready = 1'b1;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  assign io_txd = _zz_1;
  always @ (posedge core_clk) begin
    if(core_reset) begin
      clockDivider_counter_value <= 3'b000;
      stateMachine_state <= `UartCtrlTxState_defaultEncoding_IDLE;
      _zz_1 <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        `UartCtrlTxState_defaultEncoding_IDLE : begin
          if(((io_write_valid && (! io_cts)) && clockDivider_counter_willOverflow))begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_START;
          end
        end
        `UartCtrlTxState_defaultEncoding_START : begin
          if(clockDivider_counter_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_DATA;
          end
        end
        `UartCtrlTxState_defaultEncoding_DATA : begin
          if(clockDivider_counter_willOverflow)begin
            if(_zz_2)begin
              if((io_configFrame_parity == `UartParityType_defaultEncoding_NONE))begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
              end else begin
                stateMachine_state <= `UartCtrlTxState_defaultEncoding_PARITY;
              end
            end
          end
        end
        `UartCtrlTxState_defaultEncoding_PARITY : begin
          if(clockDivider_counter_willOverflow)begin
            stateMachine_state <= `UartCtrlTxState_defaultEncoding_STOP;
          end
        end
        default : begin
          if(clockDivider_counter_willOverflow)begin
            if((tickCounter_value == _zz_6))begin
              stateMachine_state <= (io_write_valid ? `UartCtrlTxState_defaultEncoding_START : `UartCtrlTxState_defaultEncoding_IDLE);
            end
          end
        end
      endcase
      _zz_1 <= (stateMachine_txd && (! io_break));
    end
  end

  always @ (posedge core_clk) begin
    if(clockDivider_counter_willOverflow)begin
      tickCounter_value <= (tickCounter_value + 3'b001);
    end
    if(clockDivider_counter_willOverflow)begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      `UartCtrlTxState_defaultEncoding_IDLE : begin
      end
      `UartCtrlTxState_defaultEncoding_START : begin
        if(clockDivider_counter_willOverflow)begin
          stateMachine_parity <= (io_configFrame_parity == `UartParityType_defaultEncoding_ODD);
          tickCounter_value <= 3'b000;
        end
      end
      `UartCtrlTxState_defaultEncoding_DATA : begin
        if(clockDivider_counter_willOverflow)begin
          if(_zz_2)begin
            tickCounter_value <= 3'b000;
          end
        end
      end
      `UartCtrlTxState_defaultEncoding_PARITY : begin
        if(clockDivider_counter_willOverflow)begin
          tickCounter_value <= 3'b000;
        end
      end
      default : begin
      end
    endcase
  end


endmodule

//BufferCC replaced by BufferCC

module MainMemory (
  input      [7:0]    readDataAdr,
  output     [15:0]   readData,
  input               writeEnable,
  input      [7:0]    writeDataAdr,
  input      [15:0]   writeData,
  input               core_clk,
  input               core_reset
);
  reg        [15:0]   _zz_4;
  reg        [15:0]   _zz_5;
  reg        [15:0]   _zz_6;
  wire       [6:0]    _zz_7;
  wire                _zz_8;
  wire                _zz_9;
  wire       [6:0]    _zz_10;
  wire                _zz_11;
  wire                _zz_12;
  wire       [6:0]    _zz_1;
  wire       [15:0]   rPortsVec_0;
  wire       [6:0]    _zz_2;
  wire       [15:0]   rPortsVec_1;
  reg        [0:0]    _zz_3;
  reg [15:0] ramList_0 [0:127];
  reg [15:0] ramList_1 [0:127];

  assign _zz_7 = writeDataAdr[6 : 0];
  assign _zz_8 = (writeEnable && (1'b0 == writeDataAdr[7 : 7]));
  assign _zz_9 = 1'b1;
  assign _zz_10 = writeDataAdr[6 : 0];
  assign _zz_11 = (writeEnable && (1'b1 == writeDataAdr[7 : 7]));
  assign _zz_12 = 1'b1;
  always @ (posedge core_clk) begin
    if(_zz_8) begin
      ramList_0[_zz_7] <= writeData;
    end
  end

  always @ (posedge core_clk) begin
    if(_zz_9) begin
      _zz_4 <= ramList_0[_zz_1];
    end
  end

  always @ (posedge core_clk) begin
    if(_zz_11) begin
      ramList_1[_zz_10] <= writeData;
    end
  end

  always @ (posedge core_clk) begin
    if(_zz_12) begin
      _zz_5 <= ramList_1[_zz_2];
    end
  end

  always @(*) begin
    case(_zz_3)
      1'b0 : begin
        _zz_6 = rPortsVec_0;
      end
      default : begin
        _zz_6 = rPortsVec_1;
      end
    endcase
  end

  assign _zz_1 = readDataAdr[6 : 0];
  assign rPortsVec_0 = _zz_4;
  assign _zz_2 = readDataAdr[6 : 0];
  assign rPortsVec_1 = _zz_5;
  assign readData = _zz_6;
  always @ (posedge core_clk) begin
    _zz_3 <= readDataAdr[7 : 7];
  end


endmodule

module J1Core (
  output              memWriteMode,
  output              ioWriteMode,
  output              ioReadMode,
  output     [15:0]   extAdr,
  output     [15:0]   extToWrite,
  input      [15:0]   toRead,
  input               stall,
  input               irq,
  input      [7:0]    intVec,
  output     [7:0]    nextInstrAdr,
  input      [15:0]   memInstr,
  input               clrActive,
  input               core_clk
);
  wire       [15:0]   _zz_13;
  wire       [15:0]   _zz_14;
  wire       [3:0]    _zz_15;
  wire       [12:0]   _zz_16;
  wire       [9:0]    _zz_17;
  wire       [15:0]   _zz_18;
  wire       [16:0]   _zz_19;
  wire       [16:0]   _zz_20;
  wire       [16:0]   _zz_21;
  wire       [16:0]   _zz_22;
  wire       [15:0]   _zz_23;
  wire       [15:0]   _zz_24;
  wire       [16:0]   _zz_25;
  wire       [15:0]   _zz_26;
  wire       [14:0]   _zz_27;
  wire       [1:0]    _zz_28;
  wire       [4:0]    _zz_29;
  wire       [4:0]    _zz_30;
  wire       [1:0]    _zz_31;
  wire       [4:0]    _zz_32;
  wire       [4:0]    _zz_33;
  wire                _zz_34;
  wire                _zz_35;
  reg        [8:0]    pc;
  wire       [8:0]    _zz_1;
  wire       [8:0]    pcN;
  wire       [8:0]    returnPC;
  wire       [1:0]    stateSelect;
  reg        [15:0]   instr;
  reg                 dStack_stackWriteEnable;
  wire       [4:0]    dStack_stackPtrN;
  reg        [4:0]    dStack_stackPtr;
  wire       [15:0]   dtosN;
  reg        [15:0]   dtos;
  wire       [15:0]   dnos;
  wire       [15:0]   rtosN;
  reg                 rStack_stackWriteEnable;
  wire       [4:0]    rStack_stackPtrN;
  reg        [4:0]    rStack_stackPtr;
  wire       [15:0]   rtos;
  wire       [16:0]   _zz_4;
  reg        [15:0]   aluResult;
  reg        [15:0]   _zz_5;
  wire       [3:0]    _zz_6;
  wire                funcTtoN;
  wire                funcTtoR;
  wire                funcWriteMem;
  wire                funcWriteIO;
  wire                funcReadIO;
  wire                isALU;
  reg        [4:0]    _zz_7;
  wire       [3:0]    _zz_8;
  reg        [4:0]    _zz_9;
  wire       [3:0]    _zz_10;
  reg        [8:0]    _zz_11;
  wire       [7:0]    _zz_12;
  (* ram_style = "distributed" *) reg [15:0] _zz_2 [0:31];
  (* ram_style = "distributed" *) reg [15:0] _zz_3 [0:31];

  assign _zz_15 = instr[11 : 8];
  assign _zz_16 = {5'd0, intVec};
  assign _zz_17 = {returnPC,1'b0};
  assign _zz_18 = {6'd0, _zz_17};
  assign _zz_19 = _zz_20;
  assign _zz_20 = {1'd0, dnos};
  assign _zz_21 = _zz_22;
  assign _zz_22 = {1'd0, dtos};
  assign _zz_23 = (dtos + dnos);
  assign _zz_24 = _zz_4[15:0];
  assign _zz_25 = 17'h0;
  assign _zz_26 = {11'd0, dStack_stackPtr};
  assign _zz_27 = instr[14 : 0];
  assign _zz_28 = instr[1 : 0];
  assign _zz_29 = ($signed(_zz_30) + $signed(_zz_7));
  assign _zz_30 = dStack_stackPtr;
  assign _zz_31 = instr[3 : 2];
  assign _zz_32 = ($signed(_zz_33) + $signed(_zz_9));
  assign _zz_33 = rStack_stackPtr;
  assign _zz_34 = (dStack_stackWriteEnable && (! stall));
  assign _zz_35 = (rStack_stackWriteEnable && (! stall));
  always @ (posedge core_clk) begin
    if(_zz_34) begin
      _zz_2[dStack_stackPtrN] <= dtos;
    end
  end

  assign _zz_13 = _zz_2[dStack_stackPtr];
  always @ (posedge core_clk) begin
    if(_zz_35) begin
      _zz_3[rStack_stackPtrN] <= rtosN;
    end
  end

  assign _zz_14 = _zz_3[rStack_stackPtr];
  assign _zz_1 = (pc + 9'h001);
  assign returnPC = (irq ? pc : _zz_1);
  assign stateSelect = {stall,irq};
  always @ (*) begin
    case(stateSelect)
      2'b00 : begin
        instr = memInstr;
      end
      2'b01 : begin
        instr = {3'b010,_zz_16};
      end
      2'b10 : begin
        instr = 16'h6000;
      end
      default : begin
        instr = 16'h6000;
      end
    endcase
  end

  assign dnos = _zz_13;
  assign rtosN = ((! instr[13]) ? _zz_18 : dtos);
  assign rtos = _zz_14;
  assign _zz_4 = ($signed(_zz_19) - $signed(_zz_21));
  always @ (*) begin
    case(_zz_15)
      4'b0000 : begin
        aluResult = dtos;
      end
      4'b0001 : begin
        aluResult = dnos;
      end
      4'b0010 : begin
        aluResult = _zz_23;
      end
      4'b1100 : begin
        aluResult = _zz_24;
      end
      4'b0011 : begin
        aluResult = (dtos & dnos);
      end
      4'b0100 : begin
        aluResult = (dtos | dnos);
      end
      4'b0101 : begin
        aluResult = (dtos ^ dnos);
      end
      4'b0110 : begin
        aluResult = (~ dtos);
      end
      4'b1001 : begin
        aluResult = {dtos[15],dtos[15 : 1]};
      end
      4'b1010 : begin
        aluResult = {dtos[14 : 0],1'b0};
      end
      4'b1011 : begin
        aluResult = rtos;
      end
      4'b0111 : begin
        aluResult = (($signed(_zz_4) == $signed(_zz_25)) ? 16'hffff : 16'h0);
      end
      4'b1000 : begin
        aluResult = (((dtos[15] ^ dnos[15]) ? dnos[15] : _zz_4[16]) ? 16'hffff : 16'h0);
      end
      4'b1111 : begin
        aluResult = (_zz_4[16] ? 16'hffff : 16'h0);
      end
      4'b1101 : begin
        aluResult = toRead;
      end
      default : begin
        aluResult = _zz_26;
      end
    endcase
  end

  assign _zz_6 = {pc[8],instr[15 : 13]};
  always @ (*) begin
    if((((_zz_6 & 4'b1000) == 4'b1000))) begin
        _zz_5 = instr;
    end else if((((_zz_6 & 4'b1100) == 4'b0100))) begin
        _zz_5 = {1'd0, _zz_27};
    end else if((((_zz_6 & 4'b1111) == 4'b0000)) || (((_zz_6 & 4'b1111) == 4'b0010))) begin
        _zz_5 = dtos;
    end else if((((_zz_6 & 4'b1111) == 4'b0001))) begin
        _zz_5 = dnos;
    end else if((((_zz_6 & 4'b1111) == 4'b0011))) begin
        _zz_5 = aluResult;
    end else begin
        _zz_5 = (1'b1 ? 16'hffff : 16'h0);
    end
  end

  assign dtosN = _zz_5;
  assign funcTtoN = (instr[6 : 4] == 3'b001);
  assign funcTtoR = (instr[6 : 4] == 3'b010);
  assign funcWriteMem = (instr[6 : 4] == 3'b011);
  assign funcWriteIO = (instr[6 : 4] == 3'b100);
  assign funcReadIO = (instr[6 : 4] == 3'b101);
  assign isALU = ((! pc[8]) && (instr[15 : 13] == 3'b011));
  assign memWriteMode = (((! clrActive) && isALU) && funcWriteMem);
  assign ioWriteMode = (((! clrActive) && isALU) && funcWriteIO);
  assign ioReadMode = (((! clrActive) && isALU) && funcReadIO);
  assign extAdr = dtosN;
  assign extToWrite = dnos;
  assign _zz_8 = {pc[8],instr[15 : 13]};
  always @ (*) begin
    if((((_zz_8 & 4'b1000) == 4'b1000)) || (((_zz_8 & 4'b1100) == 4'b0100))) begin
        dStack_stackWriteEnable = 1'b1;
        _zz_7 = 5'h01;
    end else if((((_zz_8 & 4'b1111) == 4'b0001))) begin
        dStack_stackWriteEnable = 1'b0;
        _zz_7 = 5'h1f;
    end else if((((_zz_8 & 4'b1111) == 4'b0011))) begin
        dStack_stackWriteEnable = funcTtoN;
        _zz_7 = {{3{_zz_28[1]}}, _zz_28};
    end else begin
        dStack_stackWriteEnable = 1'b0;
        _zz_7 = 5'h0;
    end
  end

  assign dStack_stackPtrN = _zz_29;
  assign _zz_10 = {pc[8],instr[15 : 13]};
  always @ (*) begin
    if((((_zz_10 & 4'b1000) == 4'b1000))) begin
        rStack_stackWriteEnable = 1'b0;
        _zz_9 = 5'h1f;
    end else if((((_zz_10 & 4'b1111) == 4'b0010))) begin
        rStack_stackWriteEnable = 1'b1;
        _zz_9 = 5'h01;
    end else if((((_zz_10 & 4'b1111) == 4'b0011))) begin
        rStack_stackWriteEnable = funcTtoR;
        _zz_9 = {{3{_zz_31[1]}}, _zz_31};
    end else begin
        rStack_stackWriteEnable = 1'b0;
        _zz_9 = 5'h0;
    end
  end

  assign rStack_stackPtrN = _zz_32;
  assign _zz_12 = {{{{{stall,clrActive},pc[8]},instr[15 : 13]},instr[7]},(dtos != 16'h0)};
  always @ (*) begin
    if((((_zz_12 & 8'h80) == 8'h80))) begin
        _zz_11 = pc;
    end else if((((_zz_12 & 8'hc0) == 8'h40))) begin
        _zz_11 = 9'h0;
    end else if((((_zz_12 & 8'hfc) == 8'h0)) || (((_zz_12 & 8'hfc) == 8'h08)) || (((_zz_12 & 8'hfd) == 8'h04))) begin
        _zz_11 = instr[8 : 0];
    end else if((((_zz_12 & 8'he0) == 8'h20)) || (((_zz_12 & 8'hfe) == 8'h0e))) begin
        _zz_11 = rtos[9 : 1];
    end else begin
        _zz_11 = _zz_1;
    end
  end

  assign pcN = _zz_11;
  assign nextInstrAdr = pcN[7 : 0];
  always @ (posedge core_clk) begin
    if(clrActive) begin
      pc <= 9'h0;
      dStack_stackPtr <= 5'h0;
      dtos <= 16'h0;
      rStack_stackPtr <= 5'h0;
    end else begin
      if((! clrActive))begin
        pc <= pcN;
      end
      if((! stall))begin
        dStack_stackPtr <= dStack_stackPtrN;
      end
      dtos <= dtosN;
      if((! stall))begin
        rStack_stackPtr <= rStack_stackPtrN;
      end
    end
  end


endmodule

module BufferCC (
  input               io_dataIn,
  output              io_dataOut,
  input               core_clk,
  input               core_reset
);
  reg                 buffers_0;
  reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @ (posedge core_clk) begin
    if(core_reset) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule
