
`include "multiplier.v"

module cpu(
    input wire clk,
    input wire rst,
    input wire [63:0] ram_din,
    input wire ram_ready,
    output wire [63:0] ram_dout,
    output wire [7:0]  ram_mask,
    output wire [27:0] ram_addr,
    output wire ram_we,
    output wire ram_re
);

    parameter STATE_INVALID         = 0;
    parameter STATE_INIT            = 1;    // Initial state.
    parameter STATE_FETCH1          = 2;    // Fetch first halfword.
    parameter STATE_FETCH2          = 3;    // Fetch second halfword.
    parameter STATE_FETCH3          = 4;    // Fetch third halfword.
    parameter STATE_EXEC_RR         = 5;
    parameter STATE_SAVE_RR         = 6;
    parameter STATE_LOAD_RX         = 7;
    parameter STATE_EXEC_RX         = 8;
    parameter STATE_SAVE_RX         = 9;
    parameter STATE_EXEC_SS         = 10;
    parameter STATE_EXEC_RS         = 11;
    parameter STATE_EXEC_SI         = 12;

    reg [31:0] gpr [0:15];  // General-purpose registers.
    reg [63:0] fpr [0:3];   // Floating point registers.
    reg [31:0] cr [0:15];   // Control registers.
    reg [31:0] pc;          // Program Counter.
    reg [7:0] state;        // Current state.
    reg [47:0] opcode;      // Current instruction.
    reg [31:0] operand;     // Operand for RX instructions.
    reg alu_ready;          // ALU result is ready.
    wire split_transfer;    // Load/ltore not on a 64-bit boundary.
    reg transfer2;          // Transfer second part of a split.

    // Parts of the PSW.
    reg [4:0] channel_mask; // Channel Masks.
    reg io_mask;            // I/O Mask.
    reg external_mask;      // External Mask.
    reg [3:0] prot_key;     // Protection key.
    reg ec_mode;            // Extended Control mode.
    reg mc_mask;            // Machine Check Mask.
    reg wait_state;         // Wait state.
    reg problem_state;      // Problem state.
    reg [15:0] int_code;    // Interrupt code.
    reg [1:0] ilcode;       // Instruction length code.
    reg [1:0] cond_code;    // Condition code.
    reg [3:0] pmask;        // Program mask.
    reg recording_mask;     // Program Event Recording Mask.
    reg translation_mode;   // Translation Mode.

    // RR Instructions (1 halfword).
    wire spm = opcode[15:8] == 8'h04;   // Set Program Mask
    wire balr = opcode[15:8] == 8'h05;  // Branch And Link Register
    wire bctr = opcode[15:8] == 8'h06;  // Branch on CounT Register
    wire bcr = opcode[15:8] == 8'h07;   // Branch on Condition Register
    wire ssk = opcode[15:8] == 8'h08;   // Set Storage Key
    wire isk = opcode[15:8] == 8'h09;   // Insert Storage Key
    wire svc = opcode[15:8] == 8'h0A;   // SuperVisor Call
    wire bsm = opcode[15:8] == 8'h0B;   // Branch and Set Mode
    wire bassm = opcode[15:8] == 8'h0C; // Branch & Save & Set Mode
    wire basr = opcode[15:8] == 8'h0D;  // Branch and Store Register
    wire mvcl = opcode[15:8] == 8'h0E;  // MoVe Character Long
    wire clcl = opcode[15:8] == 8'h0F;  // Compare Logical Char Long
    wire lpr = opcode[15:8] == 8'h10;   // Load Positive Register
    wire lnr = opcode[15:8] == 8'h11;   // Load Negative Register
    wire ltr = opcode[15:8] == 8'h12;   // Load and Test Register
    wire type_rr = opcode[15:14] == 2'b00;
    wire [3:0] rr_r1 = opcode[7:4];
    wire [3:0] rr_r2 = opcode[3:0];

    // RX Instructions (2 halfwords).
    wire type_rx = opcode[15:14] == 2'b01;
    wire [3:0] rx_r = opcode[7:4];
    wire [3:0] rx_index = opcode[15:12];
    wire [3:0] rx_base = opcode[19:16];
    wire [11:0] rx_displacement = opcode[31:20];
    wire [31:0] rx_addr = rx_index + rx_base + rx_displacement;
    wire ah = opcode[15:8] == 8'h4A;    // Add Halfword
    wire sh = opcode[15:8] == 8'h4B;    // Subtract Halfword
    wire sth = opcode[15:8] == 8'h40;   // STore Halfword
    wire stc = opcode[15:8] == 8'h42;   // STore Character
    wire bal = opcode[15:8] == 8'h45;   // Branch And Link
    wire bct = opcode[15:8] == 8'h46;   // Branch on CounT
    wire bc = opcode[15:8] == 8'h47;    // Branch on Condition
    wire lh = opcode[15:8] == 8'h48;    // Load Halfword
    wire st = opcode[15:8] == 8'h50;    // STore
    wire std = opcode[15:8] == 8'h60;   // STore Double
    wire ld = opcode[15:8] == 8'h68;    // Load Double
    wire ste = opcode[15:8] == 8'h70;   // STore Short
    wire rx_store = sth | stc | st | ste | stm;
    wire rx_branch = bal | bct | bc;
    wire rx_load = ~rx_store & ~rx_branch;

    // RS Instructions (2 halfwords).
    wire bxh = opcode[15:8] == 8'h86;   // Branch on indeX High
    wire bxle = opcode[15:8] == 8'h87;  // Branch on indeX Low or Equal
    wire srl = opcode[15:8] == 8'h88;   // Shift Right Logical
    wire sll = opcode[15:8] == 8'h89;   // Shift Left Logical
    wire sra = opcode[15:8] == 8'h8A;   // Shift Right Arithmetic
    wire sla = opcode[15:8] == 8'h8B;   // Shift Left Arithmetic
    wire srdl = opcode[15:8] == 8'h8C;  // Shift Right Double Logical
    wire sldl = opcode[15:8] == 8'h8D;  // Shift Left Double Logical
    wire srda = opcode[15:8] == 8'h8E;  // Shift Right Double Arithmetic
    wire slda = opcode[15:8] == 8'h8F;  // Shift Left Double Arithmetic
    wire stm = opcode[15:8] == 8'h90;   // STore Multiple
    wire lm = opcode[15:8] == 8'h98;    // Load Multiple
    wire cs = opcode[15:8] == 8'hBA;    // Compare and Swap
    wire cds = opcode[15:8] == 8'hBB;   // Compare Double and Swap
    wire type_rs = bxh | bxle | srl | sll | sra | sla | srdl | sldl
                 | srda | slda | stm | lm | cs | cds;
    wire rs_store = stm;
    wire rs_branch = bxh | bxle;
    wire rs_load = ~rs_store & ~rs_branch;

    // SI Instructions (2 halfwords).
    wire ssm = opcode[15:8] == 8'h80;   // Set System Mask
    wire lpsw = opcode[15:8] == 8'h82;  // Load PSW
    wire diagnose = opcode[15:8] == 8'h83;
    wire wrd = opcode[15:8] == 8'h84;   // WRite Direct
    wire rdd = opcode[15:8] == 8'h85;   // ReaD Direct
    wire tm = opcode[15:8] == 8'h91;
    wire mvi = opcode[15:8] == 8'h92;
    wire ts = opcode[15:8] == 8'h93;
    wire ni = opcode[15:8] == 8'h94;
    wire cli = opcode[15:8] == 8'h95;
    wire oi = opcode[15:8] == 8'h96;
    wire xi = opcode[15:8] == 8'h97;
    wire sio = opcode[15:8] == 8'h9C;
    wire tio = opcode[15:8] == 8'h9D;
    wire hio = opcode[15:8] == 8'h9E;
    wire tch = opcode[15:8] == 8'h9F;
    wire type_si = ssm | lpsq | diagnose | wrd | rdd | tm | mvi | ts
                 | ni | cli | oi | xi | sio | tio | hio | tch;

    // SS Instructions (3 halfwords).
    wire nc = opcode[15:8] == 8'hD4;        // aNd Character
    wire xc = opcode[15:8] == 8'hD7;        // Xor Character
    wire type_ss = opcode[15:14] == 2'b11;

    // Instruction size (in bytes).
    wire size2 = type_rr;
    wire size4 = type_rx | type_rs | type_si;
    wire size6 = type_ss;

    // Determine the next state.
    wire transfer_done = ram_ready & ~split_transfer | transfer2;
    reg [7:0] next_state;
    always @(*) begin
        next_state <= STATE_INVALID;
        case (state)
            STATE_INIT:         next_state <= STATE_FETCH1;
            STATE_FETCH1:
                if (ram_ready) begin
                    // Instruction opcode not yet decoded, so we use
                    // the current memory input.
                    if (current_hw[15:14] == 2'b00) begin
                        next_state <= STATE_EXEC_RR;
                    end else begin
                        next_state <= STATE_FETCH2;
                    end
                end else begin
                    next_state <= STATE_FETCH1;
                end
            STATE_FETCH2:
                if (ram_ready) begin
                    // At this point, the opcode will be decoded.
                    case (1'b1)
                        type_rs:    next_state <= STATE_EXEC_RS;
                        type_rx:    next_state <= STATE_LOAD_RX;
                        type_si:    next_state <= STATE_EXEC_SI;
                        type_ss:    next_state <= STATE_EXEC_SS;
                        default:    next_state <= STATE_FETCH3;
                    endcase
                end else begin
                    next_state <= STATE_FETCH2;
                end
            STATE_FETCH3:
                if (ram_ready)  next_state <= STATE_EXEC_SS;
                else next_state <= STATE_FETCH3;
            STATE_EXEC_RR:
                if (alu_ready)  next_state <= STATE_SAVE_RR;
                else next_state <= STATE_EXEC_RR;
            STATE_SAVE_RR:      next_state <= STATE_FETCH1;
            STATE_LOAD_RX:
                if (~rx_load | transfer_done) next_state <= STATE_EXEC_RX;
                else next_state <= STATE_LOAD_RX;
            STATE_EXEC_RX:
                if (alu_ready)  next_state <= STATE_SAVE_RX;
                else next_state <= STATE_EXEC_RX;
            STATE_SAVE_RX:
                if (~rx_store | transfer_done) next_state <= STATE_FETCH1;
                else next_state <= STATE_SAVE_RX;
        endcase
    end

    // Update the state.
    always @(posedge clk) begin
        if (rst) begin
            state <= STATE_INIT;
        end else begin
            state <= next_state;
        end
    end

    // Demux the current halfword (for instruction fetches).
    // Note that reading on non-halfword boundaries is an exception.
    reg [15:0] current_hw;
    always @(*) begin
        case (pc[2:0])
            3'b000:     current_hw <= ram_din[63:48];
            3'b010:     current_hw <= ram_din[47:32];
            3'b100:     current_hw <= ram_din[31:16];
            3'b110:     current_hw <= ram_din[15:0];
        endcase
    end

    // Instruction fetch.
    always @(posedge clk) begin
        if (ram_ready) begin
            case (state)
                STATE_FETCH1: opcode[15:0]  <= current_hw;
                STATE_FETCH2: opcode[31:16] <= current_hw;
                STATE_FETCH3: opcode[47:32] <= current_hw;
            endcase
        end
    end

    // Update program counter.
    reg [31:0] next_pc;
    always @(*) begin
        if (ram_ready) begin
            case (state)
                STATE_FETCH1:   next_pc <= pc + 2;
                STATE_FETCH2:   next_pc <= pc + 2;
                STATE_FETCH3:   next_pc <= pc + 2;
                default:        next_pc <= pc;
            endcase
        end else begin
            next_pc <= pc;
        end
    end
    always @(posedge clk) begin
        if (rst) begin
            pc <= 0;
        end else begin
            pc <= next_pc;
        end
    end

    // Build up the PSW.
    reg [63:0] psw;
    always @(*) begin
        if (ec_mode == 0) begin
            // Basic Control Mode
            psw[63:40] <= pc[23:0];
            psw[39:36] <= pmask;
            psw[35:34] <= cond_code;
            psw[33:32] <= ilcode;
            psw[31:16] <= int_code;
            psw[15] <= problem_state;
            psw[14] <= wait_state;
            psw[13] <= mc_mask;
            psw[12] <= ec_mode;
            psw[11:8] <= prot_key;
            psw[7] <= external_mask;
            psw[6] <= io_mask;
            psw[5:0] <= channel_mask;
        end else begin
            // Extended Control Mode
            psw[63:40] <= pc[23:0];
            psw[39:32] <= 8'b00000000;
            psw[31:24] <= 8'b00000000;
            psw[23:20] <= pmask;
            psw[19:18] <= cond_code;
            psw[17:16] <= 2'b00;
            psw[15] <= problem_state;
            psw[14] <= wait_state;
            psw[13] <= mc_mask;
            psw[12] <= ec_mode;
            psw[11:8] <= prot_key;
            psw[7] <= external_mask;
            psw[6] <= io_mask;
            psw[5] <= translation_mode;
            psw[4:2] <= 3'b0;
            psw[1] <= recording_mask;
            psw[0] <= 0;
        end
    end

    // ALU input A.
    reg [31:0] alu_in_a;
    always @(*) begin
        alu_in_a <= 31'bx;
        case (1'b1)
            type_rr: alu_in_a <= gpr[rr_r1];
        endcase
    end

    // ALU input B.
    reg [31:0] alu_in_b;
    always @(*) begin
        alu_in_b <= 31'bx;
        case (1'b1)
            type_rr: alu_in_a <= gpr[rr_r2];
        endcase
    end

    // ALU function.
    parameter ALU_NOP   = 0;
    parameter ALU_LDN   = 1;
    parameter ALU_LDP   = 2;
    parameter ALU_AND   = 4;
    parameter ALU_CMPL  = 5;
    parameter ALU_OR    = 6;
    parameter ALU_XOR   = 7;
    parameter ALU_LOAD  = 8;
    parameter ALU_CMP   = 9;
    parameter ALU_ADD   = 10;
    parameter ALU_SUB   = 11;
    parameter ALU_MUL   = 12;
    parameter ALU_DIV   = 13;
    parameter ALU_ADDL  = 14;
    parameter ALU_SUBL  = 15;
    reg [7:0] alu_func;
    always @(*) begin
        case (1'b1)
            type_rr:    alu_func <= opcode[7:0];
            default:    alu_func <= ALU_NOP;
        endcase
    end

    // Start ALU processing.
    reg alu_start;
    always @(*) begin
        case (next_state)
            STATE_EXEC_RR:  alu_start <= alu_ready;
            default:        alu_start <= 0;
        endcase
    end

    wire [63:0] mul_result;
    wire mul_ready;
    multiplier mul(
        .clk(clk),
        .start(alu_start),
        .ina(alu_in_a),
        .inb(alu_in_b),
        .result(mul_result),
        .ready(mul_ready)
    );

    // ALU.
    reg [63:0] alu_result;
    reg [1:0] alu_cond;
    always @(*) begin
        case (alu_func)
            ALU_ADD:    alu_result <= alu_in_a + alu_in_b;
            ALU_ADDL:   alu_result <= alu_in_a + alu_in_b;
            ALU_AND:    alu_result <= alu_in_a & alu_in_b;
            ALU_MUL:    alu_result <= mul_result;
        endcase
    end
    always @(*) begin
        case (alu_func)
            ALU_ADD:
                if (ah && alu_result[15] != alu_result[16])
                    alu_cond <= 3;
                else if (~ah && alu_result[31] != alu_result[32])
                    alu_cond <= 3;
                else if (alu_result[15]) alu_cond <= 1;
                else if (alu_result[14:0] != 0) alu_cond <= 2;
                else alu_cond <= 0;
            ALU_ADDL:
                begin
                    alu_cond[1] <= alu_result[32];
                    alu_cond[0] <= alu_result[31:0] == 0;
                end
            ALU_AND:
                if (nc | ni)    alu_cond[0] <= alu_result[7:0] != 0;
                else            alu_cond[0] <= alu_result[31:0] != 0;
        endcase
    end
    always @(*) begin
        case (alu_func)
            ALU_MUL:    alu_ready <= mul_ready;
            default:    alu_ready <= 1;
        endcase
    end
    wire add_word_overflow = alu_result[31] != alu_result[32];
    wire add_hw_overflow = alu_result[15] != alu_result[16];

    // ALU result and condition code.
    always @(posedge clk) begin
        case (next_state)
            STATE_SAVE_RR:
                begin
                    gpr[rr_r1] <= alu_result[31:0];
                    cond_code  <= alu_cond;
                end
        endcase
    end

    // Determine the memory access size.
    // Note that we only care about writes.
    reg [3:0] access_bytes;
    always @(*) begin
        case (1'b1)
            nc:         access_bytes <= 1;
            ni:         access_bytes <= 1;
            xc:         access_bytes <= 1;
            xi:         access_bytes <= 1;
            stc:        access_bytes <= 1;
            ah:         access_bytes <= 2;
            sth:        access_bytes <= 2;
            sh:         access_bytes <= 2;
            cds:        access_bytes <= 8;
            slda:       access_bytes <= 8;
            sldl:       access_bytes <= 8;
            srda:       access_bytes <= 8;
            srdl:       access_bytes <= 8;
            default:    access_bytes <= 4;
        endcase
    end

    // Drive the memory address.
    reg [31:0] full_addr;
    always @(*) begin
        case (next_state)
            STATE_LOAD_RX:  full_addr <= rx_addr[31:3];
            STATE_SAVE_RX:  full_addr <= rx_addr[31:3];
            default:        full_addr <= pc[31:3];
        endcase
    end
    assign ram_addr = transfer2 ? (full_addr + 1) : full_addr;

    // Determine the transfer size.
    reg [3:0] transfer_bytes;
    always @(posedge clk) begin
        // Note that we can determine the transfer size from the
        // first halfword of the instruction.
        if (state == STATE_FETCH1) begin
            case (1'b1)
                stc:        transfer_bytes <= 1;
                sth:        transfer_bytes <= 2;
                std:        transfer_bytes <= 8;
                ld:         transfer_bytes <= 8;
                default:    transfer_bytes <= 4;
            endcase
        end
    end

    // Determine if this is a split transfer.
    wire [4:0] next_addr = full_addr[3:0] + transfer_bytes;
    assign split_transfer = next_addr[4] != full_addr[4];

    // Select the data to write.
    // This is left-aligned.
    reg [63:0] dout_unshifted;
    reg [7:0] mask_unshifted;
    always @(*) begin
        dout_unshifted <= {alu_result[31:0], 32'b0};
        mask_unshifted <= 8'b11110000;
    end

    // Shift the data and mask for output.
    reg [63:0] dout_shifted;
    reg [7:0] mask_shifted;
    genvar shift_i;
    generate
        for (shift_i = 0; shift_i < 8; shift_i = shift_i + 1) begin
            always @(*) begin
                if (transfer2) begin
                    if (full_addr[2:0] == 7 - shift_i) begin
                        dout_shifted <=
                            {dout_unshifted[63-8*shift_i:0], {shift_i{8'bz}}};
                        mask_shifted <=
                            {mask_unshifted[7-shift_i:0], {shift_i{1'b0}}};
                    end
                end else begin
                    if (full_addr[2:0] == shift_i) begin
                        dout_shifted <=
                            {{shift_i{8'bz}}, dout_unshifted[63:8*shift_i]};
                        mask_shifted <=
                            {{shift_i{1'b0}}, mask_unshifted[7:shift_i]};
                    end
                end
            end
        end
    endgenerate

    // Determine if a memory transfer should be started.
    reg start_read;
    reg start_write;
    always @(*) begin
        start_read <= 0;
        start_write <= 0;
        case (next_state)
            STATE_FETCH1:   start_read <= 1;
            STATE_FETCH2:   start_read <= 1;
            STATE_FETCH3:   start_read <= 1;
            STATE_LOAD_RX:  start_read <= rx_load;
            STATE_SAVE_RX:  start_write <= rx_store;
        endcase
    end

    // Handle memory transfers.
    always @(posedge clk) begin
        if (rst) begin
            transfer2 <= 0;
        end else if (ram_ready) begin
            transfer2 <= split_transfer & ~transfer2;
        end
    end
    assign ram_re = start_read & ram_ready;
    assign ram_we = start_write & ram_ready;

endmodule
