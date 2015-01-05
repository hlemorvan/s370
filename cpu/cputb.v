
`include "cpu.v"

module cputb;

    integer i, j;
    reg clk;
    reg rst;
    reg [63:0] data_m2c;
    wire [63:0] data_c2m;
    wire [7:0] data_mask;
    wire [27:0] data_addr;
    wire data_re;
    wire data_we;
    reg data_ready;

    reg [63:0] ram [0:255];
    reg [7:0] program [0:2047];

    initial begin

        // Load the program.
        $readmemh("test.hex", program);
        for (i = 0; i < 256; i = i + 1) begin
            ram[i][63:56] <= program[i * 8 + 0];
            ram[i][55:48] <= program[i * 8 + 1];
            ram[i][47:40] <= program[i * 8 + 2];
            ram[i][39:32] <= program[i * 8 + 3];
            ram[i][31:24] <= program[i * 8 + 4];
            ram[i][23:16] <= program[i * 8 + 5];
            ram[i][15:8]  <= program[i * 8 + 6];
            ram[i][7:0]   <= program[i * 8 + 7];
        end

        $dumpvars;

        // Reset
        clk <= 0;
        rst <= 1;
        #10 clk <= 1;
        #10 clk <= 0;
        rst <= 0;

        // Run for a bit.
        for(i = 0; i < 100; i = i + 1) begin
            #10 clk <= ~clk;
        end
    end

    cpu dut(
        .clk(clk),
        .rst(rst),
        .ram_din(data_m2c),
        .ram_dout(data_c2m),
        .ram_mask(data_mask),
        .ram_addr(data_addr),
        .ram_we(data_we),
        .ram_re(data_re),
        .ram_ready(data_ready)
    );

    reg ram_wb;
    reg [63:0] ram_word;
    reg [27:0] ram_addr;
    always @(posedge clk) begin
        if (rst) begin
            data_ready <= 1;
            ram_wb = 0;
        end else if (data_re) begin
            data_ready <= 0;
            data_m2c <= ram[data_addr];
            $display("read %x", data_addr);
        end else if (data_we) begin
            $display("write %x: %x", data_addr, data_c2m);
            data_ready <= 0;
            ram_wb <= 1;
            ram_addr <= data_addr;
        end else if (ram_wb) begin
            ram[ram_addr] <= ram_word;
            data_ready <= 1;
        end else begin
            data_ready <= 1;
        end
    end

    genvar mask_i;
    for (mask_i = 0; mask_i < 8; mask_i = mask_i + 1) generate
        always @(posedge clk) begin
            if (data_we) begin
                if (data_mask[mask_i]) begin
                    ram_word[mask_i * 8 + 7:mask_i * 8]
                        = data_c2m[mask_i * 8 + 7:mask_i * 8];
                end else begin
                    ram_word[mask_i * 8 + 7:mask_i * 8]
                        = ram[data_addr][mask_i * 8 + 7:mask_i * 8];
                end
            end
        end
    endgenerate

endmodule
