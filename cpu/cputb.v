
`include "cpu.v"

module cputb;

    integer i;
    reg clk;
    reg rst;
    reg [63:0] data_m2c;
    wire [63:0] data_c2m;
    wire [7:0] data_mask;
    wire [27:0] data_addr;
    wire data_re;
    wire data_we;
    reg data_ready;

    initial begin

        $dumpvars;

        clk <= 0;
        rst <= 1;
        #10 clk <= 1;
        #10 clk <= 0;
        rst <= 0;

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

    reg [63:0] ram [1:256];
    reg ram_wb;
    reg [63:0] ram_word;
    reg [27:0] ram_addr;
    always @(posedge clk) begin
        if (rst) begin
            data_ready <= 1;
            ram_wb = 0;
        end else if (data_re) begin
            data_ready <= 0;
            case (data_addr)
                0:          data_m2c <= 64'h1E12_1423_0000_0000;
                default:    data_m2c <= ram[data_addr];
            endcase
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

    genvar j;
    for (j = 0; j < 8; j = j + 1) generate
        always @(posedge clk) begin
            if (data_we) begin
                if (data_mask[j]) begin
                    ram_word[j * 8 + 7:j * 8] = data_c2m[j * 8 + 7:j * 8];
                end else begin
                    ram_word[j * 8 + 7:j * 8] = ram[data_addr][j * 8 + 7:j * 8];
                end
            end
        end
    endgenerate

endmodule
