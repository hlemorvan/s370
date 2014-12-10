
module multiplier(
    input wire clk,
    input wire start,
    input wire [31:0] ina,
    input wire [31:0] inb,
    output reg [63:0] result,
    output reg ready
);

    reg [5:0] count;
    reg [31:0] temp;

    always @(posedge clk) begin
        if (start) begin
            count  <= 32;
            result <= 0;
            temp   <= inb;
            ready  <= 0;
        end else if (count != 0) begin
            if (temp[0] == 1) begin
                result <= result + ina;
            end else begin
                result <= result;
            end
            temp <= {1'b0, temp[31:1]};
            count <= count - 1;
        end else begin
            ready <= 1;
        end
    end

endmodule
