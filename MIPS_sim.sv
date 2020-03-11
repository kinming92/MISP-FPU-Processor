module MIPS_sim();

logic clk, rst, y;

MIPS_14 dut(clk, rst, y);

always begin
    #5 clk=~clk;
end

initial begin
    #10 clk=1'b1;
    #10 rst=1'b1;
    #10 rst=1'b0;
end
endmodule 
