
module tb_bp_2bit;

reg clk, reset, branch;
wire reg state;

initial begin
    $dumpfile("bp_2bit.vcd");
    $dumpvars(0, tb_bp_2bit);
end

bp_2bit uut(.clk(clk), .reset(reset), .branch(branch), .state(state));

initial begin
    reset = 0; #5
    reset = 1;

    branch = 0; #5
    branch = 0; #5
    branch = 1; #5
    branch = 1; #5
    branch = 0; #5
    branch = 1; #5
    branch = 0; #5
    branch = 1; #5
    branch = 1; #5
    branch = 1; #5
    branch = 0; #5
    branch = 0; #5
    branch = 1; #5
    branch = 0; #5
    branch = 1; #5



    #20
    $finish;
end

always begin
    clk = 1; #5;
    clk = 0; #5;
end

endmodule