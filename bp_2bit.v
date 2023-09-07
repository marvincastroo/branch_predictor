// Modulo del predictor de saltos de dos bits visto en clases
// branch = 1 es TAKEN
// branch = 0 es NOT TAKEN
// el estado inicial del BP es WEAK NOT TAKEN

module bp_2bit(clk, reset, branch, state);
    input clk, reset, branch;
    output reg state;

    reg [3:0] EstPres, ProxEst;

    parameter WEAK_NOT_TAKEN = 3'b000;
    parameter WEAK_TAKEN = 3'b001;
    parameter STRONG_NOT_TAKEN = 3'b010;
    parameter STRONG_TAKEN = 3'b100;

    always @(posedge clk, posedge reset) begin
        
        if (!reset) begin
            EstPres <= WEAK_NOT_TAKEN;
        end

        else begin
            EstPres <= ProxEst;
        end
    end 

    always @(*)
    case (EstPres) 
        WEAK_NOT_TAKEN: begin
            state = 0;
            if (branch == 1) ProxEst = STRONG_TAKEN;
            else ProxEst = STRONG_NOT_TAKEN;
        end

        WEAK_TAKEN: begin
            state = 1;
            if (branch == 1) ProxEst = STRONG_TAKEN;
            else ProxEst = STRONG_NOT_TAKEN;
        end

        STRONG_NOT_TAKEN: begin
            state = 0;
            if (branch == 1) ProxEst = WEAK_NOT_TAKEN;
            else ProxEst = STRONG_NOT_TAKEN;
        end

        STRONG_TAKEN: begin
            state = 1;
            if (branch == 1) ProxEst = STRONG_TAKEN;
            else ProxEst = WEAK_NOT_TAKEN;
        end
    endcase

endmodule

         
    
    
    
