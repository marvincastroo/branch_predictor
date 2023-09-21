/* Universidad de Costa Rica
   Escuela de Ingeniería Eléctrica | Estructuras de Computadoras II - IE0521
   Profesor: Gerardo Castro Jiménez| 
   Estudiantes: 
   Andrés Chaves Vargas - Carné: B92198 | Kevin Delgado Rojas - Carné: B82566
   Marvin Castro Castro - Carné: C01884 | Gabriel Briceño Cambronero - Carné: C11262
   Módulo gshare - testbench
*/

`include "gshare.v"
`include "gshare_tester.v"
module gshare_tb; 

wire wclk, wreset, wbranch; 
wire [10:0] waddr; 
wire wprediction;

initial begin
	$dumpfile("gshare.vcd");
	$dumpvars(-1, U0);
	//$monitor ("BALANCE ACTUALIZADO=%d, ENTREGAR DINERO=%d, FONDOS INSUFICIENTES=%d, PIN INCORRECTO=%d", wBALANCE_ACTUALIZADO, wENTREGAR_DINERO);
end

Gshare U0 (.clk(wclk), .reset(wreset), .branch(wbranch), 
           .addr(waddr), .prediction(wprediction));

Gshare_tester P0 (.clk(wclk), .reset(wreset), .branch(wbranch), 
           .addr(waddr), .prediction(wprediction));
endmodule
