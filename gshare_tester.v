/* Universidad de Costa Rica
   Escuela de Ingeniería Eléctrica | Estructuras de Computadoras II - IE0521
   Profesor: Gerardo Castro Jiménez| 
   Estudiantes: 
   Andrés Chaves Vargas - Carné: B92198 | Kevin Delgado Rojas - Carné: B82566
   Marvin Castro Castro - Carné: C01884 | Gabriel Briceño Cambronero - Carné: C11262
   Módulo Predictor de Saltos Gshare
*/

module Gshare_tester(
            output reg  clk, reset,
            output reg branch, 
            output reg [10:0] addr, // Dirección 
            input prediction); //Salida de la XOR al PHT

reg [3:0] ghr; //Global History Register
reg [3:0] prediction_table [10:0]; //tabla de 10 filas con 2 bits cada una
reg [3:0] Prox_prediction_value;
reg [3:0] index;

parameter WEAK_NOT_TAKEN = 3'b000;


initial begin


//Valores Iniciales de la Simulación:

clk = 0; reset = 1; branch = 0; addr = 0;
ghr = 0; prediction_table[index] = 0; index = 0;

//Prueba 1: Proceso Normal de Funcionamiento. Acción: Depósito.
#10 reset = 0;
   addr = 10'b0000;
   branch = 1; 
#10 branch = 0;

#10 branch = 1;

#10 branch = 0;

#30 branch = 1;
#200 $finish;
end 

always begin
    #5 clk = !clk;
end
endmodule