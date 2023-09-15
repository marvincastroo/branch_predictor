/* Universidad de Costa Rica
   Escuela de Ingeniería Eléctrica | Estructuras de Computadoras II - IE0521
   Profesor: Gerardo Castro Jiménez| 
   Estudiantes: 
   Andrés Chaves Vargas - Carné: B92198 | Kevin Delgado Rojas - Carné: B82566
   Marvin Castro Castro - Carné: C01884 | Gabriel Briceño Cambronero - Carné: C11262
   Módulo Predictor de Saltos Gshare
*/

module Gshare(
            input reg clk, reset,
            input reg branch, 
            input reg [31:0] addr, // Dirección 
            output reg [3:0] index_send); //Salida de la XOR al PHT

reg [3:0] ghr; //Global History Register

always @(posedge clk) begin 
   
   if (reset) ghr = 0;

   else ghr = (ghr << 1) + branch; 
end

always @(*) begin //Revisar si no es solamente ghr 

    index_send <= ghr ^ addr; 
end
endmodule