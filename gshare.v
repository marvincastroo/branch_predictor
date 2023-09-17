/* Universidad de Costa Rica
   Escuela de Ingeniería Eléctrica | Estructuras de Computadoras II - IE0521
   Profesor: Gerardo Castro Jiménez| 
   Estudiantes: 
   Andrés Chaves Vargas - Carné: B92198 | Kevin Delgado Rojas - Carné: B82566
   Marvin Castro Castro - Carné: C01884 | Gabriel Briceño Cambronero - Carné: C11262
   Módulo Predictor de Saltos Gshare
*/

module Gshare(
            input clk, reset,
            input branch, 
            input [10:0] addr, // Dirección 
            output reg prediction); //Salida de la XOR al PHT

reg [3:0] ghr; //Global History Register
reg [3:0] prediction_table [10:0]; //tabla de 10 filas con 2 bits cada una
reg [3:0] prediction_value, Prox_prediction_value, an_prediction_value;
reg [3:0] index, an_index;

parameter WEAK_NOT_TAKEN = 3'b000;
parameter WEAK_TAKEN = 3'b001;
parameter STRONG_NOT_TAKEN = 3'b010;
parameter STRONG_TAKEN = 3'b100;

always @(posedge clk) begin 
   
   if (reset) begin 
      ghr = 0;
      index = 0;
      an_index = 0;
      prediction_table[index] = WEAK_NOT_TAKEN;
      end 

   else begin
      index = ghr ^ addr;
      
      ghr <= {(ghr << 1), branch};
      
      prediction_table[index] <= Prox_prediction_value;
      an_prediction_value = prediction_table[index -1];
      an_index = an_index+1;
      end    
end

always @(*) begin //Revisar si no es solamente ghr 

   case (prediction_table[index]) 
   WEAK_NOT_TAKEN: begin
      prediction_value <= prediction_table[index];
      prediction = 0;
      if (branch == 1) Prox_prediction_value = STRONG_TAKEN;
      else Prox_prediction_value = STRONG_NOT_TAKEN;
   end

   WEAK_TAKEN: begin
      prediction_value <= prediction_table[index];
      prediction = 1;
      if (branch == 1) Prox_prediction_value = STRONG_TAKEN;
      else Prox_prediction_value = STRONG_NOT_TAKEN;
   end

   STRONG_NOT_TAKEN: begin
      prediction_value <= prediction_table[index];
      prediction = 0;
      if (branch == 1) Prox_prediction_value = WEAK_NOT_TAKEN;
      else Prox_prediction_value = STRONG_NOT_TAKEN;
   end

   STRONG_TAKEN: begin
      prediction_value <= prediction_table[index];
      prediction = 1;
      if (branch == 1) Prox_prediction_value = STRONG_TAKEN;
      else Prox_prediction_value = WEAK_TAKEN;
   end
   endcase

end

endmodule