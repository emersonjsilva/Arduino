/*-----------------------------------------------------------------------------/
 Project:         Automatic moving for robotic arm with servo SG90
 Date:            22/08/2021
 Programmers:     Emerson J. Silva, Elian P. S. Almeida
 Contact:         eng.emersonjsilva@gmail.com, elianprimo234@gmail.com

 Version:         1.0
 License:         GLP
 Aditional files: Axis.h, Axis.cpp, util.h 

 STRUCTURES:
 -----------
  TimerOn -> Struct to count time without interrupt the program
    Methods:
      timerOn -> Count time and return 1 when time bigger preset
        Params:
          int preset    -> Preset time in milliseconds
          int condition -> Contition to start de time count
        Return:
          0: if count < preset ou desable counting
          1: if count > preset and enable counting      
      timerOnONS -> Count time and return 1 when time bigger preset for one program scan
        Params:
          int preset    -> Preset time in milliseconds
          int condition -> Contition to start de time count
        Return:
          0: if count < preset ou desable counting
          1: if count > preset and enable counting         
  Filter -> Filter os signal of analog input
    Methods:
      filter -> Function to filter analog input
        params: float inpt -> Analog input to filter
        return: float outpt -> Analog input filtered
  
 FUNCTIONS:
 ----------
  splitString__ -> split a string with separator and index
    Params:
      String str_ -> Input string
      char sep    -> Char separator
      int index   -> Index of substring
    Return:
      String substring of index
            
/----------------------------------------------------------------------------*/

#include "Arduino.h"

// Temporizer //
typedef struct{
	unsigned long actual_;
	unsigned long acc = -1;
	int ONS = 0;

	int timerOn(unsigned int preset, int condition)
	{
		if(condition > 0 ){
			unsigned long actual_ = millis();
			if(acc <= 0) acc = actual_;

			if((actual_ - acc) > preset)
			{
				return 1;    
			}
			return 0;
		}
		else{
			acc = -1;
			ONS = 0;
			return 0;
		}
	}
	
	int timerOnONS(unsigned int preset, int condition){
		
		if(timerOn(preset, condition) == 1 && ONS == 0){
			ONS = 1;
			return 1;
		}
		return 0;	
	}
	
	void resetTimer() {
    ONS = 0;
	  acc = -1;
	  }
} TimerOn;

typedef struct{
	float outpt = 0.0;
	float k = 0.99999999;
	int init = 0;
	
	float filter(float inpt){
		if(init == 0){
			outpt = inpt;
			init = 1;
		}		
		outpt = ( inpt * (1.0 - k) + outpt * k);
		return outpt;	
	}
} Filter;

// Based on: https://qastack.com.br/arduino/1013/how-do-i-split-an-incoming-string
String splitString__(String str_, char sep, int index)
{
    int found = 0;
    int strStart = 0;
    int strEnd = -1;
    int maxStr = str_.length() - 1;

    for (int i = 0; i <= maxStr && found <= index; i++) {
        if (str_.charAt(i) == sep || i == maxStr) {
            found++;
            strStart = strEnd + 1;
            strEnd = (i == maxStr) ? i+1 : i;
        }
    }
    return found > index ? str_.substring(strStart, strEnd) : "";
}
