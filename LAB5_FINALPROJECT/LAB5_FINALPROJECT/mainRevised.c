	while(1){
		if(opt1Flag){ //triggered on a rising edge for an active low signal (i.e. when the object has just passed optical sensor 1)
			opt1Flag=0x00; //reset flag
			OI_Count+=1; //add one to amount of objects unsorted	
		} 
		if(opt2Flag){
			opt2Flag=0x00; //reset flag
			OR_Count+=1;
			ADCSRA |= _BV(ADSC); //initialize an ADC conversion
			startMeasureFlag=0x01;//allow ADC conversions to continue
		}
		if(optExitFlag){ //object has hit sensor at end of conveyor
			optExitFlag=0x00; //reset flag
			//corresponding positions (black=0;aluminum=50;white=100;steel=150)
			//if object type matches stepper location; do nothing...
			stepperMovement=stepperPosition-materialArray[EX_Count].type;
			if (stepperMovement!=0){//if object type doesn't match stepper location; stop motor, move stepper, start motor
				PORTB &=0xF0; //Apply Vcc brake to motor
				//stepper rotation logic
				if (stepperMovement==150) stepperMovement=-50;
				else if (stepperMovement==-150) stepperMovement=50;
				else if (stepperMovement== 100) stepperMovement=-100; //counter-clockwise is more efficient for particular stepper
				stepperControl(stepperMovement, &stepperPosition, &stepperIteration);//rotate stepper to proper location
				PORTB |=0b00001000; //start motor forwards
			}		
			EX_Count+=1;
		}
		if((ADCResultFlag) && (startMeasureFlag){ //if an ADC conversion is complete
			ADCResultFlag=0; //reset flag to allow interrupt to be triggered right away if necessary		
			if(ADCResult>(oldADCResult+0x0A)) oldADCResult=ADCResult; //reflectivity is increasing still (buffer implemented of 10(0x0A))
			else if((ADCResult+0x3B)<oldADCResult){ //reflectivities have been reducing and are 59(0x3B) lower than maximum reflectivity reached(buffer)
				materialArray[RL_Count].reflectance=oldADCResult;//value of oldADCResult is the maximum possible reflectivityis added to struct array
				tempFerrous=tempIndArray[RL_Count];
				materialArray[RL_Count].inductive=tempFerrous;//inductivity of material stored; 1 for inductive; 0 for non-ferrous
				if(tempFerrous){ //object is metal: aluminum (light), steel (dark)
					if (oldADCResult>AL_REFLECTIVITY) materialArray[RL_Count].type=150;//object is aluminium
					else materialArray[RL_Count].type=50;//object is steel 			
				} else { //object is plastic: white (light), black (dark)
					if (oldADCResult>WH_REFLECTIVITY) materialArray[RL_Count].type=100;//object is white plastic	
					else materialArray[RL_Count].type=0;//object is black plastic						
				}
				RL_Count+=1;//add one to amount of objects that have had their reflectivities measured
				oldADCResult=0x00;//reset oldADCResult to 0 for the next objects reflectivites to be measured
				startMeasureFlag=0x00; //set flag to zero so ADC conversions cannot occur
			}
			ADCSRA |= _BV(ADSC); //re-trigger ADC
		} else ADCResultFlag=0;
		if (inductiveFlag){ //triggered on a falling edge when a ferrous material is in front of inductive sensor
			inductiveFlag=0x00; //reset flag
			tempIndArray[OI_Count]=0x01;//add inductiveFlag = 1 to temporary array based on OI_Count
		}
	//efficient modulus for counters; forces them to stay within 0->63 as struct array only has 64 places
	OI_Count &= 0b00111111;
	RL_Count &= 0b00111111;
	OR_Count &= 0b00111111;
	EX_Count &= 0b00111111;
	}