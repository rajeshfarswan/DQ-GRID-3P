//***********************************************************************************//
//******************GRID CONNECTED THREE PHASE INVERTER******************************// 
//*****************BASED ON SINE PWM USING DQ CONTROL********************************//
//***********************************************************************************//

#include "main.h"    // Include file containing processor registors definitions and function definitions
#include "user.h"    // Include definitions of local variables
#include "asmMath.h" // Include definition of math functions 

int main(void) //main
{

init();     // call processor initilisation code

starting(); // Before-PWM initiliasation and soft-start of system

  while(1) //main loop 
	{

//current control//
    if(current_Flag) //20Khz
       {     
           //read inverter currents
          asm("disi #0x3FFF");
           {
            Avalue = asmADC(0x0808) - offset; //adc channel 8 
            Bvalue = asmADC(0x0909) - offset; //adc channel 9
            Cvalue = asmADC(0x0A0A) - offset; //adc channel 10
           } 
          asm("disi #0x0000");
           //
           //detect peak current set if fault
          if(Avalue > current_max) fault_Flag = 1; 
          if(Bvalue > current_max) fault_Flag = 1;
          if(Cvalue > current_max) fault_Flag = 1;  

          if(Avalue < current_min) fault_Flag = 1;  
          if(Bvalue < current_min) fault_Flag = 1;
          if(Cvalue < current_min) fault_Flag = 1; 
           // 

			asmABCtoDQ(); //converter feedback currents a-b-c to dq frame

            //current D PI
            IPreError = currentP_Dout;
            currentP_Dout = asmPIcontroller(current_Dref,Dvalue,current_Pgain,current_Igain);

            //current D PI filter
            FOF_PreOut = I_DFOFout;
            I_DFOFout = asmFO_Filter(currentP_Dout,Filter_const_I);

            Dvalue = I_DFOFout + ffd_FOFout; // add dc-link voltage feedforward feedback

            if(Dvalue >= I_max) Dvalue = I_max; //clamp
            if(Dvalue <= I_min) Dvalue = I_min;
       
            //current Q PI
            IPreError = currentP_Qout;
            currentP_Qout = asmPIcontroller(0,Qvalue,current_Pgain,current_Igain);

            //current Q PI filter
            FOF_PreOut = I_QFOFout;
            I_QFOFout = asmFO_Filter(currentP_Qout,Filter_const_I);

            Qvalue = I_QFOFout; //copy Q value

            if(Qvalue >= I_max) Qvalue = I_max; //clamp
            if(Qvalue <= I_min) Qvalue = I_min;
      
            asmDQtoABC(); //current d-q PI output to three phase refs

            asmPWM(); //generate three phase duty cycle           
            
                               current_Flag = 0;  //reset flag  
                                  } 
//current control//

//D-Q PLL//        
	if(pll_Flag) //1.3Khz
		{
          asm("disi #0x3FFF"); //read grid voltages  
           {
          Avalue = asmADC(0x0505) - offset; //adc channel 5 
          Bvalue = asmADC(0x0606) - offset; //adc channel 6    
          Cvalue = asmADC(0x0707) - offset; //adc channel 7
            } 
          asm("disi #0x0000");
          //

          asmABCtoDQ(); //three phase to d-q voltage feedbacks

          Vgrid_Dvalue = Dvalue;   //copy grid output voltage peak value

          FOF_PreOut = Osc_FOFout; //filter voltage q value
          Osc_FOFout = asmFO_Filter(Qvalue,Filter_const_P);
          
         /*PLL PI************************************************************/
          IPreError = oscPI_out;
          oscPI_out = asmPIcontroller(0,Osc_FOFout,oscPI_Pgain,oscPI_Igain); //set voltage q value to zero
          oscPI_out = asmINT_MPQ(OSC_Fcentral,oscPI_out); //generate sin cos ref

          if(oscPI_out >= OSC_Fmax) oscPI_out = OSC_Fmax;
          if(oscPI_out <= OSC_Fmin) oscPI_out = OSC_Fmin;
         /*PLL PI************************************************************/ 

				         pll_Flag = 0; //reset flag
						  }
//PLL//	


//feed forward and soft start//
		if(ffd_Flag) //0.5Khz   
      		{  
              asmDClink(); //monitor dc link and generate feedforward
              
              //ffd filter
              FOF_PreOut = ffd_FOFout;
              ffd_FOFout = asmFO_Filter(ffd_value,Filter_const_F);

                            //soft start and PLL sync delay
                            sync_tick++;

                            if((!sync_flag) && (sync_tick >= _300ms_count)) 
                                   {  
                                       if(ffd_value >= ffd_max) fault_Flag = 0;
                                       if(ffd_value <= ffd_min) fault_Flag = 0;

                                       if(ffd_value == 0) fault_Flag = 0; 
           
                                       SET = 0x0077; //all three switces are enabled
                                       sync_flag = 1; //reset sync flag 
                                       sync_tick = 0; //reset flag
                                    }   

                           if(sync_flag) //if PLL sync done
                                   {   
                                       if(ffd_value >= ffd_max) fault_Flag = 1; //set fault if feedforward is high or low or zero
                                       if(ffd_value <= ffd_min) fault_Flag = 1;

                                       if(ffd_value == 0) fault_Flag = 1; 

                                       current_Dref--; //initiate soft start //update grid current ref

                                           if(current_Dref <= current_Dsetpoint)
                                               { 
                                                 current_Dref = current_Dsetpoint; //set max grid current ref
                           
                                                   }
                                       sync_tick = 0; //reset flag
                                       }
                             //soft start and sync delay  
                
       		                    ffd_Flag = 0; //reset flag
       							}
//feed forward and soft start//

    			ClrWdt();
    		}//while end////////////////////////////////////////////////////////

  
		ClrWdt();
	} //main end////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////


//T1 interrupt for oscillator tracking
		void _ISRFAST __attribute__((interrupt, no_auto_psv)) _T1Interrupt(void) //99.88Khz
  			{   
                  //harmonic oscillator for generating Sine and Cos ref. 
                  OSC_F =  oscPI_out  + OSC_Fcentral; //determine angular frequency
                 
                  theta = theta + OSC_F;              //increment theta angle
     
         		    if(theta >= theta_2PI)           //reset harmonic oscillator
            		    {
           				qSin = 0;
           				qCos = 32440; //0.99
           				theta = 0;
              				}
              		else 
                    asmHARMONIC();                   //harmonic oscillator

     			T1us_Flag = 0; //reset flag
                
   					} //T1 interupt end

///////////////////////////////////////////////////////////////////////

		//fault interrupt
		void _ISR __attribute__((interrupt, no_auto_psv)) _FLTBInterrupt(void)
  			 {
     			PWMenable = 0; //disable pwm if fault
     			SET = 0;       //all switches off
          
     			RL2_ON = 0;    //open all relays 
               
     			RL3_ON = 0;      
     			RL4_ON = 0; 
     			RL5_ON = 0;     
  
		fault_Flag = 0;  //reset flag
            
   			}//fault end

//////////////////////////////////////////////////////////////////////

			//initial startup routine before turning on PWM
			void starting(void)
  				{
                    PWM_offset = PWM_PERIOD; //initialise PWM period value
                    PWM_offset_N = -PWM_PERIOD;

					PWM_max = PWM_offset*8; //PI saturation values
					PWM_min = -PWM_offset*8;
					SET = 0; //initialise PWM control registers
					PWMenable = 0; //reset PWM control register
					 //
					FAULT_ENABLE = 1; //0x000f; //reset fault register
					delay(30); //delay 30ms
					ADC_ON = 1;
					//precharging init
					RL1_ON = 1;  //precharging enable
					delay(150); //delay 1500ms
					//precharging init ends
					
					offset = asmADC(0x0e0e); //2.5V offset //read adc channel 14
					//
					//initiates startup
					RL1_ON = 0;  //precharging disable
					delay(30); //delay 30ms
					RL2_ON = 1;  //bypass precharging
					delay(30); //delay 30ms
					
					//set pwm
					PWM1 = PWM_offset;
					PWM2 = PWM_offset;
					PWM3 = PWM_offset;
					//SET = 0x0077; //all three switces are enabled
					//
					RL3_ON = 1;  //connect to grid
					delay(20); //delay 30ms
					RL4_ON = 1;
					delay(20); //delay 30ms
					RL5_ON = 1;
					
					delay(30); //delay 30ms
					//PWM_InterruptEnable = 0;
					PWMenable = 1; //enable all pwm and timers
					T1ON = 1;
                    T2ON = 1;
                    T3ON = 1;
                    T4ON = 1;
                    T5ON = 0;
                    
					// 
					  	}//startup routine end

///////////////////////////////////////////////////////////////////////

			











