/* Arduino Worm Bot is a adaptation of python code of Tim Busbice for control a robot simulating the connectome of a C. Elegans worm, by Fernando Vidal Olmos 

Original source code: http://www.connectomeengine.com/Home/Downloads
this code is an adaption of the code of Tim Busbice and the code of Fernando Vidal Ramos
Author: Rony David Carías Vidal
Date: 2016-08-06
Universidad Mariano Gálvez de Guatemala
*/

/* neural list identification */
#include <avr/pgmspace.h>
#include "DefineNeuronas.h"
#include "ValoresConectoma.h"

/* Arduino pin conections */
#define IN1 5
#define IN2 4
#define IN3 3
#define IN4 2
#define ENA 6
#define ENB 7
#define TRIG 11
#define ECHO 12
#define SOUT 10
#define LED 13

/* Global specifications */
#define THRESHOLD 30
#define MUSCLE_CM_ESTIMULATE 20


/* Motor function mode */
#define SPEED 1
#define RIGHT_SPEED 2
#define LEFT_SPEED 3
#define STOP 4
#define LEFT_STOP 5
#define RIGHT_STOP 6
#define RIGHT_FWD 7
#define RIGHT_BWD 8
#define LEFT_FWD 9
#define LEFT_BWD 10
#define RIGHT_ROT 11
#define LEFT_ROT 12
#define BWD 13
#define FWD 14

long dist; //distancia medida por el sensor de ultrasonido
long time;//tiempo para medicion del ultrasonido
long Time; //tiempo desde envio de información
String inString = "";    // cadena de entrada desde bluetooth
float turnratio;
int postsynaptic[397][1];
int accumleft = 0;
int accumright = 0;
int new_speed = 0;
int tope=0;
int PIR=10; // Pin del Sensor PIR
boolean EnvioBluetooth;
boolean thisState = 0;
boolean nextState = 1;


//******creación de conectoma*****
void createpostsynaptic()
{
  for(int i=0; i<397; i++){
      resetSynapticNeuron(i);
                      }
}
//********exitación de neurona**********
void postSynapticNeuron(int neuron, int sum)
{
        postsynaptic[neuron][nextState] = postsynaptic[neuron][thisState] += sum;
}

//*****reset de sinapsis de las neuronas*****                         
void resetSynapticNeuron(int neuron)
{
        postsynaptic[neuron][thisState] = 0;
        postsynaptic[neuron][nextState] = 0;
}
//******propagación de neurona**********
//ForNeuron(array_neurona, array_pesos, tope_for);
void ForNeuron(prog_int16_t Neuron[],prog_int16_t valores[], int tope)
{   
      for(int i=0; i<tope; i++)
      {
        postSynapticNeuron(pgm_read_word_near(&Neuron[i]), pgm_read_word_near(&valores[i]));
       }  
}
//*********activación de neurona************
void dendriteAccumulate(int dneuron)
{
  switch(dneuron) 
  {
    case N_ADAL: ForNeuron(NeuronADAL,AccuADAL,(sizeof(NeuronADAL)/sizeof(int))); break;
    case N_ADAR: ForNeuron(NeuronADAR,AccuADAR,(sizeof(NeuronADAR)/sizeof(int))); break;
    case N_ADEL: ForNeuron(NeuronADEL,AccuADEL,(sizeof(NeuronADEL)/sizeof(int))); break;
    case N_ADER: ForNeuron(NeuronADER,AccuADER,(sizeof(NeuronADER)/sizeof(int))); break;
    case N_ADFL: ForNeuron(NeuronADFL,AccuADFL,(sizeof(NeuronADFL)/sizeof(int)));  break;
    case N_ADFR: ForNeuron(NeuronADFR,AccuADFR,(sizeof(NeuronADFR)/sizeof(int)));  break;
    case N_ADLL: ForNeuron(NeuronADLL,AccuADLL,(sizeof(NeuronADLL)/sizeof(int))); break;
    case N_ADLR: ForNeuron(NeuronADLR,AccuADLR,(sizeof(NeuronADLR)/sizeof(int)));  break;
    case N_AFDL: ForNeuron(NeuronAFDL,AccuAFDL,(sizeof(NeuronAFDL)/sizeof(int))); break;
    case N_AFDR: ForNeuron(NeuronAFDR,AccuAFDR,(sizeof(NeuronAFDR)/sizeof(int))); break;//10
    case N_AIAL: ForNeuron(NeuronAIAL,AccuAIAL,(sizeof(NeuronAIAL)/sizeof(int)));  break;
    case N_AIAR: ForNeuron(NeuronAIAR,AccuAIAR,(sizeof(NeuronAIAR)/sizeof(int)));  break;
    case N_AIBL: ForNeuron(NeuronAIBL,AccuAIBL,(sizeof(NeuronAIBL)/sizeof(int)));  break;
    case N_AIBR: ForNeuron(NeuronAIBR,AccuAIBR,(sizeof(NeuronAIBR)/sizeof(int)));   break;
    case N_AIML: ForNeuron(NeuronAIML,AccuAIML,(sizeof(NeuronAIML)/sizeof(int))); break;
    case N_AIMR: ForNeuron(NeuronAIMR,AccuAIMR,(sizeof(NeuronAIMR)/sizeof(int)));   break;
    case N_AINL: ForNeuron(NeuronAINL,AccuAINL,(sizeof(NeuronAINL)/sizeof(int)));   break;
    case N_AINR: ForNeuron(NeuronAINR,AccuAINR,(sizeof(NeuronAINR)/sizeof(int))); break;
    case N_AIYL: ForNeuron(NeuronAIYL,AccuAIYL,(sizeof(NeuronAIYL)/sizeof(int))); break;
    case N_AIYR: ForNeuron(NeuronAIYR,AccuAIYR,(sizeof(NeuronAIYR)/sizeof(int)));  break;//20
    case N_AIZL: ForNeuron(NeuronAIZL,AccuAIZL,(sizeof(NeuronAIZL)/sizeof(int))); break;
    case N_AIZR: ForNeuron(NeuronAIZR,AccuAIZR,(sizeof(NeuronAIZR)/sizeof(int)));  break;
    case N_ALA: ForNeuron(NeuronALA,AccuALA,(sizeof(NeuronALA)/sizeof(int)));  break;
    case N_ALML: ForNeuron(NeuronALML,AccuALML,(sizeof(NeuronALML)/sizeof(int))); break;
    case N_ALMR: ForNeuron(NeuronALMR,AccuALMR,(sizeof(NeuronALMR)/sizeof(int)));  break;
    case N_ALNL: ForNeuron(NeuronALNL,AccuALNL,(sizeof(NeuronALNL)/sizeof(int))); break;
    case N_ALNR: ForNeuron(NeuronALNR,AccuALNR,(sizeof(NeuronALNR)/sizeof(int)));  break;
    case N_AQR: ForNeuron(NeuronAQR,AccuAQR,(sizeof(NeuronAQR)/sizeof(int))); break;
    case N_AS1: ForNeuron(NeuronAS1_,AccuAS1_,(sizeof(NeuronAS1_)/sizeof(int))); break;
    case N_AS2: ForNeuron(NeuronAS2_,AccuAS2_,(sizeof(NeuronAS2_)/sizeof(int)));  break;//30
    case N_AS3: ForNeuron(NeuronAS3_,AccuAS3_,(sizeof(NeuronAS3_)/sizeof(int)));  break;
    case N_AS4: ForNeuron(NeuronAS4_,AccuAS4_,(sizeof(NeuronAS4_)/sizeof(int)));  break;
    case N_AS5: ForNeuron(NeuronAS5_,AccuAS5_,(sizeof(NeuronAS5_)/sizeof(int))); break;
    case N_AS6: ForNeuron(NeuronAS6_,AccuAS6_,(sizeof(NeuronAS6_)/sizeof(int)));  break;
    case N_AS7: ForNeuron(NeuronAS7_,AccuAS7_,(sizeof(NeuronAS7_)/sizeof(int))); break;
    case N_AS8: ForNeuron(NeuronAS8_,AccuAS8_,(sizeof(NeuronAS8_)/sizeof(int))); break;
    case N_AS9: ForNeuron(NeuronAS9_,AccuAS9_,(sizeof(NeuronAS9_)/sizeof(int))); break;
    case N_AS10: ForNeuron(NeuronAS10_,AccuAS10_,(sizeof(NeuronAS10_)/sizeof(int))); break;
    case N_AS11: ForNeuron(NeuronAS11_,AccuAS11_,(sizeof(NeuronAS11_)/sizeof(int)));  break;
    case N_ASEL: ForNeuron(NeuronASEL,AccuASEL,(sizeof(NeuronASEL)/sizeof(int))); break;//40
    case N_ASER: ForNeuron(NeuronASER,AccuASER,(sizeof(NeuronASER)/sizeof(int))); break;
    case N_ASGL: ForNeuron(NeuronASGL,AccuASGL,(sizeof(NeuronASGL)/sizeof(int))); break;
    case N_ASGR: ForNeuron(NeuronASGR,AccuASGR,(sizeof(NeuronASGR)/sizeof(int)));   break;
    case N_ASHL: ForNeuron(NeuronASHL,AccuASHL,(sizeof(NeuronASHL)/sizeof(int))); break;
    case N_ASHR: ForNeuron(NeuronASHR,AccuASHR,(sizeof(NeuronASHR)/sizeof(int))); break;
    case N_ASIL: ForNeuron(NeuronASIL,AccuASIL,(sizeof(NeuronASIL)/sizeof(int))); break;
    case N_ASIR: ForNeuron(NeuronASIR,AccuASIR,(sizeof(NeuronASIR)/sizeof(int))); break;
    case N_ASJL: ForNeuron(NeuronASJL,AccuASJL,(sizeof(NeuronASJL)/sizeof(int))); break;
    case N_ASJR: ForNeuron(NeuronASJR,AccuASJR,(sizeof(NeuronASJR)/sizeof(int))); break;
    case N_ASKL: ForNeuron(NeuronASKL,AccuASKL,(sizeof(NeuronASKL)/sizeof(int))); break;//50    
    case N_ASKR: ForNeuron(NeuronASKR,AccuASKR,(sizeof(NeuronASKR)/sizeof(int))); break;
    case N_AUAL: ForNeuron(NeuronAUAL,AccuAUAL,(sizeof(NeuronAUAL)/sizeof(int))); break;
    case N_AUAR: ForNeuron(NeuronAUAR,AccuAUAR,(sizeof(NeuronAUAR)/sizeof(int))); break;
    case N_AVAL: ForNeuron(NeuronAVAL,AccuAVAL,(sizeof(NeuronAVAL)/sizeof(int))); break;
    case N_AVAR: ForNeuron(NeuronAVAR,AccuAVAR,(sizeof(NeuronAVAR)/sizeof(int)));  break; 
    case N_AVBL: ForNeuron(NeuronAVBL,AccuAVBL,(sizeof(NeuronAVBL)/sizeof(int))); break;
    case N_AVBR: ForNeuron(NeuronAVBR,AccuAVBR,(sizeof(NeuronAVBR)/sizeof(int))); break;
    case N_AVDL: ForNeuron(NeuronAVDL,AccuAVDL,(sizeof(NeuronAVDL)/sizeof(int))); break;
    case N_AVDR: ForNeuron(NeuronAVDR,AccuAVDR,(sizeof(NeuronAVDR)/sizeof(int))); break;
    case N_AVEL: ForNeuron(NeuronAVEL,AccuAVEL,(sizeof(NeuronAVEL)/sizeof(int))); break;//60  
    case N_AVER: ForNeuron(NeuronAVER,AccuAVER,(sizeof(NeuronAVER)/sizeof(int)));  break;
    case N_AVFL: ForNeuron(NeuronAVFL,AccuAVFL,(sizeof(NeuronAVFL)/sizeof(int))); break;
    case N_AVFR: ForNeuron(NeuronAVFR,AccuAVFR,(sizeof(NeuronAVFR)/sizeof(int)));  break;
    case N_AVG: ForNeuron(NeuronAVG,AccuAVG,(sizeof(NeuronAVG)/sizeof(int))); break;
    case N_AVHL: ForNeuron(NeuronAVHL,AccuAVHL,(sizeof(NeuronAVHL)/sizeof(int))); break;   
    case N_AVHR: ForNeuron(NeuronAVHR,AccuAVHR,(sizeof(NeuronAVHR)/sizeof(int)));  break;
    case N_AVJL: ForNeuron(NeuronAVJL,AccuAVJL,(sizeof(NeuronAVJL)/sizeof(int))); break;
    case N_AVJR: ForNeuron(NeuronAVJR,AccuAVJR,(sizeof(NeuronAVJR)/sizeof(int))); break;
    case N_AVKL: ForNeuron(NeuronAVKL,AccuAVKL,(sizeof(NeuronAVKL)/sizeof(int)));  break;
    case N_AVKR: ForNeuron(NeuronAVKR,AccuAVKR,(sizeof(NeuronAVKR)/sizeof(int)));  break;//70  
    case N_AVL: ForNeuron(NeuronAVL,AccuAVL,(sizeof(NeuronAVL)/sizeof(int))); break;
    case N_AVM: ForNeuron(NeuronAVM,AccuAVM,(sizeof(NeuronAVM)/sizeof(int))); break;
    case N_AWAL: ForNeuron(NeuronAWAL,AccuAWAL,(sizeof(NeuronAWAL)/sizeof(int))); break;
    case N_AWAR: ForNeuron(NeuronAWAR,AccuAWAR,(sizeof(NeuronAWAR)/sizeof(int)));  break;
    case N_AWBL: ForNeuron(NeuronAWBL,AccuAWBL,(sizeof(NeuronAWBL)/sizeof(int))); break;  
    case N_AWBR: ForNeuron(NeuronAWBR,AccuAWBR,(sizeof(NeuronAWBR)/sizeof(int))); break;
    case N_AWCL: ForNeuron(NeuronAWCL,AccuAWCL,(sizeof(NeuronAWCL)/sizeof(int))); break;
    case N_AWCR: ForNeuron(NeuronAWCR,AccuAWCR,(sizeof(NeuronAWCR)/sizeof(int))); break;
    case N_BAGL: ForNeuron(NeuronBAGL,AccuBAGL,(sizeof(NeuronBAGL)/sizeof(int))); break;
    case N_BAGR: ForNeuron(NeuronBAGR,AccuBAGR,(sizeof(NeuronBAGR)/sizeof(int))); break;//80           
    case N_BDUL: ForNeuron(NeuronBDUL,AccuBDUL,(sizeof(NeuronBDUL)/sizeof(int))); break;
    case N_BDUR: ForNeuron(NeuronBDUR,AccuBDUR,(sizeof(NeuronBDUR)/sizeof(int))); break;
    case N_CEPDL: ForNeuron(NeuronCEPDL,AccuCEPDL,(sizeof(NeuronCEPDL)/sizeof(int))); break;
    case N_CEPDR: ForNeuron(NeuronCEPDR,AccuCEPDR,(sizeof(NeuronCEPDR)/sizeof(int))); break;
    case N_CEPVL: ForNeuron(NeuronCEPVL,AccuCEPVL,(sizeof(NeuronCEPVL)/sizeof(int))); break;   
    case N_CEPVR: ForNeuron(NeuronCEPVR,AccuCEPVR,(sizeof(NeuronCEPVR)/sizeof(int))); break;
    case N_DA1: ForNeuron(NeuronDA1_,AccuDA1_,(sizeof(NeuronDA1_)/sizeof(int))); break;
    case N_DA2: ForNeuron(NeuronDA2_,AccuDA2_,(sizeof(NeuronDA2_)/sizeof(int))); break;
    case N_DA3: ForNeuron(NeuronDA3_,AccuDA3_,(sizeof(NeuronDA3_)/sizeof(int))); break;
    case N_DA4: ForNeuron(NeuronDA4_,AccuDA4_,(sizeof(NeuronDA4_)/sizeof(int))); break;//90   
    case N_DA5: ForNeuron(NeuronDA5_,AccuDA5_,(sizeof(NeuronDA5_)/sizeof(int)));  break;
    case N_DA6: ForNeuron(NeuronDA6_,AccuDA6_,(sizeof(NeuronDA6_)/sizeof(int))); break;
    case N_DA7: ForNeuron(NeuronDA7_,AccuDA7_,(sizeof(NeuronDA7_)/sizeof(int))); break;
    case N_DA8: ForNeuron(NeuronDA8_,AccuDA8_,(sizeof(NeuronDA8_)/sizeof(int)));  break;
    case N_DA9: ForNeuron(NeuronDA9_,AccuDA9_,(sizeof(NeuronDA9_)/sizeof(int)));  break;       
    case N_DB1: ForNeuron(NeuronDB1_,AccuDB1_,(sizeof(NeuronDB1_)/sizeof(int))); break;
    case N_DB2: ForNeuron(NeuronDB2_,AccuDB2_,(sizeof(NeuronDB2_)/sizeof(int))); break;
    case N_DB3: ForNeuron(NeuronDB3_,AccuDB3_,(sizeof(NeuronDB3_)/sizeof(int))); break;
    case N_DB4: ForNeuron(NeuronDB4_,AccuDB4_,(sizeof(NeuronDB4_)/sizeof(int))); break;
    case N_DB5: ForNeuron(NeuronDB5_,AccuDB5_,(sizeof(NeuronDB5_)/sizeof(int))); break;//100 
    case N_DB6: ForNeuron(NeuronDB6_,AccuDB6_,(sizeof(NeuronDB6_)/sizeof(int))); break;
    case N_DB7: ForNeuron(NeuronDB7_,AccuDB7_,(sizeof(NeuronDB7_)/sizeof(int)));  break;
    case N_DD1: ForNeuron(NeuronDD1_,AccuDD1_,(sizeof(NeuronDD1_)/sizeof(int))); break;
    case N_DD2: ForNeuron(NeuronDD2_,AccuDD2_,(sizeof(NeuronDD2_)/sizeof(int))); break;
    case N_DD3: ForNeuron(NeuronDD3_,AccuDD3_,(sizeof(NeuronDD3_)/sizeof(int)));  break;   
    case N_DD4: ForNeuron(NeuronDD4_,AccuDD4_,(sizeof(NeuronDD4_)/sizeof(int))); break;
    case N_DD5: ForNeuron(NeuronDD5_,AccuDD5_,(sizeof(NeuronDD5_)/sizeof(int))); break;
    case N_DD6: ForNeuron(NeuronDD6_,AccuDD6_,(sizeof(NeuronDD6_)/sizeof(int))); break;
    case N_DVA: ForNeuron(NeuronDVA,AccuDVA,(sizeof(NeuronDVA)/sizeof(int))); break;
    case N_DVB: ForNeuron(NeuronDVB,AccuDVB,(sizeof(NeuronDVB)/sizeof(int))); break;//110 
    case N_DVC: ForNeuron(NeuronDVC,AccuDVC,(sizeof(NeuronDVC)/sizeof(int))); break;
    case N_FLPL: ForNeuron(NeuronFLPL,AccuFLPL,(sizeof(NeuronFLPL)/sizeof(int))); break;
    case N_FLPR: ForNeuron(NeuronFLPR,AccuFLPR,(sizeof(NeuronFLPR)/sizeof(int))); break;
    case N_HSNL: ForNeuron(NeuronHSNL,AccuHSNL,(sizeof(NeuronHSNL)/sizeof(int))); break;
    case N_HSNR: ForNeuron(NeuronHSNR,AccuHSNR,(sizeof(NeuronHSNR)/sizeof(int))); break;
    case N_I1L: ForNeuron(NeuronI1L,AccuI1L,(sizeof(NeuronI1L)/sizeof(int))); break;
    case N_I1R: ForNeuron(NeuronI1R,AccuI1R,(sizeof(NeuronI1R)/sizeof(int))); break;
    case N_I2L: ForNeuron(NeuronI2L,AccuI2L,(sizeof(NeuronI2L)/sizeof(int))); break;
    case N_I2R: ForNeuron(NeuronI2R,AccuI2R,(sizeof(NeuronI2R)/sizeof(int))); break;
    case N_I3: ForNeuron(NeuronI3_,AccuI3_,(sizeof(NeuronI3_)/sizeof(int))); break;//120
    case N_I4: ForNeuron(NeuronI4_,AccuI4_,(sizeof(NeuronI4_)/sizeof(int))); break;
    case N_I5: ForNeuron(NeuronI5_,AccuI5_,(sizeof(NeuronI5_)/sizeof(int))); break;
    case N_I6: ForNeuron(NeuronI6_,AccuI6_,(sizeof(NeuronI6_)/sizeof(int))); break;
    case N_IL1DL: ForNeuron(NeuronIL1DL,AccuIL1DL,(sizeof(NeuronIL1DL)/sizeof(int))); break;
    case N_IL1DR: ForNeuron(NeuronIL1DR,AccuIL1DR,(sizeof(NeuronIL1DR)/sizeof(int))); break;
    case N_IL1L: ForNeuron(NeuronIL1L,AccuIL1L,(sizeof(NeuronIL1L)/sizeof(int))); break;
    case N_IL1R: ForNeuron(NeuronIL1R,AccuIL1R,(sizeof(NeuronIL1R)/sizeof(int))); break;
    case N_IL1VL: ForNeuron(NeuronIL1VL,AccuIL1VL,(sizeof(NeuronIL1VL)/sizeof(int))); break;
    case N_IL1VR: ForNeuron(NeuronIL1VR,AccuIL1VR,(sizeof(NeuronIL1VR)/sizeof(int))); break;
    case N_IL2DL: ForNeuron(NeuronIL2DL,AccuIL2DL,(sizeof(NeuronIL2DL)/sizeof(int))); break;//130  
    case N_IL2DR: ForNeuron(NeuronIL2DR,AccuIL2DR,(sizeof(NeuronIL2DR)/sizeof(int))); break;
    case N_IL2L: ForNeuron(NeuronIL2L,AccuIL2L,(sizeof(NeuronIL2L)/sizeof(int))); break;
    case N_IL2R: ForNeuron(NeuronIL2R,AccuIL2R,(sizeof(NeuronIL2R)/sizeof(int))); break;
    case N_IL2VL: ForNeuron(NeuronIL2VL,AccuIL2VL,(sizeof(NeuronIL2VL)/sizeof(int))); break;
    case N_IL2VR: ForNeuron(NeuronIL2VR,AccuIL2VR,(sizeof(NeuronIL2VR)/sizeof(int))); break; 
    case N_LUAL: ForNeuron(NeuronLUAL,AccuLUAL,(sizeof(NeuronLUAL)/sizeof(int))); break;
    case N_LUAR: ForNeuron(NeuronLUAR,AccuLUAR,(sizeof(NeuronLUAR)/sizeof(int))); break;
    case N_M1: ForNeuron(NeuronM1_,AccuM1_,(sizeof(NeuronM1_)/sizeof(int))); break;
    case N_M2L:  ForNeuron(NeuronM2L,AccuM2L,(sizeof(NeuronM2L)/sizeof(int))); break;
    case N_M2R: ForNeuron(NeuronM2R,AccuM2R,(sizeof(NeuronM2R)/sizeof(int))); break;//140  
    case N_M3L: ForNeuron(NeuronM3L,AccuM3L,(sizeof(NeuronM3L)/sizeof(int)));
    case N_M3R: ForNeuron(NeuronM3R,AccuM3R,(sizeof(NeuronM3R)/sizeof(int))); break;
    case N_M4: ForNeuron(NeuronM4_,AccuM4_,(sizeof(NeuronM4_)/sizeof(int))); break;
    case N_M5: ForNeuron(NeuronM5_,AccuM5_,(sizeof(NeuronM5_)/sizeof(int))); break;
    case N_MCL: ForNeuron(NeuronMCL,AccuMCL,(sizeof(NeuronMCL)/sizeof(int))); break;
    case N_MCR: ForNeuron(NeuronMCR,AccuMCR,(sizeof(NeuronMCR)/sizeof(int)));  break;
    case N_MI: ForNeuron(NeuronMI,AccuMI,(sizeof(NeuronMI)/sizeof(int))); break;
    case N_NSML: ForNeuron(NeuronNSML,AccuNSML,(sizeof(NeuronNSML)/sizeof(int)));  break;
    case N_NSMR: ForNeuron(NeuronNSMR,AccuNSMR,(sizeof(NeuronNSMR)/sizeof(int))); break;
    case N_OLLL: ForNeuron(NeuronOLLL,AccuOLLL,(sizeof(NeuronOLLL)/sizeof(int)));  break;//150
    case N_OLLR: ForNeuron(NeuronOLLR,AccuOLLR,(sizeof(NeuronOLLR)/sizeof(int))); break;
    case N_OLQDL: ForNeuron(NeuronOLQDL,AccuOLQDL,(sizeof(NeuronOLQDL)/sizeof(int))); break;
    case N_OLQDR: ForNeuron(NeuronOLQDR,AccuOLQDR,(sizeof(NeuronOLQDR)/sizeof(int))); break;
    case N_OLQVL: ForNeuron(NeuronOLQVL,AccuOLQVL,(sizeof(NeuronOLQVL)/sizeof(int))); break;
    case N_OLQVR: ForNeuron(NeuronOLQVR,AccuOLQVR,(sizeof(NeuronOLQVR)/sizeof(int))); break;
    case N_PDA: ForNeuron(NeuronPDA,AccuPDA,(sizeof(NeuronPDA)/sizeof(int))); break;
    case N_PDB: ForNeuron(NeuronPDB,AccuPDB,(sizeof(NeuronPDB)/sizeof(int))); break;
    case N_PDEL: ForNeuron(NeuronPDEL,AccuPDEL,(sizeof(NeuronPDEL)/sizeof(int))); break;
    case N_PDER: ForNeuron(NeuronPDER,AccuPDER,(sizeof(NeuronPDER)/sizeof(int))); break;
    case N_PHAL: ForNeuron(NeuronPHAL,AccuPHAL,(sizeof(NeuronPHAL)/sizeof(int))); break;//160
    case N_PHAR: ForNeuron(NeuronPHAR,AccuPHAR,(sizeof(NeuronPHAR)/sizeof(int))); break;
    case N_PHBL: ForNeuron(NeuronPHBL,AccuPHBL,(sizeof(NeuronPHBL)/sizeof(int))); break;
    case N_PHBR: ForNeuron(NeuronPHBR,AccuPHBR,(sizeof(NeuronPHBR)/sizeof(int))); break;
    case N_PHCL: ForNeuron(NeuronPHCL,AccuPHCL,(sizeof(NeuronPHCL)/sizeof(int))); break;
    case N_PHCR: ForNeuron(NeuronPHCR,AccuPHCR,(sizeof(NeuronPHCR)/sizeof(int))); break;
    case N_PLML: ForNeuron(NeuronPLML,AccuPLML,(sizeof(NeuronPLML)/sizeof(int))); break;
    case N_PLMR: ForNeuron(NeuronPLMR,AccuPLMR,(sizeof(NeuronPLMR)/sizeof(int))); break;
    case N_PLNL: ForNeuron(NeuronPLNL,AccuPLNL,(sizeof(NeuronPLNL)/sizeof(int))); break;
    case N_PLNR: ForNeuron(NeuronPLNR,AccuPLNR,(sizeof(NeuronPLNR)/sizeof(int))); break;
    case N_PQR: ForNeuron(NeuronPQR,AccuPQR,(sizeof(NeuronPQR)/sizeof(int))); break; //170
    case N_PVCL: ForNeuron(NeuronPVCL,AccuPVCL,(sizeof(NeuronPVCL)/sizeof(int))); break;
    case N_PVCR: ForNeuron(NeuronPVCR,AccuPVCR,(sizeof(NeuronPVCR)/sizeof(int))); break;
    case N_PVDL: ForNeuron(NeuronPVDL,AccuPVDL,(sizeof(NeuronPVDL)/sizeof(int))); break;
    case N_PVDR: ForNeuron(NeuronPVDR,AccuPVDR,(sizeof(NeuronPVDR)/sizeof(int))); break;
    case N_PVM: ForNeuron(NeuronPVM,AccuPVM,(sizeof(NeuronPVM)/sizeof(int))); break;
    case N_PVNL: ForNeuron(NeuronPVNL,AccuPVNL,(sizeof(NeuronPVNL)/sizeof(int))); break;
    case N_PVNR: ForNeuron(NeuronPVNR,AccuPVNR,(sizeof(NeuronPVNR)/sizeof(int))); break;
    case N_PVPL: ForNeuron(NeuronPVPL,AccuPVPL,(sizeof(NeuronPVPL)/sizeof(int))); break;
    case N_PVPR: ForNeuron(NeuronPVPR,AccuPVPR,(sizeof(NeuronPVPR)/sizeof(int))); break;
    case N_PVQL: ForNeuron(NeuronPVQL,AccuPVQL,(sizeof(NeuronPVQL)/sizeof(int))); break;//180
    case N_PVQR: ForNeuron(NeuronPVQR,AccuPVQR,(sizeof(NeuronPVQR)/sizeof(int))); break;
    case N_PVR: ForNeuron(NeuronPVR,AccuPVR,(sizeof(NeuronPVR)/sizeof(int))); break;
    case N_PVT: ForNeuron(NeuronPVT,AccuPVT,(sizeof(NeuronPVT)/sizeof(int))); break;
    case N_PVWL: ForNeuron(NeuronPVWL,AccuPVWL,(sizeof(NeuronPVWL)/sizeof(int))); break;
    case N_PVWR: ForNeuron(NeuronPVWR,AccuPVWR,(sizeof(NeuronPVWR)/sizeof(int))); break;
    case N_RIAL: ForNeuron(NeuronRIAL,AccuRIAL,(sizeof(NeuronRIAL)/sizeof(int))); break;
    case N_RIAR: ForNeuron(NeuronRIAR,AccuRIAR,(sizeof(NeuronRIAR)/sizeof(int))); break;
    case N_RIBL: ForNeuron(NeuronRIBL,AccuRIBL,(sizeof(NeuronRIBL)/sizeof(int))); break;
    case N_RIBR: ForNeuron(NeuronRIBR,AccuRIBR,(sizeof(NeuronRIBR)/sizeof(int))); break;
    case N_RICL: ForNeuron(NeuronRICL,AccuRICL,(sizeof(NeuronRICL)/sizeof(int))); break;//190
    case N_RICR: ForNeuron(NeuronRICR,AccuRICR,(sizeof(NeuronRICR)/sizeof(int))); break;
    case N_RID: ForNeuron(NeuronRID,AccuRID,(sizeof(NeuronRID)/sizeof(int))); break;
    case N_RIFL: ForNeuron(NeuronRIFL,AccuRIFL,(sizeof(NeuronRIFL)/sizeof(int))); break;
    case N_RIFR: ForNeuron(NeuronRIFR,AccuRIFR,(sizeof(NeuronRIFR)/sizeof(int))); break;
    case N_RIGL: ForNeuron(NeuronRIGL,AccuRIGL,(sizeof(NeuronRIGL)/sizeof(int))); break;
    case N_RIGR: ForNeuron(NeuronRIGR,AccuRIGR,(sizeof(NeuronRIGR)/sizeof(int))); break;
    case N_RIH: ForNeuron(NeuronRIH,AccuRIH,(sizeof(NeuronRIH)/sizeof(int))); break;
    case N_RIML: ForNeuron(NeuronRIML,AccuRIML,(sizeof(NeuronRIML)/sizeof(int))); break;
    case N_RIMR: ForNeuron(NeuronRIMR,AccuRIMR,(sizeof(NeuronRIMR)/sizeof(int))); break;
    case N_RIPL: ForNeuron(NeuronRIPL,AccuRIPL,(sizeof(NeuronRIPL)/sizeof(int))); break;//200
    case N_RIPR: ForNeuron(NeuronRIPR,AccuRIPR,(sizeof(NeuronRIPR)/sizeof(int))); break;
    case N_RIR: ForNeuron(NeuronRIR,AccuRIR,(sizeof(NeuronRIR)/sizeof(int))); break;
    case N_RIS: ForNeuron(NeuronRIS,AccuRIS,(sizeof(NeuronRIS)/sizeof(int))); break;
    case N_RIVL: ForNeuron(NeuronRIVL,AccuRIVL,(sizeof(NeuronRIVL)/sizeof(int))); break;
    case N_RIVR: ForNeuron(NeuronRIVR,AccuRIVR,(sizeof(NeuronRIVR)/sizeof(int))); break;
    case N_RMDDL: ForNeuron(NeuronRMDDL,AccuRMDDL,(sizeof(NeuronRMDDL)/sizeof(int))); break;
    case N_RMDDR: ForNeuron(NeuronRMDDR,AccuRMDDR,(sizeof(NeuronRMDDR)/sizeof(int))); break;
    case N_RMDL: ForNeuron(NeuronRMDL,AccuRMDL,(sizeof(NeuronRMDL)/sizeof(int))); break;
    case N_RMDR: ForNeuron(NeuronRMDR,AccuRMDR,(sizeof(NeuronRMDR)/sizeof(int))); break;
    case N_RMDVL: ForNeuron(NeuronRMDVL,AccuRMDVL,(sizeof(NeuronRMDVL)/sizeof(int))); break;//210
    case N_RMDVR: ForNeuron(NeuronRMDVR,AccuRMDVR,(sizeof(NeuronRMDVR)/sizeof(int))); break;
    case N_RMED: ForNeuron(NeuronRMED,AccuRMED,(sizeof(NeuronRMED)/sizeof(int))); break;
    case N_RMEL: ForNeuron(NeuronRMEL,AccuRMEL,(sizeof(NeuronRMEL)/sizeof(int))); break;
    case N_RMER: ForNeuron(NeuronRMER,AccuRMER,(sizeof(NeuronRMER)/sizeof(int))); break;//debug
    case N_RMEV: ForNeuron(NeuronRMEV,AccuRMEV,(sizeof(NeuronRMEV)/sizeof(int))); break;
    case N_RMFL: ForNeuron(NeuronRMFL,AccuRMFL,(sizeof(NeuronRMFL)/sizeof(int))); break;
    case N_RMFR: ForNeuron(NeuronRMFR,AccuRMFR,(sizeof(NeuronRMFR)/sizeof(int))); break;
    case N_RMGL: ForNeuron(NeuronRMGL,AccuRMGL,(sizeof(NeuronRMGL)/sizeof(int))); break;
    case N_RMGR: ForNeuron(NeuronRMGR,AccuRMGR,(sizeof(NeuronRMGR)/sizeof(int))); break;
    case N_RMHL: ForNeuron(NeuronRMHL,AccuRMHL,(sizeof(NeuronRMHL)/sizeof(int))); break;//220
    case N_RMHR: ForNeuron(NeuronRMHR,AccuRMHR,(sizeof(NeuronRMHR)/sizeof(int))); break;
    case N_SAADL: ForNeuron(NeuronSAADL,AccuSAADL,(sizeof(NeuronSAADL)/sizeof(int))); break;
    case N_SAADR: ForNeuron(NeuronSAADR,AccuSAADR,(sizeof(NeuronSAADR)/sizeof(int))); break;
    case N_SAAVL: ForNeuron(NeuronSAAVL,AccuSAAVL,(sizeof(NeuronSAAVL)/sizeof(int))); break;
    case N_SAAVR: ForNeuron(NeuronSAAVR,AccuSAAVR,(sizeof(NeuronSAAVR)/sizeof(int))); break;
    case N_SABD: ForNeuron(NeuronSABD,AccuSABD,(sizeof(NeuronSABD)/sizeof(int))); break;
    case N_SABVL: ForNeuron(NeuronSABVL,AccuSABVL,(sizeof(NeuronSABVL)/sizeof(int))); break;
    case N_SABVR: ForNeuron(NeuronSABVR,AccuSABVR,(sizeof(NeuronSABVR)/sizeof(int))); break;
    case N_SDQL: ForNeuron(NeuronSDQL,AccuSDQL,(sizeof(NeuronSDQL)/sizeof(int))); break;
    case N_SDQR: ForNeuron(NeuronSDQR,AccuSDQR,(sizeof(NeuronSDQR)/sizeof(int))); break;//230
    case N_SIADL: ForNeuron(NeuronSIADL,AccuSIADL,(sizeof(NeuronSIADL)/sizeof(int))); break;
    case N_SIADR: ForNeuron(NeuronSIADR,AccuSIADR,(sizeof(NeuronSIADR)/sizeof(int))); break;
    case N_SIAVL: ForNeuron(NeuronSIAVL,AccuSIAVL,(sizeof(NeuronSIAVL)/sizeof(int))); break;
    case N_SIAVR: ForNeuron(NeuronSIAVR,AccuSIAVR,(sizeof(NeuronSIAVR)/sizeof(int))); break;
    case N_SIBDL: ForNeuron(NeuronSIBDL,AccuSIBDL,(sizeof(NeuronSIBDL)/sizeof(int))); break;
    case N_SIBDR: ForNeuron(NeuronSIBDR,AccuSIBDR,(sizeof(NeuronSIBDR)/sizeof(int))); break;
    case N_SIBVL: ForNeuron(NeuronSIBVL,AccuSIBVL,(sizeof(NeuronSIBVL)/sizeof(int))); break;
    case N_SIBVR: ForNeuron(NeuronSIBVR,AccuSIBVR,(sizeof(NeuronSIBVR)/sizeof(int))); break;
    case N_SMBDL: ForNeuron(NeuronSMBDL,AccuSMBDL,(sizeof(NeuronSMBDL)/sizeof(int))); break;
    case N_SMBDR: ForNeuron(NeuronSMBDR,AccuSMBDR,(sizeof(NeuronSMBDR)/sizeof(int))); break;//240
    case N_SMBVL: ForNeuron(NeuronSMBVL,AccuSMBVL,(sizeof(NeuronSMBVL)/sizeof(int))); break;
    case N_SMBVR: ForNeuron(NeuronSMBVR,AccuSMBVR,(sizeof(NeuronSMBVR)/sizeof(int))); break;
    case N_SMDDL: ForNeuron(NeuronSMDDL,AccuSMDDL,(sizeof(NeuronSMDDL)/sizeof(int))); break;
    case N_SMDDR: ForNeuron(NeuronSMDDR,AccuSMDDR,(sizeof(NeuronSMDDR)/sizeof(int))); break;
    case N_SMDVL: ForNeuron(NeuronSMDVL,AccuSMDVL,(sizeof(NeuronSMDVL)/sizeof(int))); break;
    case N_SMDVR: ForNeuron(NeuronSMDVR,AccuSMDVR,(sizeof(NeuronSMDVR)/sizeof(int))); break;
    case N_URADL: ForNeuron(NeuronURADL,AccuURADL,(sizeof(NeuronURADL)/sizeof(int))); break;
    case N_URADR: ForNeuron(NeuronURADR,AccuURADR,(sizeof(NeuronURADR)/sizeof(int))); break;
    case N_URAVL: ForNeuron(NeuronURAVL,AccuURAVL,(sizeof(NeuronURAVL)/sizeof(int))); break;
    case N_URAVR: ForNeuron(NeuronURAVR,AccuURAVR,(sizeof(NeuronURAVR)/sizeof(int))); break;//250
    case N_URBL: ForNeuron(NeuronURBL,AccuURBL,(sizeof(NeuronURBL)/sizeof(int))); break;
    case N_URBR: ForNeuron(NeuronURBR,AccuURBR,(sizeof(NeuronURBR)/sizeof(int))); break;
    case N_URXL: ForNeuron(NeuronURXL,AccuURXL,(sizeof(NeuronURXL)/sizeof(int))); break;
    case N_URXR: ForNeuron(NeuronURXR,AccuURXR,(sizeof(NeuronURXR)/sizeof(int))); break;
    case N_URYDL: ForNeuron(NeuronURYDL,AccuURYDL,(sizeof(NeuronURYDL)/sizeof(int))); break;
    case N_URYDR: ForNeuron(NeuronURYDR,AccuURYDR,(sizeof(NeuronURYDR)/sizeof(int))); break;
    case N_URYVL: ForNeuron(NeuronURYVL,AccuURYVL,(sizeof(NeuronURYVL)/sizeof(int))); break;
    case N_URYVR: ForNeuron(NeuronURYVR,AccuURYVR,(sizeof(NeuronURYVR)/sizeof(int))); break;
    case N_VA1: ForNeuron(NeuronVA1_,AccuVA1_,(sizeof(NeuronVA1_)/sizeof(int))); break;
    case N_VA2: ForNeuron(NeuronVA2_,AccuVA2_,(sizeof(NeuronVA2_)/sizeof(int))); break;//260
    case N_VA3: ForNeuron(NeuronVA3_,AccuVA3_,(sizeof(NeuronVA3_)/sizeof(int))); break;
    case N_VA4: ForNeuron(NeuronVA4_,AccuVA4_,(sizeof(NeuronVA4_)/sizeof(int))); break;
    case N_VA5: ForNeuron(NeuronVA5_,AccuVA5_,(sizeof(NeuronVA5_)/sizeof(int))); break;
    case N_VA6: ForNeuron(NeuronVA6_,AccuVA6_,(sizeof(NeuronVA6_)/sizeof(int))); break;
    case N_VA7: ForNeuron(NeuronVA7_,AccuVA7_,(sizeof(NeuronVA7_)/sizeof(int))); break;
    case N_VA8: ForNeuron(NeuronVA8_,AccuVA8_,(sizeof(NeuronVA8_)/sizeof(int))); break;
    case N_VA9: ForNeuron(NeuronVA9_,AccuVA9_,(sizeof(NeuronVA9_)/sizeof(int))); break;
    case N_VA10: ForNeuron(NeuronVA10_,AccuVA10_,(sizeof(NeuronVA10_)/sizeof(int))); break;
    case N_VA11: ForNeuron(NeuronVA11_,AccuVA11_,(sizeof(NeuronVA11_)/sizeof(int))); break;
    case N_VA12: ForNeuron(NeuronVA12_,AccuVA12_,(sizeof(NeuronVA12_)/sizeof(int))); break;//270    
    case N_VB1: ForNeuron(NeuronVB1_,AccuVB1_,(sizeof(NeuronVB1_)/sizeof(int))); break;
    case N_VB2: ForNeuron(NeuronVB2_,AccuVB2_,(sizeof(NeuronVB2_)/sizeof(int))); break;
    case N_VB3: ForNeuron(NeuronVB3_,AccuVB3_,(sizeof(NeuronVB3_)/sizeof(int))); break;
    case N_VB4: ForNeuron(NeuronVB4_,AccuVB4_,(sizeof(NeuronVB4_)/sizeof(int))); break;
    case N_VB5: ForNeuron(NeuronVB5_,AccuVB5_,(sizeof(NeuronVB5_)/sizeof(int))); break;
    case N_VB6: ForNeuron(NeuronVB6_,AccuVB6_,(sizeof(NeuronVB6_)/sizeof(int))); break;
    case N_VB7: ForNeuron(NeuronVB7_,AccuVB7_,(sizeof(NeuronVB7_)/sizeof(int))); break;
    case N_VB8: ForNeuron(NeuronVB8_,AccuVB8_,(sizeof(NeuronVB8_)/sizeof(int))); break;
    case N_VB9: ForNeuron(NeuronVB9_,AccuVB9_,(sizeof(NeuronVB9_)/sizeof(int))); break;
    case N_VB10: ForNeuron(NeuronVB10_,AccuVB10_,(sizeof(NeuronVB10_)/sizeof(int))); break;//280
    case N_VB11: ForNeuron(NeuronVB11_,AccuVB11_,(sizeof(NeuronVB11_)/sizeof(int))); break;
    case N_VC1: ForNeuron(NeuronVC1_,AccuVC1_,(sizeof(NeuronVC1_)/sizeof(int))); break;
    case N_VC2: ForNeuron(NeuronVC2_,AccuVC2_,(sizeof(NeuronVC2_)/sizeof(int))); break;
    case N_VC3: ForNeuron(NeuronVC3_,AccuVC3_,(sizeof(NeuronVC3_)/sizeof(int))); break;
    case N_VC4: ForNeuron(NeuronVC4_,AccuVC4_,(sizeof(NeuronVC4_)/sizeof(int))); break;
    case N_VC5: ForNeuron(NeuronVC5_,AccuVC5_,(sizeof(NeuronVC5_)/sizeof(int))); break;
    case N_VC6: ForNeuron(NeuronVC6_,AccuVC6_,(sizeof(NeuronVC6_)/sizeof(int))); break;
    case N_VD1: ForNeuron(NeuronVD1_,AccuVD1_,(sizeof(NeuronVD1_)/sizeof(int))); break;
    case N_VD2: ForNeuron(NeuronVD2_,AccuVD2_,(sizeof(NeuronVD2_)/sizeof(int))); break;
    case N_VD3: ForNeuron(NeuronVD3_,AccuVD3_,(sizeof(NeuronVD3_)/sizeof(int))); break;//290
    case N_VD4: ForNeuron(NeuronVD4_,AccuVD4_,(sizeof(NeuronVD4_)/sizeof(int))); break;
    case N_VD5: ForNeuron(NeuronVD5_,AccuVD5_,(sizeof(NeuronVD5_)/sizeof(int))); break;
    case N_VD6: ForNeuron(NeuronVD6_,AccuVD6_,(sizeof(NeuronVD6_)/sizeof(int))); break;
    case N_VD7: ForNeuron(NeuronVD7_,AccuVD7_,(sizeof(NeuronVD7_)/sizeof(int))); break;
    case N_VD8: ForNeuron(NeuronVD8_,AccuVD8_,(sizeof(NeuronVD8_)/sizeof(int))); break;
    case N_VD9: ForNeuron(NeuronVD9_,AccuVD9_,(sizeof(NeuronVD9_)/sizeof(int))); break;
    case N_VD10: ForNeuron(NeuronVD10_,AccuVD10_,(sizeof(NeuronVD10_)/sizeof(int))); break;
    case N_VD11: ForNeuron(NeuronVD11_,AccuVD11_,(sizeof(NeuronVD11_)/sizeof(int))); break;
    case N_VD12: ForNeuron(NeuronVD12_,AccuVD12_,(sizeof(NeuronVD12_)/sizeof(int))); break;
    case N_VD13: ForNeuron(NeuronVD13_,AccuVD13_,(sizeof(NeuronVD13_)/sizeof(int))); break;//300

 
  }
}

/* Motor functions */
void motor(byte mode)
{
    switch(mode)
    {
        case STOP:
            digitalWrite (IN3, LOW);
            digitalWrite (IN4, LOW);
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, LOW);
            analogWrite(ENB,0);
            analogWrite(ENA,0);
        break;
        case LEFT_STOP:
            digitalWrite (IN3, LOW);
            digitalWrite (IN4, LOW);
        break;
        case RIGHT_STOP:
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, LOW);
        break;
        case RIGHT_FWD:
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, HIGH);
        break;
        case RIGHT_BWD:
            digitalWrite (IN1, HIGH);
            digitalWrite (IN2, LOW);
        break;
        case LEFT_FWD:
            digitalWrite (IN3, LOW);
            digitalWrite (IN4, HIGH);
        break;
        case LEFT_BWD:
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
        break;
        case RIGHT_ROT:
            digitalWrite (IN1, HIGH);
            digitalWrite (IN2, LOW);
            digitalWrite (IN3, LOW);
            digitalWrite (IN4, HIGH);
        break;
        case LEFT_ROT:
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, HIGH);
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
        break;
        case BWD:
            digitalWrite (IN1, HIGH);
            digitalWrite (IN2, LOW);
            digitalWrite (IN3, HIGH);
            digitalWrite (IN4, LOW);
        break;
        case FWD:
            digitalWrite (IN1, LOW);
            digitalWrite (IN2, HIGH);
            digitalWrite (IN3, LOW);
            digitalWrite (IN4, HIGH);
        break;
    }
}
//*****cambiar velocidad a los dos motores*****
void speed(int speed)
{
  analogWrite(ENB,speed);
  analogWrite(ENA,speed);
}
//*****cambiar velociad a motor derecho**********
void right_speed(int speed)
{
  analogWrite(ENA,speed);      
}
//*****cambiar velociad a motor izquierdo*********
void left_speed(int speed)
{
  analogWrite(ENB, speed);      
}
/* End Motor function */

/* Sensor function */
//**************Leer sensor de gas***************
int ReadGas()
{ 
  return analogRead(0);
}
//*****Medicion de distancia con ultrasonido******
long distance() 
{
  digitalWrite(TRIG,LOW);
  delayMicroseconds(5);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  time=pulseIn(ECHO, HIGH);//  time=pulseIn(ECHO, HIGH,1764); //MUSCLE_CM_ESTIMULATE/0.017
  dist=int(0.017*time);

	if(dist<MUSCLE_CM_ESTIMULATE)
	{
		digitalWrite(LED, HIGH);
	}
	else
	{
		digitalWrite(LED, LOW);
	}
  
  return dist;
}

int GLPLevel()
{
	return analogRead(0);	//regresa el valor de la lectura analogica del sensor GLP (gases)
}
/* End Sensor Function */

/* Connectome functions */
//**********disparo de neurona individual**********
void fireNeuron(int fneuron)
{
  if(fneuron != N_MVULVA)
  {
    dendriteAccumulate(fneuron);
  }
}

//list of muscles for get values of movement created for the stimulated neurons
int mLeft[] = {N_MDL07, N_MDL08, N_MDL09, N_MDL10, N_MDL11, N_MDL12, N_MDL13, N_MDL14, N_MDL15, N_MDL16, N_MDL17, N_MDL18, N_MDL19, N_MDL20, N_MDL21, N_MDL22, N_MDL23, N_MVL07, N_MVL08, N_MVL09, N_MVL10, N_MVL11, N_MVL12, N_MVL13, N_MVL14, N_MVL15, N_MVL16, N_MVL17, N_MVL18, N_MVL19, N_MVL20, N_MVL21, N_MVL22, N_MVL23};
int mRight[] ={N_MDR07, N_MDR08, N_MDR09, N_MDR10, N_MDR11, N_MDR12, N_MDR13, N_MDR14, N_MDR15, N_MDR16, N_MDR17, N_MDR18, N_MDR19, N_MDR20, N_MDR21, N_MDR22, N_MDR23, N_MVR07, N_MVR08, N_MVR09, N_MVR10, N_MVR11, N_MVR12, N_MVR13, N_MVR14, N_MVR15, N_MVR16, N_MVR17, N_MVR18, N_MVR19, N_MVR20, N_MVL21, N_MVR22, N_MVR23};

boolean IsMuscle(int muscle)
{
	//length of mLeft and mRight is the same
	for(int i=0; i<(sizeof(mLeft)/sizeof(int)); i++)
	{
		if( (muscle==mLeft[i]) || (muscle==mRight[i]) )
		{
			return true;
		}
	}
	return false;
}

void runconnectome()
{
  for (int pscheck = 0; pscheck < 397; pscheck++)  {
      if((pscheck<N_MVL01 || pscheck>N_MVL23) && (pscheck<N_MDL01 || pscheck>N_MDL23) && (pscheck<N_MVR01 || pscheck>N_MVR23) && (pscheck<N_MDR01 || pscheck>N_MDR23) && abs(postsynaptic[pscheck][thisState])>THRESHOLD)
      {
        fireNeuron(pscheck);
        resetSynapticNeuron(pscheck);
      }
  }
  motorcontrol();
  thisState = (thisState==0) ? 1 : 0;
  nextState = (nextState==0) ? 1 : 0;
}

//set turn on wheels of robots 
void SetTurn(float ratio){
    if(ratio<=0.6)
    {
      motor(LEFT_ROT);
      delay(800);
    }
    else if(ratio>=2)
    {
      motor(RIGHT_ROT);
      delay(800);
    }
	
}

void motorcontrol()
{
  // accumulate left and right muscles and the accumulated values are
  // used to move the left and right motors of the robot

  for (int pscheck = 0; pscheck < (sizeof(mLeft)/sizeof(int)); pscheck++)
  {   
      accumleft += postsynaptic[mLeft[pscheck]][thisState];
      postsynaptic[mLeft[pscheck]][thisState] = 0;
   }
   
  for (int pscheck = 0; pscheck < (sizeof(mRight)/sizeof(int)); pscheck++)
  {
      accumright += postsynaptic[mRight[pscheck]][thisState];
      postsynaptic[mRight[pscheck]][thisState] = 0;
   }
  

  // We turn the wheels according to the motor weight accumulation
  new_speed = abs(accumleft) + abs(accumright); 
  turnratio = float(accumright) / float(accumleft);  
  speed((abs(new_speed)>150) ? 210 : 175);

  if(accumleft==0 && accumright==0)
  {
    motor(STOP);
  }
  else if(accumright<=0 && accumleft<0)
  {  
	SetTurn(turnratio);
	motor(BWD);
    delay(500);
  }
  else if(accumright<=0 && accumleft>=0)
  {
    motor(RIGHT_ROT);
    delay(800);
  }
  else if(accumright >= 0 && accumleft <= 0)
  {
    motor(LEFT_ROT);
    delay(800);
  }
  else if(accumright >= 0 && accumleft > 0)
  {    
	SetTurn(turnratio);
	motor(FWD);
    delay(500);
  }
  else
  {
    motor(STOP);
  }
  
  accumleft = 0;
  accumright = 0;
  //delay(500);
}

/* End Connectome functions */

/*envio de los valores del conectoma entero*/
void SendData()
{
  for(int s=0; s<397; s++)
  {
    Serial.print(s);
    Serial.print(":");
    Serial.print(postsynaptic[s][0]);
    Serial.print(",");  
  }
}

void setup()
{
  // put your setup code here, to run once:
 pinMode(ENB, OUTPUT); 
 pinMode(ENA, OUTPUT); 
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(LED, OUTPUT);
 pinMode(TRIG, OUTPUT);
 pinMode(ECHO, INPUT);
 pinMode(10, INPUT);
 
 createpostsynaptic();
 Serial.begin(115200);
 Time=millis(); 
}


void loop()
{  
  // put your main code here, to run repeatedly:


if(distance()<MUSCLE_CM_ESTIMULATE) //Neuronas de la "naríz"
  {
    dendriteAccumulate(N_FLPR);//Sensory neuron (Mechanosensory) (name originally derived from "FLAP")--Head
    dendriteAccumulate(N_FLPL);//Sensory neuron (Mechanosensory) (name originally derived from "FLAP")--Head
    dendriteAccumulate(N_ASHL);//Sensory neuron (Osmo-, mechano- and odorsensory, nociceptive)--Lateral ganglia of head
    dendriteAccumulate(N_ASHR);//Sensory neuron (Osmo-, mechano- and odorsensory, nociceptive)--Lateral ganglia of head
    dendriteAccumulate(N_IL1VL);//All polymodal: [Sensory neuron (Mechanosensory), Motor neuron, Interneuron]--Head
    dendriteAccumulate(N_IL1VR);//All polymodal: [Sensory neuron (Mechanosensory), Motor neuron, Interneuron]--Head
    dendriteAccumulate(N_OLQDL);//All polymodal: [Sensory neuron (Mechanosensory) Interneuron]--Head
    dendriteAccumulate(N_OLQDR);//All polymodal: [Sensory neuron (Mechanosensory) Interneuron]--Head
    dendriteAccumulate(N_OLQVR);//All polymodal: [Sensory neuron (Mechanosensory) Interneuron]--Head
    dendriteAccumulate(N_OLQVL);//All polymodal: [Sensory neuron (Mechanosensory) Interneuron]--Head 
  }
  
if(digitalRead(PIR)==HIGH) // Neuronas sensoriales quimicas (comida), Experimentar con termosensoras
  {    
    dendriteAccumulate(N_ADFL); //Sensory neuron (chemosensory)--Lateral ganglia in head
    dendriteAccumulate(N_ADFR); //Sensory neuron (chemosensory)--Lateral ganglia in head
    dendriteAccumulate(N_ASGR); //Sensory neuron (Chemosensory)--Lateral ganglia of head
    dendriteAccumulate(N_ASGL); //Sensory neuron (Chemosensory)--Lateral ganglia of head
    dendriteAccumulate(N_ASIL); //Sensory neuron (Chemosensory, thermosensory)--Lateral ganglia of head
    dendriteAccumulate(N_ASIR); //Sensory neuron (Chemosensory, thermosensory)--Lateral ganglia of head
    dendriteAccumulate(N_ASJR); //Sensory neuron--Lateral ganglia of head
    dendriteAccumulate(N_ASJL); //Sensory neuron--Lateral ganglia of head
 
  }
  
if(GLPLevel()>500) //Neuronas sensoras de gases (experimentar con sensoras de C02, oxigeno)
{
	dendriteAccumulate(N_AFDL); //Sensory neuron (thermosensory and CO2- sensory) -- Lateral ganglia in head
	dendriteAccumulate(N_AFDR); ////Sensory neuron (thermosensory and CO2- sensory) -- Lateral ganglia in head
}


  runconnectome();

  
//Envio de datos del conectoma cada 5 segundos  
  if( ((millis()-Time) > 5000) )
  {
    SendData();
    Time=millis();
  }
  
//Exitar conectoma por bluetooth (aplicación para PC)
  if(Serial.available()>0)
  {
    /*
      42 ASCII = * (fin de cadena) 
      44 ASCII = , (separador) 
      66 ASCII = B (Envio por bluetooth ON)
      90 ASCII = Z (Envio por bluetooth OFF)   
    */
    int inChar;   
   do
   {
      inChar = Serial.read();
      if (isDigit(inChar))
      {
        inString += (char)inChar;//convert the incoming byte to a char and add it to the string   
      }
      
      if(inChar==44 || inChar==42)
      {
         dendriteAccumulate(inString.toInt());
         inString="";
      }
      
      if(inChar==66)
      {
          EnvioBluetooth=true; 
      }
      
      if(inChar==90)
      {
          EnvioBluetooth=false; 
      }   
       
   }while(inChar!= 42);
   
  }  		
         
}

