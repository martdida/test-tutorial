/*****************************************************************
 * Title    : INA219_measuring_srv.c
 * Author   : Martin Dida
 * Date     : 23.Feb.2017
 * Brief    : Application to measure voltage and current consumption
 *            on LED/LCD display boards (for Amena.sk)
 *            utilizing blocking multiplexing on stdin fd 
 *            via select() system call and communicating via network
 * Version  : v1
 * Options  : </dev/i2c-*> 
 ****************************************************************/
//#define _FILE_OFFSET_BITS 64
#define SELF
#define DEBUG
//#define PRINT
#define JSON

/****************************************************************/
/************************** Includes ****************************/
/****************************************************************/
#include <fcntl.h>
#include <stdio_ext.h>
#include <time.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/select.h>
#include "../../header/tlpi_hdr.h"
#include "../../header/get_num.h"   /* Declares our functions for handling
				 numeric arguments (getInt(),
				 getLong()) */
#include "../../header/error_functions.h"  /* Declares our error-handling
					functions */
#include "../../i2c/header/i2c.h"
#include "../header/INA219.h"
#include "../../header/curr_time.h"
#include "../../../linux_programming/header/genServer.h"

/****************************************************************/
/***************** Global Variable Definitions ******************/
/****************************************************************/
// Usually put in dedicated header file with specifier "extern"


/****************************************************************/
/************ Local Symbolic Constant Definitions ***************/
/****************************************************************/
// When more, can be put in extra header file

// To use 4us writing/reading delay for INA219 circuit
#define INA219


#ifndef BUF_SIZE          /* Allow "gcc -D" to override definition */
#define BUF_SIZE 1024
#endif

/****************************************************************/
/**************** New Local Types Definitions *******************/
/****************************************************************/
// Uses "typedef" keyword to define new type


/****************************************************************/
/************ Static global Variable Definitions ****************/
/****************************************************************/
// Must be labeled "static"
static char *contMessage = "To continue enter 'voltage', 'current' \
'log', 'exit'";
     


/****************************************************************/
/********* Static Local Functions Prototype Declarations ********/
/****************************************************************/
// Use full prototype declarations. Must be labeled "static"
static void sigContHandler(int sig)
{
  write(csck,
	, contMessage
	, strlen(contMessage));
}
 


/****************************************************************/
/*********************** Main Function **************************/
/****************************************************************/

#ifdef SELF
int main(int argc, char *argv[])
{
  // Processe's and files related variables
  int i2cfd, nfds, readyfds;
  fd_set readfds;
  gid_t rgid, egid;      // keeping real and effective group id
  //  char *userPath, logFilePath[256];
  char logEntry[BUF_SIZE] ;

  // Signal related variables
  struct sigaction sa;
  
    
  // Variable handling read/write functionality of i2c device
  int numRead, numWritten;
  char RDbuf[2];
  char * command[10]; */

  /* Variable keeping values from registers
   * calibration register, configuration register, current register
   * shunt voltage register and 2nd complement of negative value from
   * shunt voltage
   */
  short confRegVal = 0,
    calibRegVal = 0;

  MEASURE_DATA sIna_measuring = {};

  // Variable keeping real voltage and current values
  double realShuntVoltVal = 0.0;
  double realBusVoltVal = 0.0;
  double realPowerVal = 0.0;
  double realCurrVal = 0.0;

  // Variables related to time and timers(needed for logs)
  //  struct timeval timeout;
  
  //  struct tm *currTime;
  //char formTime[50];
  
  
  unsigned char configuration = config_reg;
  unsigned char calibration = calib_reg;
  unsigned char current = curr_data_reg;
  unsigned char power = power_data_reg;
  unsigned char shunt = shunt_volt_reg;
  unsigned char bus = bus_volt_reg;

  /***************************************************************************/
  /************************* PART SETTING SYSTEMS CONFIG *********************/
  /***************************************************************************/

  // Initializing of some variables
  memset(&sIna_measuring, 0, sizeof(struct measured_value));
  memset(logEntry, 0, BUF_SIZE);

  // Check program's entry
  if (argc < 2 || strcmp(argv[1], "--help") == 0)
    usageErr("%s </dev/i2c-[01]\n", argv[0]);

  // Create RD/WR file streams
  FILE* rx = fdopen(csck, "r");
  if ( !rd )
      errExit("fdopen(rx)");

  FILE* tx = fdopen(dup(csck), "w");
  if ( !wr ) {
    if (fclose(rx) == EOF)
      fprintf(stderr,
              "fclose(rx)\n");
      errExit("fdopen(tx)");
  }

  setlinebuf(rx);
  setlinebuf(tx);

  /* SIGCONT signal handler activation */
  sigemptyset(&sa.sa_mask);
  sa.sa_handler = sigContHandler;
  sa.sa_flags = 0;
  if (sigaction(SIGCONT, &sa, NULL) == -1)
    errExit("sigaction(SIGCONT)");

  /* Set effective group id to real group id to prohibit security breaches
   * Since this point egid will equal real gid = martin = 1000
   */
  egid = getegid();  // Current effect gid for subsequent i2c dev file opening
  if (setegid(getgid()) == -1) {
    fprintf(tx,
	    "setegid(real-gid)\n");
    exit(EXIT_FAILURE);
  }
  rgid = getegid(); // real gid after i2c dev file opening, security reason

#ifdef DEBUG
  fprintf(tx,
          "Effective gid before opening file:%d\n",
          (int)rgid);
#endif // DEBUG
  
  // Set effective gid back to i2c group to be able to open i2c-1 file
  if (setegid(egid) == -1) {
    fprintf(tx,
	    "{ \"ERROR\":\"setegid-effective-gid\" }");
    exit(EXIT_FAILURE);
  }

  // Open i2c device with INA's slave address to communicate with INA
  i2cfd = i2c_init(argv[1], INA_SLV_ADDR);

#ifdef DEBUG
  fprintf(tx,
          "Effective gid exactly after opening file:%d\n",
          (int)egid);
#endif // DEBUG

  /**** Set effective gid back to real gid for security reasons ****/
  if (setegid(rgid) == -1)
    errExit("{ \"ERROR\":\"setegid-back-real-gid\" }");

#ifdef DEBUG
  egid = getegid();
  fprintf(tx,
          "Effective gid back in real gid: %d, security\n",
          (int)egid);
#endif // DEBUG

  /*
   * Create format of measured data entries to write to file
   */

 /****************************************************************************/
 /**************************** I2C INA-219 COMMUNICATION *********************/
 /****************************************************************************/
#ifdef DEBUG
  // Read init data from configuration register of INA219
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &configuration, RDbuf);
  if (numRead == -1) {
    fprintf(tx,
	    "{ \"ERROR\":\"i2c_read_data_word(configuration_reg)\" }\n");
    exit(EXIT_FAILURE);
  }

  strtosh(RDbuf, confRegVal);

  fprintf(tx,
          "The init value of configuration register: 0x%02hx\n",
          confRegVal);

#endif // DEBUG
  
/**********************************************************************/
/***** Set configuration register to 0x199f and re-read its value *****/
/**********************************************************************/
	  
  // Configure confRegValto value 0x199f (0x1fff)
  confRegVal= setreg(shuntBusCont, SADC_Sample128, BADC_Sample128, PGA_gain8);

  // Write confRegVal value in configuration register
  numWritten = i2c_write_data_word(i2cfd, &configuration, confRegVal);
  if (numWritten == -1) {
    fprintf(tx,
	    "{ \"ERROR\":\"i2c_write_data_word(set-config-reg)\" }\n");
    exit(EXIT_FAILURE);
  }

#ifdef DEBUG
  // Re-read, if confRegVal value set correctly in configuration register
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &configuration, RDbuf);
  if (numRead == -1) {
    fprintf(tx,
	    "read-set-conf-register\n");
    exit(EXIT_FAILURE);
  }

  strtosh(RDbuf, confRegVal)


    fprintf(tx,
            "The set value of config register: 0x%02hx\n",
            confRegVal);
#endif // DEBUG

/**************** Check init value of calibration register ****************/
#ifdef DEBUG
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &calibration, RDbuf);
  if (numRead == -1) {
    fprintf(tx,
	   "i2c_read_data_word-calib-reg-init\n");
    exit(EXIT_FAILURE);
  }

  strtosh(RDbuf, calibRegVal)

  fprintf(tx,
          "The init value of calibration register: 0x%02hx\n",
          calibRegVal);

#endif // DEBUG

/****** Set calibration register to 0x1400 and re-read its value ******/

  // Write calibRegVal value in calibration register
  calibRegVal= 0x1400;
  numWritten = i2c_write_data_word(i2cfd, &calibration, calibRegVal);
  if (numWritten == -1) {
    fprintf(tx,
	    "{ \"ERROR\":\"i2c_write_data_word(set-calib-reg)\" }\n");
    exit(EXIT_FAILURE);
  }
  
#ifdef DEBUG
  // Re-read calibRegVal value set correctly in calibration register
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &calibration, RDbuf);
  if (numRead == -1) {
    fprintf(tx,
	    "i2c_read_data_word-calib-reg-set\n");
    exit(EXIT_FAILURE);
  }

  strtosh(RDbuf, calibRegVal)
    
  fprintf(tx,
          "The set value of calibration register: 0x%02hx\n",
          calibRegVal);
  
#endif // DEBUG
  
  /*
   * Read Current, Power, Bus & Shunt Voltage Register values
   * convert it to human readable format, write it to log file
   */

  fprintf(tx,
          "{ \"INFO\":\"Enter 'voltage', 'current', 'log', 'exit'\" }\n");

  for (;;) {

     /* Set timeval to zero and make ready readfds for select syscall */
    //     timeout.tv_sec = 0;
    //     timeout.tv_usec = 0;
     nfds = STDIN_FILENO + 1;
     FD_ZERO(&readfds);
     FD_SET(STDIN_FILENO, &readfds);

     // Wait in blocking mode until stdin fd is ready for reading
     while ((readyfds = select(nfds, &readfds, NULL, NULL, NULL)) == -1 && errno == EINTR);
     if (readyfds == -1) {
       fprintf(tx,
	       "{ \"ERROR\":\"select\" errno: %s }\n"
	       , strerror(errno));
       exit(EXIT_FAILURE);
     }

     // Check if stdin fd already polled
     if (FD_ISSET(STDIN_FILENO, &readfds) == 1) {

       fgets(command, sizeof command, rx);
       if (strcmp(command, "log") == 0) {

	 // Read value from shunt voltage register
	 numRead = i2c_read_data_word(i2cfd, &shunt, RDbuf);
	 if (numRead == -1) {
	   fprintf(tx,
		   "{ \"ERROR\":\"i2c_read_data_word(shunt-volt-reg)\" }\n");
	   exit(EXIT_FAILURE);
	 }

	 strtosh(RDbuf, sIna_measuring.shuntRegVal)
	 
	 // Read value from bus voltage register
	 numRead = i2c_read_data_word(i2cfd, &bus, RDbuf);
	 if (numRead == -1) {
	   fprintf(tx,
		   "{ \"ERROR\":\"i2c_read_data_word(bus-volt-reg)\" }\n");
	   exit(EXIT_FAILURE);
	 }

	 // Convert shunt-voltage. If negative voltage convert it to positive
	 if (sign(sIna_measuring.shuntRegVal) == -1) {
	   sIna_measuring.complVal = complement(sIna_measuring.shuntRegVal);
	   realShuntVoltVal = shuntVoltConv(sIna_measuring.complVal);
	 }
	 else {
	   realShuntVoltVal = shuntVoltConv(sIna_measuring.shuntRegVal);
	 }
	 
	 // Make bus voltage conversions
	 strtosh(RDbuf, sIna_measuring.busRegVal)
	 if (sIna_measuring.busRegVal & CNVR)
	   realBusVoltVal = busVoltConv(sIna_measuring.busRegVal);
	 else{
	    printf("Bus voltage not measured this time\n");
	 }

	 // Read value from current register
	 numRead = i2c_read_data_word(i2cfd, &current, RDbuf);
	 if (numRead == -1) {
	   fprintf(tx,
		   "{ \"ERROR\":\"i2c_read_data_word(current-reg)\" }\n");
	   exit(EXIT_FAILURE);
	 }

	 // Make current conversions
	 strtosh(RDbuf, sIna_measuring.currRegVal)
	 realCurrVal = currConv(sIna_measuring.currRegVal);

#ifdef DEBUG
       //       printf("The value of busRegVal: 0x%02hx\n", sIna_measuring.busRegVal);
#endif //DEBUG
     
#ifdef JSON
	 fprintf(tx,
             "{\n\"log\":{ \"timestamp\":\"%s\", \"voltage\":%.2f, \"current\":%.2f }\n}\n",
		currTime("%d/%m/%y %T"), realBusVoltVal + (realShuntVoltVal / 1000), realCurrVal);
     
#else // JSON
	 fprintf(tx,
             "The actual value of shunt voltage: %.2f mV\n",
             realShuntVoltVal);
	 fprintf(tx,
             "The actual value of bus voltage: %.2f\n",
             realBusVoltVal);
#endif // JSON
       }

       /********************** Voltage measuring **************************/
       else if (strcmp(command, "voltage") == 0) {
	 
     // Read value from shunt voltage register
     numRead = i2c_read_data_word(i2cfd, &shunt, RDbuf);
     if (numRead == -1) {
       fprintf(tx,
	       "{ \"ERROR\":\"i2c_read_data_word(shunt-volt-reg)\" }\n");
       exit(EXIT_FAILURE);
     }

     strtosh(RDbuf, sIna_measuring.shuntRegVal)

     // If negative voltage convert it to positive
     if (sign(sIna_measuring.shuntRegVal) == -1) {
       sIna_measuring.complVal = complement(sIna_measuring.shuntRegVal);
       realShuntVoltVal = shuntVoltConv(sIna_measuring.complVal);
     }
     else {
       realShuntVoltVal = shuntVoltConv(sIna_measuring.shuntRegVal);
     }

     /* Read value from bus voltage register
      * Check if data converted and get bus voltage real value
      */
     numRead = i2c_read_data_word(i2cfd, &bus, RDbuf);
     if (numRead == -1) {
       fprintf(tx,
	       "{ \"ERROR\":\"i2c_read_data_word(bus-volt-reg)\" }\n");
       exit(EXIT_FAILURE);
     }

     strtosh(RDbuf, sIna_measuring.busRegVal)
       
#ifdef DEBUG
       //       printf("The value of busRegVal: 0x%02hx\n", sIna_measuring.busRegVal);
#endif //DEBUG
     
       if (sIna_measuring.busRegVal & CNVR)
         realBusVoltVal = busVoltConv(sIna_measuring.busRegVal);
       else{
	 //     printf("Bus voltage not measured this time\n");
       }
#ifdef JSON
	 fprintf(tx,
             "{ \"timestamp\":\"%s\", \"voltage\":%.2f };\n",
		currTime("%d/%m/%y %T"), realBusVoltVal + realShuntVoltVal / 1000);
#else // JSON
	 fprintf(tx,
             "The actual value of shunt voltage: %.2f mV\n",
             realShuntVoltVal);
	 fprintf(tx,
             "The actual value of bus voltage: %.2f\n",
             realBusVoltVal);
#endif // JSON
       }

       /*********************************** Current measuring *************************************/
       else if (strcmp(command, "current") == 0) {

	 // Read value from current register
	 numRead = i2c_read_data_word(i2cfd, &current, RDbuf);
	 if (numRead == -1) {
	   fprintf(tx,
		   "{ \"ERROR\":\"i2c_read_data_word(current-reg)\" }\n");
	   exit(EXIT_FAILURE);
	 }

	 strtosh(RDbuf, sIna_measuring.currRegVal)

	 realCurrVal = currConv(sIna_measuring.currRegVal);

#ifdef JSON
	 fprintf(tx,
            "{ \"timestamp\":\"%s\", \"current\":%.2f };\n",
            currTime("%d/%m/%y %T"), realCurrVal);
#else // JSON
     fprintf(tx,
             "The actual value of current: %.2f A\n",
             realCurrVal);
#endif // JSON
       }
       
#ifdef POWER
       else if (strcmp(command, "power") == 0) {@

	 // Read value from power register
	 numRead = i2c_read_data_word(i2cfd, &power, RDbuf);
	 if (numRead == -1) {
	   fprintf(stderr,
		   "{ \"ERROR\":\"i2c_read_data_word(power-reg)\" }\n");
	   exit(EXIT_FAILURE);
	 }
     
	 strtosh(RDbuf, sIna_measuring.powerRegVal)

	 realPowerVal = pwrConv(sIna_measuring.powerRegVal);

#ifdef JSON
	 fprintf(tx,
             "{ \"timestamp\":\"%s\", \"power\":%.2f };\n",
             currTime("%d/%m/%y %T"), realPowerVal);
     
#else // JSON
	 fprintf(tx,
             "The actual value of power: %.2f W\n",
             realPowerVal);
#endif // JSON
       }
#endif // POWER

       /******************************* Exit of program
       else if (strcmp(command, "exit") == 0) {
#ifdef JSON
         fprintf(tx,
                 "{ \"INFO\":\"You are exiting %s application\" }\n",
                 argv[0]);
#else //JSON
         fprintf(tx,
                 "You are exiting %s application",
                 argv[0]);
#endif //JSON
	 break;
       }
       else {
#ifdef JSON
	 printf("{ \"WARN\":\"Unrecognized command! Valid commands are: 'voltage', 'current', 'log', 'exit'\" }\n");
#else //JSON
	 printf("Unrecognized command!\n"
		"Valid commands are: \'voltage\', \'current\', \'log\', \'exit\'\n");
#endif //JSON
	 
       }
     }
  }

  if(close(i2cfd) == -1) {
    fprintf(stderr,
	    "{ \"ERROR\":\"close-i2cfd\" }\n");
    exit(EXIT_FAILURE);
  }


  exit(EXIT_SUCCESS);
}
  
#endif // SELF


/****************************************************************/
/************* Static Local Functions Definitions ***************/
/****************************************************************/
// Must be labeled "static"



  


/****************************************************************/
/**************** Global Functions Definitions ******************/
/****************************************************************/





