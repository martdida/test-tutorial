/*****************************************************************
 * Title    : INAsrv_v1.c
 * Author   : Martin Dida
 * Date     : 03.May.2017
 * Brief    : INA219 server using fork to handle
 *            concurrent client accesses
 * Version  : 1.0
 * Options  : <eth0|wlan0> </dev/i2c-*>
 ****************************************************************/
//#define _FILE_OFFSET_BITS 64
#define SELF
#define DEBUG
#define JSON

/************************** Includes ****************************/
#include <sys/stat.h>
#include <fcntl.h>
#include <ctype.h>
#include <wait.h>
#include <signal.h>
#include <linux/i2c-dev.h>
#include "../header/tlpi_hdr.h"
#include "../header/get_num.h"   /* Declares our functions for handling
				 numeric arguments (getInt(), 
				 getLong()) */
#include "../header/error_functions.h"  /* Declares our error-handling 
					functions */
#include "../header/mkaddr.h"
#include "../header/INA219.h"
#include "../../rpi_programming/i2c/header/i2c.h"
#include "../../rpi_programming/header/curr_time.h"

/***************** Global Variable Definitions ******************/
// Usually put in dedicated header file with specifier "extern"



/************ Local Symbolic Constant Definitions ***************/
// When more, can be put in extra header file

#ifndef BUF_SIZE          /* Allow "gcc -D" to override definition */
#define BUF_SIZE 1024
#endif

/**************** New Local Types Definitions *******************/
// Uses "typedef" keyword to define new type



/************ Static global Variable Definitions ****************/
// Must be labeled "static"



//******** Static Local Functions Prototype Declarations ********/
// Use full prototype declarations. Must be labeled "static"
static void sigChldHandler(int sig)
{
  int savedErrno;

  savedErrno = errno;

  // Catch all exiting child processes 
  while (waitpid(-1, NULL, WNOHANG) > 0)
    continue;

  errno = savedErrno;
}

/*********************** Main Function **************************/
#ifdef SELF
int main(int argc, char *argv[])
{
  
  /****** Variable declaration ******/
  int ret;
  
  // File descriptors
  int ssck;                               // Normal listening socket
  int csck;                               // Client's accepted socket
  int i2cfd;                              // fd to open i2c device

  // Signal handling variables
  struct sigaction sa;

  // Networking related variables
  struct sockaddr_in addr_srvr;           // AF_INET
  struct sockaddr_in addr_clnt;           // AF_INET
  int len_inet;
  int optval = 1;
  //char *srvr_addr = NULL;

  /* Variable related to IO streams both for terminal
     and for i2c communication */
  FILE *rx = NULL;
  FILE *tx = NULL;
  char buf[BUF_SIZE];
  
  char RDbuf[2];
  int numRead, numWritten;

  // Variables related to groups and processes
  pid_t chldPid;
  gid_t rgid, egid;                 // keeping real and effective group id

  /* Variable keeping values from registers
   * calibration register, configuration register, current register
   * shunt voltage register and 2nd complement of negative value from
   * shunt voltage
   */
  short confRegVal = 0,
    calibRegVal = 0;

  measured_data_s sIna_measuring = {};
  
  // Variable keeping real voltage and current values
  double realShuntVoltVal = 0.0;
  double realBusVoltVal = 0.0;
//  double realPowerVal = 0.0;
  double realCurrVal = 0.0;

  unsigned char configuration = config_reg;
  unsigned char calibration = calib_reg;
  unsigned char current = curr_data_reg;
//  unsigned char power = power_data_reg;
  unsigned char shunt = shunt_volt_reg;
  unsigned char bus = bus_volt_reg;
  

  /********************************************************************
   ***************     SIGNAL HANDLERS SETTING      *******************
   ********************************************************************/
  
  // Set signal handler for exited children
  sigemptyset(&sa.sa_mask);

  /* Necessary to set flag SA_RESTART. Since accept(), read(), write()
     syscalls are by default set to block state, and catching of 
     SIGCHLD signal and its processing causes EINTR by which 
     aforementioned syscalls fail. If we specify SA_RESTART they are
     restarted by kernel upon returning from signal handler */
  sa.sa_flags = SA_RESTART;
  sa.sa_handler = sigChldHandler;
  
  if (sigaction(SIGCHLD, &sa, NULL) == -1)
    errExit("sigaction(2)");

  /********************************************************************
   ****************   INA219 AND SERVER SETTING   *********************
   ********************************************************************/

  
/*********************   INA219 CONFIGURATION   ************************/
  
  
  /* Set effective group id to real group id to prohibit security 
   * breaches.
   * From this moment egid will equal real gid = martin = 1000 */

  // Current effect gid for subsequent i2c dev file opening
  egid = getegid();  
  if (setegid(getgid()) == -1)
    errExit("setegid-real-gid");

  // real gid after i2c dev file opening, security reason
  rgid = getegid();    

  // Check program's command-line config entry
  if (argc < 3 || strcmp(argv[1], "--help") == 0)
    usageErr("%s <eth0|wlan0> </dev/i2c-*>\n", argv[0]);

#ifdef DEBUG
  printf("Effective gid before opening file:%d\n", (int)rgid);
#endif // DEBUG
  
  // Set effective gid back to i2c group to be able to open i2c-1 file
  if (setegid(egid) == -1)
    errExit("setegid-i2c-openning");

  // Open i2c device with INA's slave address to communicate with INA
  i2cfd = i2c_init(argv[2], INA_SLV_ADDR);

#ifdef DEBUG
  printf("Effective gid exactly after opening file:%d\n", (int)egid);
#endif // DEBUG

  /**** Set effective gid back to real gid for security reasons ****/
  if (setegid(rgid) == -1)
    errExit("setegid-back-real-gid");

#ifdef DEBUG
  egid = getegid();
  printf("Effective gid back in real gid: %d, security\n", (int)egid);
#endif // DEBUG

  #ifdef DEBUG
  
  // Read init data from configuration register of INA219
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &configuration, RDbuf);
  if (numRead == -1)
    errExit("i2c_read_data_word-config-reg-init");

  strtosh(RDbuf, confRegVal);


  printf("The init value of configuration register: 0x%02hx\n", confRegVal);
#endif // DEBUG
  
/***** Set configuration register to 0x199f and re-read its value *****/

  // Configure confRegValto value 0x199f (0x1fff)
  confRegVal= setreg(shuntBusCont, SADC_Sample128, BADC_Sample128, PGA_gain8);

  // Write confRegVal value in configuration register
  numWritten = i2c_write_data_word(i2cfd, &configuration, confRegVal);
  if (numWritten == -1)
    errExit("write-set-conf-register");

#ifdef DEBUG
  // Re-read, if confRegVal value set correctly in configuration register
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &configuration, RDbuf);
  if (numRead == -1)
    errExit("read-set-conf-register");

  strtosh(RDbuf, confRegVal)
	  
  printf("The set value of config register: 0x%02hx\n", confRegVal);
#endif // DEBUG

/**************** Check init value of calibration register ****************/
#ifdef DEBUG
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &calibration, RDbuf);
  if (numRead == -1)
    errExit("i2c_read_data_word-calib-reg-init");

  strtosh(RDbuf, calibRegVal)

  printf("The init value of calibration register: 0x%02hx\n", calibRegVal);
#endif // DEBUG
  
/****** Set calibration register to 0x1400 and re-read its value ******/

  // Write calibRegVal value in calibration register
  calibRegVal= 0x1400;
  numWritten = i2c_write_data_word(i2cfd, &calibration, calibRegVal);
  if (numWritten == -1)
    errExit("i2c_write_data_word-calib-reg-set");

#ifdef DEBUG
  // Re-read calibRegVal value set correctly in calibration register
  memset(RDbuf, 0, I2C_BUF_SIZE);
  numRead = i2c_read_data_word(i2cfd, &calibration, RDbuf);
  if (numRead == -1)
    errExit("i2c_read_data_word-calib-reg-set");

  strtosh(RDbuf, calibRegVal)
    
  printf("The set value of calibration register: 0x%02hx\n", calibRegVal);
#endif // DEBUG

  /********************************************************************
   **********************   SERVER SETTING   **************************
   *******************************************************************/
  
  // Create TCP/IP socket to use
  ssck = socket(AF_INET, SOCK_STREAM, 0);
  if (ssck == -1)
    errExit("socket(2)");

  // Make chosen interface address of server socket address either
  if (getIfaddr(ssck, (struct sockaddr *)&addr_srvr, argv[1]) == -1)
    errExit("getIfaddr()");
  addr_srvr.sin_port = htons(2500);
  addr_srvr.sin_family = AF_INET;

  /* Set socket option to suppress EADDRINUSE error
     More infor LPI Kerrisk ch. 61.10 */
  if (setsockopt(ssck, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1)
      errExit("setsockopt(2)");
      
#ifdef DEBUG
  printf("IP address: %s\n", inet_ntoa(addr_srvr.sin_addr));
  printf("Port: %u\n", ntohs(addr_srvr.sin_port));
#endif // DEBUG

  // Bind the server address to socket:
  len_inet = sizeof addr_srvr;
  if (bind(ssck, (struct sockaddr*)&addr_srvr, len_inet) == -1)
    errExit("bind(2)");

  /* Make socket listening */
  ret = listen(ssck, 10);
  if (ret == -1)
    errExit("listen(2)");

  /* Start processing clients requests */
  while (1) {

    /* Wait for any client to connect */
    
    /* get ready the length of client's address structure 
       pass it as "result-value" to accept syscall */
    len_inet = sizeof addr_clnt;    
csck = accept(ssck, (struct sockaddr *)&addr_clnt, (socklen_t *)&addr_clnt);
    if (csck == -1)
      errExit("accept(2)");

    /* Fork to process new client's accepted connection */
    switch (chldPid = fork()) {
    case -1:
      /* If forking child failed close accepted connection
	 and continue to accept new connection */
      if (close(csck) == -1) 
	fprintf(stderr,	"close(csck)\n");
      break;

    case 0:               // Child process

      // Close copy of unneeded listening socket
      if (close(ssck) == -1)    
	fprintf(stderr,
		"%s close(ssck)\n", strerror(errno));
      
      // Create streams:
      rx = fdopen(csck, "r");
      if ( !rx ) {           // If rx stream creation failed
	_exit(EXIT_FAILURE);
      }

      tx = fdopen(dup(csck), "w");
      if ( !tx ) {           // If tx stream creation failed
	fclose(rx);
	_exit(EXIT_FAILURE);
      }

      // Set both streams to line buffered mode
      setlinebuf(rx);
      setlinebuf(tx);

  /* Process client's request. In our case it is reading voltage
     current and log from INA219 measuring system and transmitting 
     it back to client */

      // Clear I/O buffer
      memset(buf, 0, sizeof buf);
      
      while ( fgets(buf, sizeof buf, rx) ) {
	strtok(buf, "\r\n");
	// buf[strlen(buf) - 2] = '\0';   // Terminate command with nul

/**********************************   Voltage   ************************************/
	if ( !strcmp(buf, "voltage") ) {

	  // Read value from shunt voltage register
	  numRead = i2c_read_data_word(i2cfd, &shunt, RDbuf);
	  if (numRead == -1) {
	    fprintf(tx,
		    "{ \"ERROR\":\"i2c_read_data_word(shunt-volt-reg)\" }\n");
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	  
	    _exit(EXIT_FAILURE);
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
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	  
	    _exit(EXIT_FAILURE);       // _exit should close all open file descriptors
	  }

	  strtosh(RDbuf, sIna_measuring.busRegVal)
	  
#ifdef DEBUG
	    // printf("The value of busRegVal: 0x%02hx\n", sIna_measuring.busRegVal);
#endif //DEBUG
     
	    if (sIna_measuring.busRegVal & CNVR) 
	      realBusVoltVal = busVoltConv(sIna_measuring.busRegVal);
	    else{
	      //     printf("Bus voltage not measured this time\n");
	    }
	
#ifdef JSON
	  fprintf(tx, "{ \"timestamp\":\"%s\", \"voltage\":%.2f };\n",
		  currTime("%d/%m/%y %T"), realBusVoltVal + realShuntVoltVal / 1000);
#else // JSON
	  fprintf(tx, "The actual value of shunt voltage: %.2f mV\n", realShuntVoltVal);
	  fprintf(tx, "The actual value of bus voltage: %.2f\n", realBusVoltVal);
#endif // JSON

       }


/**********************************   Current    **********************************/
	else if ( !strcmp(buf, "current") ) {

	  // Read value from current register
	  numRead = i2c_read_data_word(i2cfd, &current, RDbuf);
	  if (numRead == -1) {
	    fprintf(tx,
		    "{ \"ERROR\":\"i2c_read_data_word(current-reg)\" }\n");
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	  
	    _exit(EXIT_FAILURE);
	  }

	  strtosh(RDbuf, sIna_measuring.currRegVal)

	    realCurrVal = currConv(sIna_measuring.currRegVal);

#ifdef JSON
	  fprintf(tx, "{ \"timestamp\":\"%s\", \"current\":%.2f };\n",
		  currTime("%d/%m/%y %T"), realCurrVal);
#else // JSON
	  fprintf(tx, "The actual value of current: %.2f A\n", realCurrVal);
#endif // JSON

	}

/*************************************    log    ***********************************/
	else if ( !strcmp(buf, "log") ) {

	  // Read value from shunt voltage register
	  numRead = i2c_read_data_word(i2cfd, &shunt, RDbuf);
	  if (numRead == -1) {
	    fprintf(tx,
		    "{ \"ERROR\":\"i2c_read_data_word(shunt-volt-reg)\" }\n");
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	   
	    _exit(EXIT_FAILURE);
	  }

	  strtosh(RDbuf, sIna_measuring.shuntRegVal)

	    // Read value from bus voltage register
	    numRead = i2c_read_data_word(i2cfd, &bus, RDbuf);
	  if (numRead == -1) {
	    fprintf(tx,
		    "{ \"ERROR\":\"i2c_read_data_word(bus-volt-reg)\" }\n");
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	   
	    _exit(EXIT_FAILURE);
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

	  // Read value from current register
	  numRead = i2c_read_data_word(i2cfd, &current, RDbuf);
	  if (numRead == -1) {
	    fprintf(tx,
		    "{ \"ERROR\":\"i2c_read_data_word(current-reg)\" }\n");
	    fclose(tx);
	    shutdown(fileno(rx), SHUT_RDWR);
	    fclose(rx);
	   
	    _exit(EXIT_FAILURE);
	  }

	  // Make current conversions
	  strtosh(RDbuf, sIna_measuring.currRegVal)
	    realCurrVal = currConv(sIna_measuring.currRegVal);
	 
#ifdef DEBUG
	  //       printf("The value of busRegVal: 0x%02hx\n", sIna_measuring.busRegVal);
#endif //DEBUG
     
#ifdef JSON
	  fprintf(tx, "{\n\"log\":{ \"timestamp\":\"%s\", \"voltage\":%.2f, \"current\":%.2f }\n}\n",
		  currTime("%d/%m/%y %T"), realBusVoltVal + (realShuntVoltVal / 1000), realCurrVal);
#else // JSON
	  fprintf(tx, "The actual value of shunt voltage: %.2f mV\n", realShuntVoltVal);
	  fprintf(tx, "The actual value of bus voltage: %.2f\n", realBusVoltVal);
#endif // JSON

	}

      /*****************************************  exit  *******************************************/
	else if ( !strcmp(buf, "exit") ) {
	  fclose(tx);
	  shutdown(fileno(rx), SHUT_RDWR);
	  fclose(rx);
	  _exit(EXIT_SUCCESS);
	}


      /****************************************  Unknown command  *********************************/
	else {
#ifdef JSON
	  fprintf(tx, "{ \"WARN\":\"Unrecognized command! Valid commands are: 'voltage', 'current', 'log', 'exit'\" }\n");
#else //JSON
	  fprintf(tx, "Unrecognized command!\n"
		  "Valid commands are: \'voltage\', \'current\', \'log\', \'exit\'\n");
#endif //JSON
	
	}
      }
      

    default :
      close(csck);     // Uneeded copy of client's connected socket
      break;
    }
  }

  exit(EXIT_SUCCESS);
}
#endif // SELF



/**************** Global Functions Definitions ******************/



/***************** Local Functions Definitions ******************/
// Must be labeled "static"
