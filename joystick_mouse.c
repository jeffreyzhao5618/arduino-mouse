#include <stdio.h>   /* Standard input/output definitions */
#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <stdlib.h>  /* Exit and system*/

/*
 * Parts of code taken from:
 *    https://en.wikibooks.org/wiki/Serial_Programming/termios
 *    https://www.cmrr.umn.edu/~strupp/serial.html
 *    https://blog.mbedded.ninja/programming/operating-systems/linux/linux-serial-ports-using-c-cpp/
 */


int
main () {

  int fd; /* File descriptor for the port */

  fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY);
  if (fd == -1)
  {
    /*
     * Could not open the port.
     */

    perror("open_port: Unable to open /dev/ttyUSB0 - ");
    exit(1);
  }
  //
  // Check if the file descriptor is pointing to a TTY device or not.
  //
  if(!isatty(fd)) {
    printf("Error: file is not a tty.\n");
    exit(1);
  }

  struct termios tty;

  //
  // Get the current configuration of the serial interface
  //
  if(tcgetattr(fd, &tty) < 0) {
    printf("Error getting current termios config.");
    exit(1);
  }

  // Set input mode flags
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow constrol
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable special handling of received bytes

  // Set output mode flags
  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed

  // Set control mode flags
  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)  
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  // Set local mode flags
  tty.c_lflag &= ~ICANON; // Disable canonical mode
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP

  // Set VMIN and VTIME
  tty.c_cc[VTIME] = 10; // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;  

  // Set Baud Rate
  if(cfsetispeed(&tty, B9600) < 0 || cfsetospeed(&tty, B9600) < 0) {
    printf("Error setting ispeed and ospeed\n");
    exit(1);
  }

  //
  // Finally, apply the configuration
  //
  if(tcsetattr(fd, TCSAFLUSH, &tty) < 0) {
    printf("Error setting termios config.");
    exit(1);
  }

  printf("Starting...\n"); 
  const int BUF_SIZE = 13; // Size of buffer
  const int CMD_SIZE = 40;
  int rrc; // Read return code for error checking
  char* fgrc; // Fgets return code for error checking
  char buffer[BUF_SIZE];
  char cmd[CMD_SIZE]; // Command string array used for move_mouse
  FILE* fpa = fdopen(fd, "r"); // File pointer for fgets
  FILE* fpb; // File pointer for popen
  const char* DELIM = ","; // Delim char for strtok
  char* nl; // For removing \n char from instruction during parsing

  // Mouse settings
  const int MAX = 1023; // Max x and y value
  const int XCEN = 504; // x value when stick centered
  const int YCEN = 525; // y value when stick centered
  const int THRESH = 10; // Amount of varience from centered joystick before cursor starts moving
  const int MXSP = 14; // Max speed/amount of px to move

  // Track states
  int pcs = 1; // prev stick clicked state; 1 = not clicked; 0 = clicked
  int jsc; // joystick click val
  int jsx; // joystick x val
  int jsy; // joystick y val
  int movex; // Amount of px to move in x dir
  int movey; // Amount of px to move in y dir

  sleep(2); // Sleep for 2 seconds before starting

  // Wait for a new instruction to begin
  do {
    rrc = read(fd, buffer, 1);
    if (rrc < 0) {
      printf("Error reading: %s\n", strerror(errno));
      exit(1);
    }
  } while (*buffer != '\n');

  // Start reading new instruction  
  while (1) {
    fgrc = fgets(buffer, BUF_SIZE, fpa);
    if (fgrc == NULL) {
      printf("Error reading: %s\n", strerror(errno));
      exit(1);
    }

    // Print instruction
    // BUG: If delay is too short (<40) print causes segfault
    //printf("%s", buffer);

    // Remove \n char from instruction
    nl = strchr(buffer, '\n');
    if (nl)
      *nl = '\0';

    // Parse instruction
    // XVAL,YVAL,CLICK
    jsx = atoi(strtok(buffer, DELIM));
    jsy = atoi(strtok(NULL, DELIM));
    jsc = atoi(strtok(NULL, DELIM));

    // Move mouse
    if (abs(XCEN - jsx) > THRESH)
      movex = (XCEN > jsx) ?
        -((float) (XCEN - jsx) / XCEN) * MXSP :
        ((float) (jsx - XCEN) / (MAX - XCEN)) * MXSP;
    else
      movex = 0;
    if (abs(YCEN - jsy) > THRESH)
      movey = (XCEN > jsy) ?
        -((float) (YCEN - jsy) / YCEN) * MXSP :
        ((float) (jsy - YCEN) / (MAX - YCEN)) * MXSP;
    else
      movey = 0;

    // Create command
    sprintf(cmd, "xdotool mousemove_relative -- %d %d", movex, movey);
    system(cmd); // Execute command

    // Click
    // Check if click state cahnged
    if (pcs != jsc) {
      if (pcs == 1) {
        // Switch clicked
        pcs = 0;
        system("xdotool mousedown 1"); // Mousedown
      } else {
        // Switch unclicked
        pcs = 1;
        system("xdotool mouseup 1"); // Mouseup
      }    
    }
  } 

  return 0;
}
