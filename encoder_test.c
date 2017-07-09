//compile with  gcc threaded_main.c -lpthread -lrt -lm -o main
#include <termios.h>
#include <pthread.h>
#include <stdio.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <signal.h>
#include <malloc.h>
#include <netdb.h>
#include <sys/wait.h>
#include <sys/socket.h>
#include <resolv.h>
#include <arpa/inet.h>
#include <sys/un.h>
#include "serial.c"
#include <unistd.h>
#include <sys/time.h>
#include "forklift_dbw_smoothspeed.c"
#include <sys/time.h>
#include<math.h>
#include <netinet/in.h>
#include <netinet/ip.h>
#include <sys/wait.h>
#include <sys/socket.h>

#define SWITCH 0
#define STEARING 1
#define FORWARD 2
#define REVERSE 3
#define EMERGENCY 4
#define LIFT_DOWN 5 
#define LIFT_UP 6 
#define HORN 7
#define DOME_LIGHT 8
#define OMRON_FORK1 9
#define OMRON_FORK2 10
#define OMRON_DRIVE1 11
#define OMRON_DRIVE2 12
#define GROUND_HEIGHT 917827048
#define PICK_CONVEYOR 918569018
#define DROP_CONVEYOR 918286151
#define MAX_HEIGHT 918695692
#define ENCODER_OFFSET 50000



#define SERIAL_DEV  "/dev/ttyACM0"
#define SERIAL_DEV1 "/dev/pts/1"
//char c[20];
//char cbuf[20],data[30];
int serialfd;
int currentangle = 0;
int steering_offset = 0;
unsigned int *intdat,intval,*outdat,temp,*out1dat;
int currentspeed = 0;
struct termios term;
double wheelbase = 1.350; //1.4
double rpmdivider = 5215.53;//5208.33; //5230.73
double Xodo = 0.0;
double Yodo = 0.0;
double thetaodo = 0.0;
double velrobotframe = 0.0;
int encoderCount = 0;
int prevencoderCount = 0;
int  steerpluscount = 0;
int steerminuscount = 0;
int odomthreadrunning = 0;
float buf[3] = {0.0,0.0,0.0};
double amclXnew = 0.0;
double amclYnew = 0.0;
double amclthetaradian = 0.0;
int sock;
int addr_len, bytes_read;
struct sockaddr_in server_addr , client_addr;
int udpSocket, nBytes;
struct sockaddr_in serverAddr, clientAddr;
struct sockaddr_storage serverStorage;
socklen_t addr_size, client_addr_size;

int amclthreadrunning = 0;
int warn,obs,alpha;
int mask[]={
	0x00000008,//0. Key Switch        
	0x00000010,//1. stearing ON                            
        0x00000040,//2. Forward ON                                
        0x00000080,//3. Reverse ON                              
        0x00000100,//4. Vehicle Emergency
        0x00000200,//5. Lift Down
        0x00000400,//6. Lift UP
        0x00000800,//7. Horn                              
        0x00001000,//8. Dome Light Red
        0x00004000,//9. OMRON Fork sensor S1
        0x00008000,//10. OMRON Fork sensor S2
        0x00040000,//11. OMRON Drive sensor S1
        0x00080000};//12. OMRON Drive sensor S1



int initUDPEncoderData()
{
	if ((udpSocket = socket(AF_INET, SOCK_DGRAM, 0)) == -1)
	{
		perror("Client-socket() error");
	}
	else
		printf("Client-socket() OK\n");
	/*Configure settings in address struct*/
	serverAddr.sin_family = AF_INET;
	serverAddr.sin_port = htons(5005);
	serverAddr.sin_addr.s_addr = inet_addr("192.168.16.202");
	memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);


	clientAddr.sin_family = AF_INET;
	clientAddr.sin_port = htons(2222);
	clientAddr.sin_addr.s_addr = inet_addr("192.168.1.170");
	memset(clientAddr.sin_zero, '\0', sizeof clientAddr.sin_zero);
	/*Bind socket with address struct*/
	bind(udpSocket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));

	/*Initialize size variable to be used later on*/
	addr_size = sizeof serverStorage;
	return 0;
}


void receive_udp()


{
	int bytesreadUDP = 0;
	char buf[50];
	bytesreadUDP = recvfrom(sockUDP,buf,50,0,(struct sockaddr *)&client_addr, &addr_len);
	encoder_value = buf[0];
	bzero(buf,50);
}

void start_serial(void)
{
	serialfd = init96008n1(SERIAL_DEV);
	if(serialfd != NULL)
		printf("serial Detected\n");	
	else
		printf("serial Not opened\n");
	sleep(1);
}

void move_val(int val1 ,int val2)
{
	char command[10],cbuf[30], command1[10];
	int n;
	sprintf(command, "%d,%d\r", val1,val2);
	//sprintf(command1, "%d\r", val2);
	write(serialfd, command, strlen(command));
       // write(sd, command1, strlen(command1)); 
	usleep(100000);
}

void start_DIO(void)
{
	int fd = open("/dev/mem", O_RDWR|O_SYNC);
	    unsigned char *start,*start1;
	   // unsigned int *intdat,intval,*outdat,temp,*out1dat;
	    //intval=0xFFFFFFFF;
	    off_t addr,page,in,out,addr1,page1,out1;
	    addr=0xEE000000;
	    addr1=0xE8000000;
	    in=0x0104;//DIO64 outout port
	    out=0x0108;//DIO64 input port
	    out1=0x0008;//DIO output port
	    page = addr & 0xffff0000;
	    page1= addr1 & 0xFFFF0000;

	    start = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, page);//start for DIO64
	    start1 = mmap(0, getpagesize(), PROT_READ|PROT_WRITE, MAP_SHARED, fd, page1);//start for DIO

	    if (start == MAP_FAILED) {
	        perror("mmap:");
	        return 0;
	    }
		intdat = (unsigned int *)(start + (in & 0xFFFF));//pointer for DIO64 outout port
		outdat = (unsigned int *)(start + (out & 0xFFFF));//pointer for DIO64 input port
	    	out1dat = (unsigned int *)(start1 + (out1 & 0xFFFF));//pointer for DIO output port

}
void dio_rw(int func, int state)
{
	int tempOP;
	tempOP = *intdat;
		
	if(state == 0)
	tempOP = tempOP & (~mask[func]);	
	else if(state == 1)
	tempOP = tempOP | mask[func];
	
	*intdat = tempOP;
}
	 
static void exit_gracefully()
{
	close_forklift();
	odomthreadrunning = 0;
	amclthreadrunning = 0;
	fprintf(stderr,"Exiting......\n");
	sleep(1);

	// turn on Canonical processing in term
	term.c_lflag |= ICANON;

	// turn on screen echo in term
	term.c_lflag |= ECHO;

	// set the terminal configuration for stdin according to term, now
	tcsetattr( fileno(stdin), TCSANOW, &term);
	exit(0);
}

/*!
  \brief Receives keyboard inputs
  \return int - char input from keyboard
  */
static int kbhit(void)
{
	struct termios oldt, newt;
	int ch = 0;

	tcgetattr( STDIN_FILENO, &oldt );
	newt = oldt;

	newt.c_cc[VMIN] = 0;
	newt.c_cc[VTIME] = 1;
	newt.c_lflag &= ~( ICANON | ECHO );
	tcsetattr( STDIN_FILENO, TCSANOW, &newt );

	ch = getchar();

	//        tcsetattr( STDIN_FILENO, TCSANOW, &oldt );

	return ch;
}

void encoderSteerAngle()
{
        double encoderangle = 0.0;
        odomthreadrunning = 1;
        while(odomthreadrunning == 1)
        {
                int encodervalue = readEncoderValue(2);
                if(encodervalue < 0)
                {
                         encoderangle = (-90.0/10340)*encodervalue;
                       //printf("Encoder Angle: %f\n",encoderangle);
                }
                else if(encodervalue >= 0)
                {
                         encoderangle = (-90.0/10340)*encodervalue;
                       //printf("Encoder Angle: %f\n",encoderangle);
                }
                update_odometry(encoderangle);
                usleep(50000);
        }
        pthread_exit(0);
}


void update_odometry(double steertheta)
{
       // float wheelbase = 1111;
       // float rpmdivider = 12345;
        encoderCount = readEncoderValue(1);
        int steercount = readEncoderValue(2);
	
	if(abs(prevencoderCount - encoderCount) > 1000)
                encoderCount = prevencoderCount;

        //svel = (encoderCount - prevencoderCoun)/rpmdivider;
        velrobotframe = ((encoderCount - prevencoderCount)/rpmdivider) * cos(steertheta*M_PI/180.0);
        Xodo += velrobotframe * cos(thetaodo);
        Yodo += velrobotframe * sin(thetaodo);
        thetaodo += atan(((encoderCount - prevencoderCount)/(wheelbase*rpmdivider)) * sin(steertheta*M_PI/180.0));
        if(thetaodo > 2.0*M_PI)
                 thetaodo = thetaodo - 2.0*M_PI;
        else if(thetaodo < 0.0)
                 thetaodo = 2.0*M_PI + thetaodo;
//      printf("ENCODER : %d PREV ENCODER: %d",encoderCount,prevencoderCount);
        prevencoderCount = encoderCount;
        printf("X : %f Y : %f Theta : %f Steertheta:%f velROBO:%f steer count:%d\n",Xodo,Yodo,thetaodo,steertheta,velrobotframe,steercount);
//      printf("ODO %f,%f,%f\n", Xodo, Yodo, thetaodo);
        int bytesread = 0;
          //      char buf[50];
                bytesread = recvfrom(sock,buf,50,0,(struct sockaddr *)&client_addr, &addr_len);
                char command[50];
                sprintf(command,"%lf,%lf,%lf\r",Xodo,Yodo,thetaodo);
                ///printf("\nbytes_read:: %d\n",bytesread);
                sendto(sock,command, strlen(command), 0,(struct sockaddr *)&client_addr, sizeof(struct sockaddr));
                fflush(stdout);
                bzero(command,50);

}


int main(int argc, char *argv[])
{

	tcgetattr( fileno(stdin), &term );    
	signal(SIGTERM, exit_gracefully);		//Ctrl + C handling
	signal(SIGHUP, exit_gracefully);
	signal(SIGINT, exit_gracefully);
	signal(SIGPIPE, exit_gracefully);

	if (argc != 2)
        {
                printf("Usage ./forklift_keyboard <steering offset>");
                return -1;
        }
        steering_offset = atoi(argv[1]);
	
	initudp();
        sleep(2);
	start_roboteq();
	startDIO();
	char ch;
	struct timeval init_time, curr_time;
	int i = 0;
	init_forklift();
	int temp_left = 0;
	int temp_right = 0;	

        encoderCount = readEncoderValue(1);
        prevencoderCount = readEncoderValue(1);

	int isUDPStart = 1;
	isUDPStart = initUDPEncoderData();
	udpthreadrunning = 1;
	pthread_t readThreadObj;
	pthread_create(&readThreadObj, NULL, &readThread,NULL);
	sleep(1);
 
	

	while(1)
	{
		ch = kbhit();
		switch(ch)
		{
			case 65: // Up arrow
				move_forward();
				currentspeed = currentspeed + 5;
				if(currentspeed < 35)
					currentspeed = 35;
				setZoneNormal();
				set_immediate_speed(currentspeed);
				printf("Forward Movement %d\n", currentspeed);
				break;

			case 66: // Down arrow
				move_backward();
				currentspeed = currentspeed + 5;
				if(currentspeed < 35)
					currentspeed = 35;
				set_immediate_speed(currentspeed);
				printf("Reverse Movement %d\n", currentspeed);
				break;

			case 67: // Right arrow
				currentangle = currentangle + 1;
				//currentangle = 60;
				/*
				if(currentangle > 75)
					currentangle = 75;*/
				set_steering_angle(currentangle, steering_offset);
                                printf("Turn Movement %d\n", currentangle);
				break;
			case 68: // Left arrow
				currentangle = currentangle - 1;

				/*if(currentangle < -50)
					currentangle = -50;*/
				set_steering_angle(currentangle, steering_offset);
                                printf("Turn Movement %d\n", currentangle);
				break;

			case 10: //Enter
				setZoneClose();
				currentspeed = 0;
				currentangle = 0;
				set_immediate_speed(0);
			//	set_steering_angle(0, steering_offset);
				stop_lift_movement();
				stop_forklift_movement();
				printf("Stopping\n");
				break;


			case 'p'://lift up
				move_lift_UP();
				getForkTableLimit();
				break;

                        case '#'://lift up
				alpha = getReverseWarning();
				printf("alpha is %d",alpha);
                                break;


			case 'l':
				move_lift_UP();
				while(getForkTableLimit() != 1)
				{
					usleep(100000);
				}
				sleep(1);
				stop_lift_movement();
				break;
			case 'u':
				if(getForkTableLimit() == 1)
					move_lift_DOWN();
				while(getForkTableLimit() == 1)
                                {
                                        usleep(100000);
                                }
                                stop_lift_movement();
                                break;
	
			case 'd'://lift down
				move_lift_DOWN();
				getForkTableLimit();		
			        break;
			
			case 'b'://dome light AUTO MODE
				dio_rw(DOME_LIGHT,1);
				printf("dome light on\n");				
				break;
			
			case 'c'://dome light manual MODE
				dio_rw(DOME_LIGHT,0);
				printf("Dome light off\n");
				break;
			
			case 'k'://key off
				dio_rw(SWITCH,0);
				printf("KEY OFF\n");
				break;

			case 'z': //read DIO
							
					if(load == 0)
					{					
						int curvalue = value;
						move_lift_DOWN();
						while(abs(curvalue - GROUND_HEIGHT) > ENCODER_OFFSET)
						{ 
							curvalue = value;
							usleep(1000);
						}
						stop_lift_movement();
						sleep(1);
						load = 1;
					}
					break;		

	/*	else if(goal_behaviour == 'V')
		{
			speed = 0.0;
			stop_forklift_movement();
			sleep(1);
			int curvalue = value;
			if(forkposition > 2 )
			{
				move_lift_DOWN();
				while(abs(curvalue - DROP_CONVEYOR) > ENCODER_OFFSET)
				{
					curvalue = value;
					usleep(10000);
				}
				stop_lift_movement();
				sleep(1);
				forkposition = 2;
			}
			else if (forkposition < 2)
			{
				move_lift_UP();
				while(abs(curvalue - DROP_CONVEYOR) > ENCODER_OFFSET)
				{
					curvalue = value;
					usleep(10000);
				}
				stop_lift_movement();
				sleep(1);
				forkposition = 2;
			}
			speed = DROP_SPEED;
		}
		else if(goal_behaviour == 'P')   /// pick from ground
		{
			speed = 0.0;
			stop_forklift_movement();
			sleep(1);
			int curvalue = value;
			if(forkposition > 1)
			{
				move_lift_DOWN();
				while(abs(curvalue - GROUND_HEIGHT) >ENCODER_OFFSET)
				{
					curvalue = value;
					usleep(10000);
				}
				stop_lift_movement();
				sleep(1);
				forkposition = 1;
			}
			else if (forkposition < 1)
			{
				move_lift_UP();
				while(abs(curvalue - GROUND_HEIGHT) > ENCODER_OFFSET)
				{
					curvalue = value;
					usleep(10000);
				}
				stop_lift_movement();
				sleep(1);
				forkposition = 1;
			}

			speed = PICK_SPEED;

		}*/
			case 's'://Speed Slow
                                Xodo = 14.328;
                                Yodo = 5.802;
                                thetaodo = -0.044;
                                amclthreadrunning = 1;
				encoderCount = readEncoderValue(1);
			        prevencoderCount = readEncoderValue(1);
                                pthread_create(&odomthread, NULL, &encoderSteerAngle, NULL);
                                sleep(1);
                                pthread_create(&amclthread, NULL, &read_amcl_pose, NULL);
                                usleep(10000);
                                break;

			case 'h': //read steering centre
				
				temp_right = homeSteeringRight();
				temp_left = homeSteeringLeft();
				if(temp_right != 1000 & temp_left != 1000)
					steering_offset = (temp_right + temp_left)/2;
				else if(temp_right != 1000)
					steering_offset = temp_right;
				else
					steering_offset = temp_left;
			
				printf("Steering offset : %d\n", steering_offset);
				break;
			case 'v'://key off
                                currentangle = 0;
                                set_steering_angle(currentangle, steering_offset);
                                printf("Centre\n");
                                sleep(3);
                                resetEncoder();
                                steerpluscount = readEncoderValue(2);
                                printf("centre Count: %d\n",steerpluscount);
                                usleep(100000);
                                break;
			case 'q'://turn right
                                currentangle = 90;
                                set_steering_angle(currentangle, steering_offset);
                                sleep(3);
                                steerpluscount = readEncoderValue(2);
                                printf("Plus Count: %d\n",steerpluscount);
                                break;

                        case 'w'://turn left
                                currentangle = -90;
                                set_steering_angle(currentangle, steering_offset);
                                sleep(3);
                                steerminuscount = readEncoderValue(2);
                                printf("minus Count: %d\n",steerminuscount);
                                break;

                        case '@':
                                warn = getForkProxyLeft();
                                printf("warning %d \n",warn);
                                obs = getForkProxyRight();
                                printf("obstacle %d \n ",obs);
				obs = getPalletStop();
				printf("Pallet Stop %d \n",obs);
                                break;


                        case 'm': 
				warn= getFrontWarning();
                                printf("warning %d \n",warn);
                                obs = getFrontObstacle();
                                printf("obstacle %d \n ",obs);


			case 'r': //read encoder
				printf("Channel 1 %d channel 2 %d\n", readEncoderValue(1), readEncoderValue(2));	
				break;
			case 'o':
				getFrontObstacle();
				getFrontWarning();
				break;
		}
		usleep(1000);
	}
	return 0;	
}

