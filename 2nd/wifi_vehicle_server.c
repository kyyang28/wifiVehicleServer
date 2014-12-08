

#include <sys/types.h>          /* See NOTES */
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>

/* socket
 * bind
 * listen
 * accept
 * send/recv
 */
#define SERVER_PORT                         8888
#define BACKLOG                             10

#define MOTOR_DEV                           "/dev/mars_motor"
#define DS18B20_DEV                         "/dev/mars_ds18b20"

#define TEMP_CONVERT_RATIO_FOR_12BIT        0.0625

/* request type */
#define REQ_CMD_TYPE_DIRECTION              0
#define REQ_CMD_TYPE_BUZZER                 1
#define REQ_CMD_TYPE_SPEED                  2
#define REQ_CMD_TYPE_TEMPERATURE            3

#define WIFI_VEHICLE_MOVE_FORWARD           0
#define WIFI_VEHICLE_MOVE_BACKWARD          1
#define WIFI_VEHICLE_MOVE_LEFT              2
#define WIFI_VEHICLE_MOVE_RIGHT             3
#define WIFI_VEHICLE_STOP                   4

#define WIFI_VEHICLE_BUZZER_ON              5
#define WIFI_VEHICLE_BUZZER_OFF             6

struct reqMsg {
    unsigned char type;
    unsigned char dir;
    unsigned char speed;        /* 0 ~ 100 */
    float temp;
};

int iMotorBuzzerFd;
int iDs18b20Fd;

static void *dutyCycleThread(void *arg)
{
    int dutyCycle;                  // 0 ~ 100
    char val;

    /* Change the usleep value to manually control the duty cycle */
    while (1) {
        dutyCycle = *(int *)arg;

        val = 1;
        write(iMotorBuzzerFd, &val, 1);

        usleep(dutyCycle*1000);

        val = 0;
        write(iMotorBuzzerFd, &val, 1);        
        usleep(100*1000 - dutyCycle*1000);
    }

    return arg;
}

static float getTemperature(void)
{
    float temperature;
    unsigned int tmp = 0;

    read(iDs18b20Fd, &tmp, sizeof(tmp));

    temperature = tmp * TEMP_CONVERT_RATIO_FOR_12BIT;
    
    return temperature;
}

int main(int argc, char *argv[])
{
	int iSocketServer;
	int iSocketClient;
	struct sockaddr_in tSocketServerAddr;
	struct sockaddr_in tSocketClientAddr;
	int iRet;
	int iSendLen;
	socklen_t iAddrLen;
	int iRecvLen;
	int iClientNum = -1;
    struct reqMsg request;
    pthread_t dutyCycleID;
    int dutyCycle = 30;                  // 0 ~ 100

    /* Open motor device */
    iMotorBuzzerFd = open(MOTOR_DEV, O_RDWR);
    if (iMotorBuzzerFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", MOTOR_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", MOTOR_DEV);
    }

    /* Open ds18b20 device */
    iDs18b20Fd = open(DS18B20_DEV, O_RDWR);
    if (iDs18b20Fd < 0) {
        printf("[USER]Error: Open %s is failed!\n", DS18B20_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", DS18B20_DEV);
    }

	signal(SIGCHLD, SIG_IGN);
	
	iSocketServer = socket(AF_INET, SOCK_STREAM, 0);
	if (-1 == iSocketServer) {
		printf("socket error!\n");
		return -1;
	}

	tSocketServerAddr.sin_family      = AF_INET;
	tSocketServerAddr.sin_port        = htons(SERVER_PORT);  /* host to net, short */
 	tSocketServerAddr.sin_addr.s_addr = INADDR_ANY;
	memset(tSocketServerAddr.sin_zero, 0, 8);
	
	iRet = bind(iSocketServer, (const struct sockaddr *)&tSocketServerAddr, sizeof(struct sockaddr));
	if (-1 == iRet) {
		printf("bind error!\n");
		return -1;
	}

	iRet = listen(iSocketServer, BACKLOG);
	if (-1 == iRet) {
		printf("listen error!\n");
		return -1;
	}

	while (1) {
		iAddrLen = sizeof(struct sockaddr);
		iSocketClient = accept(iSocketServer, (struct sockaddr *)&tSocketClientAddr, &iAddrLen);
		if (-1 != iSocketClient) {
			iClientNum++;
			printf("Get connect from client %d : %s\n",  iClientNum, inet_ntoa(tSocketClientAddr.sin_addr));
			if (!fork()) {

                /* create pthread */
                pthread_create(&dutyCycleID, NULL, &dutyCycleThread, &dutyCycle);

                /* 子进程的源码 */
				while (1) {

                    /* 接收客户端发来的数据并显示出来 */
					iRecvLen = recv(iSocketClient, &request, sizeof(struct reqMsg), 0);
					if (iRecvLen <= 0) {
						close(iSocketClient);
						return -1;
					}else {
					    
					    switch (request.type) {
                        case REQ_CMD_TYPE_DIRECTION:
                            ioctl(iMotorBuzzerFd, request.dir);
                            //printf("Get Msg From Client %d: REQ_CMD_TYPE_DIRECTION = %d\n", iClientNum, request.dir);
                            break;

                        case REQ_CMD_TYPE_BUZZER:
                            ioctl(iMotorBuzzerFd, request.dir);
                            //printf("Get Msg From Client %d: REQ_CMD_TYPE_BUZZER = %d\n", iClientNum, request.dir);
                            break;

                        case REQ_CMD_TYPE_SPEED:
                            dutyCycle = request.speed;          // request.speed = duty cycle
                            //printf("Get Msg From Client %d: REQ_CMD_TYPE_SPEED = %d\n", iClientNum, request.speed);
                            break;

                        case REQ_CMD_TYPE_TEMPERATURE:
                            /* Acquire the temperature from ds18b20 */
                            request.temp = getTemperature();
                            //printf("Get Msg From Client %d: REQ_CMD_TYPE_TEMPERATURE\n", iClientNum);

                            /* send the temperature to the client */
                            iSendLen = send(iSocketClient, &request, sizeof(struct reqMsg), 0);
                            if (iSendLen <= 0) {
                                close(iSocketClient);
                                return -1;
                            }
                            
                            break;

                        default:
                            printf("No such request type!\n");
                            break;
					    }                        
					}
				}				
			}
		}
	}
	
	close(iSocketServer);
	return 0;
}



