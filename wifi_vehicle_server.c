
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
#include <stdlib.h>
#include <signal.h>
#include <fcntl.h>
#include <pthread.h>
#include <string.h>

/* socket
 * bind
 * listen
 * accept
 * send/recv
 */
#define SERVER_PORT                             8888
#define BACKLOG                                 10

#define LEDS_DEV                                "/dev/mars_leds"
#define MOTOR_DEV                               "/dev/mars_motor"
#define DS18B20_DEV                             "/dev/mars_ds18b20"
#define ULTRA_DEV                               "/dev/mars_ultrasonic"
#define BUZZER_DEV                              "/dev/mars_buzzer"
#define CAMSERVO_HORIZONTAL_DEV                 "/dev/mars_camServo_horizontal_pwm"
#define CAMSERVO_VERTICAL_DEV                   "/dev/mars_camServo_vertical_pwm"
#define BUZZER                                  0
#define BUZZER_HIGH                             1
#define BUZZER_LOW                              0

#define TEMP_CONVERT_RATIO_FOR_12BIT            0.0625

/* request type */
#define REQ_CMD_TYPE_VECHILE_DIRECTION          0
#define REQ_CMD_TYPE_BUZZER                     1
#define REQ_CMD_TYPE_SPEED                      2
#define REQ_CMD_TYPE_TEMPERATURE                3
#define REQ_CMD_TYPE_CAMSERVO_OPERATION         4
#define REQ_CMD_TYPE_LEDS_ONOFF_OPERATION       5
#define REQ_CMD_TYPE_LEDS_PWM_OPERATION         6

/* Vehicle movements related */
#define WIFI_VEHICLE_MOVE_FORWARD               0
#define WIFI_VEHICLE_MOVE_BACKWARD              1
/* DONOT use number '2', since ioctl is taken this number. If you have to use 2, then use magic number */
/* We don't use ioctl's magic number here, because our app is Windows Qt, it cannot use <sys/ioctl> header file */
#define WIFI_VEHICLE_MOVE_LEFT                  3
#define WIFI_VEHICLE_MOVE_RIGHT                 4
#define WIFI_VEHICLE_STOP                       5

/* Camera servo related */
#define MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE	1
#define MARS_PWM_IOCTL_STOP_OPSCODE		        0

#define MARS_PWM_CAMSERVO_TYPE_HORIZONTAL       0
#define MARS_PWM_CAMSERVO_TYPE_VERTICAL         1

#define MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_70K         700000            // 180°
#define MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_100K        1000000           // 135°
#define MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_140K        1400000           // 90°      
#define MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_180K        1800000           // 45°
#define MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_219K        2190000           // 0°

#define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_120K          1200000           // 180°
#define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_146K          1460000           // 135°
#define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_178K          1780000           // 90°      
#define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_234K          2340000           // 45°
#define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_269K          2690000           // 0°

struct reqMsg {
    unsigned char type;
    unsigned char dir;
    unsigned int  camServoType;
    unsigned int  camServoOpsCode;
    unsigned int  camServoHorizontalDutyNS;
    unsigned int  camServoVerticalDutyNS;
    unsigned char speed;        /* 0 ~ 100 */
    float temp;
};

static int iLedsFd;
static int iMotorBuzzerFd;
static int iDs18b20Fd;
static int iBuzzerFd;
static int iUltraFd;
static int iCamServoHorizontalFd;
static int iCamServoVerticalFd;

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

static void ultraMonitorThread(void)
{
    int i;
    int highlvl_duration[2];
    float gul_distance1 = 0;
    float gul_distance2 = 0;
    float Ultr_Temp1 = 0;
    float Ultr_Temp2 = 0;
        
    memset(highlvl_duration, 0, sizeof(highlvl_duration));

    while (1)
    {
        Ultr_Temp1 = 0;
        Ultr_Temp2 = 0;

        /* 测量5次求平均值的方法来尽量精准测量距离 */
        for (i = 0; i < 5; i++) {
            read(iUltraFd, &highlvl_duration, sizeof(highlvl_duration));
            //printf("[%s] j = %d\n", __FUNCTION__, j);
            Ultr_Temp1 += 340 / 2 * highlvl_duration[0] * 10;                          //模块最大可测距3m
            Ultr_Temp2 += 340 / 2 * highlvl_duration[1] * 10;                          //模块最大可测距3m
            //printf("[%s]%1.4f\n", __FUNCTION__, Ultr_Temp);
        }

        gul_distance1 = Ultr_Temp1 / 5 / 1000000 * 100;               // 乘以100 为了转换为 cm, 不乘100就是米的单位
        gul_distance2 = Ultr_Temp2 / 5 / 1000000 * 100;               // 乘以100 为了转换为 cm, 不乘100就是米的单位
        printf("[%s][Ultra1] %1.4f cm\n", __FUNCTION__, gul_distance1);
        printf("[%s][Ultra2] %1.4f cm\n", __FUNCTION__, gul_distance2);

        if (gul_distance2 < 35) {
            ioctl(iBuzzerFd, BUZZER, BUZZER_HIGH);            
            //ioctl(iMotorBuzzerFd, WIFI_VEHICLE_STOP);
        }else {
            ioctl(iBuzzerFd, BUZZER, BUZZER_LOW);
        }
    }
    
}

static float getTemperature(void)
{
    float temperature = 0;
    unsigned int tmp = 0;

    /*
     * Here we need to read the tmp sixth times, because we need to get the correct temperature value
     */
    read(iDs18b20Fd, &tmp, sizeof(tmp));
    read(iDs18b20Fd, &tmp, sizeof(tmp));
    read(iDs18b20Fd, &tmp, sizeof(tmp));
    read(iDs18b20Fd, &tmp, sizeof(tmp));
    read(iDs18b20Fd, &tmp, sizeof(tmp));
    read(iDs18b20Fd, &tmp, sizeof(tmp));

    //printf("[server] tmp = %d\n", tmp);

    temperature = tmp * TEMP_CONVERT_RATIO_FOR_12BIT;

    //printf("[server] temperature = %1.2f\n", temperature);
    return temperature;
}

/**
 *  @description: control the horizontal camera servo
 *  @params
 *      opsCode: set opsCode to MARS_PWM_IOCTL_SET_DUTYRATIO_OPS indicates that we want to set the dutyNS
 *               set opsCode to MARS_PWM_IOCTL_STOP_OPS indicates that we want to stop the servo
 *  @params
 *      dutyNS: 
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_70K         700000            // 180°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_100K        1000000           // 135°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_140K        1400000           // 90°      
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_180K        1800000           // 45°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_219K        2190000           // 0°
 */
static void set_mars_pwm_camServo_horizontal_ops(unsigned int opsCode, unsigned int dutyNS)
{
    if (opsCode == MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE) {
        // this IOCTL command is the key to set frequency
        int ret = ioctl(iCamServoHorizontalFd, MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE, dutyNS);
        if(ret < 0) {
            printf("Failed to set the frequency of the camServo!\n");
            exit(1);
        }
    }else if (opsCode == MARS_PWM_IOCTL_STOP_OPSCODE) {
        int ret = ioctl(iCamServoHorizontalFd, MARS_PWM_IOCTL_STOP_OPSCODE);
        if(ret < 0) {
            printf("stop the camServo\n");
            exit(1);
        }
    }
}

/**
 *  @description: control the vertical camera servo
 *  @params
 *      opsCode: set opsCode to MARS_PWM_IOCTL_SET_DUTYRATIO_OPS indicates that we want to set the dutyNS
 *               set opsCode to MARS_PWM_IOCTL_STOP_OPS indicates that we want to stop the servo
 *  @params
 *      dutyNS: 
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_120K          1200000             // 180°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_146K          1460000             // 135°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_178K          1780000             // 90°      
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_234K          2340000             // 45°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_269K          2690000             // 0°
 */
static void set_mars_pwm_camServo_vertical_ops(unsigned int opsCode, unsigned int dutyNS)
{
    if (opsCode == MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE) {
        // this IOCTL command is the key to set frequency
        int ret = ioctl(iCamServoVerticalFd, MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE, dutyNS);
        if(ret < 0) {
            printf("Failed to set the frequency of the camServo!\n");
            exit(1);
        }
    }else if (opsCode == MARS_PWM_IOCTL_STOP_OPSCODE) {
        int ret = ioctl(iCamServoVerticalFd, MARS_PWM_IOCTL_STOP_OPSCODE);
        if(ret < 0) {
            printf("stop the camServo\n");
            exit(1);
        }
    }
}

/**
 *  @description: setup the initial state of the horizontal camera servo
 *  @params
 *      opsCode: set opsCode to MARS_PWM_IOCTL_SET_DUTYRATIO_OPS indicates that we want to set the dutyNS
 *               set opsCode to MARS_PWM_IOCTL_STOP_OPS indicates that we want to stop the servo
 *  @params
 *      dutyNS: 
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_70K         700000            // 180°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_100K        1000000           // 135°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_140K        1400000           // 90°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_180K        1800000           // 45°
 *          MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_219K        2190000           // 0°
 */
static void set_mars_pwm_camServo_horizontal_initState(unsigned int opsCode, unsigned int dutyNS)
{
    set_mars_pwm_camServo_horizontal_ops(opsCode, dutyNS);
}

/**
 *  @description: setup the initial state of the vertical camera servo
 *  @params
 *      opsCode: set opsCode to MARS_PWM_IOCTL_SET_DUTYRATIO_OPS indicates that we want to set the dutyNS
 *               set opsCode to MARS_PWM_IOCTL_STOP_OPS indicates that we want to stop the servo
 *  @params
 *      dutyNS: 
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_120K          1200000             // 180°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_146K          1460000             // 135°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_178K          1780000             // 90°      
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_234K          2340000             // 45°
 *          #define MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_269K          2690000             // 0°
 */
static void set_mars_pwm_camServo_vertical_initState(unsigned int opsCode, unsigned int dutyNS)
{
    set_mars_pwm_camServo_vertical_ops(opsCode, dutyNS);
}

static int open_mars_wifi_vehicle_hw_dev(void)
{    
    /* Open leds device */
    iLedsFd = open(LEDS_DEV, O_RDWR);
    if (iLedsFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", LEDS_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", LEDS_DEV);
    }

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

    /* Open buzzer device */
    iBuzzerFd = open(BUZZER_DEV, O_RDWR);
    if (iBuzzerFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", BUZZER_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", BUZZER_DEV);
    }

    /* Open ultrasonic device */
    iUltraFd = open(ULTRA_DEV, O_RDWR);
    if (iUltraFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", ULTRA_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", ULTRA_DEV);
    }

    /* Open camServo horizontal device */
    iCamServoHorizontalFd = open(CAMSERVO_HORIZONTAL_DEV, O_RDWR);
    if (iCamServoHorizontalFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", CAMSERVO_HORIZONTAL_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", CAMSERVO_HORIZONTAL_DEV);
    }

    /* Open camServo vertical device */
    iCamServoVerticalFd = open(CAMSERVO_VERTICAL_DEV, O_RDWR);
    if (iCamServoVerticalFd < 0) {
        printf("[USER]Error: Open %s is failed!\n", CAMSERVO_VERTICAL_DEV);
        return -1;
    }else {
        printf("[USER]Open %s successful!\n", CAMSERVO_VERTICAL_DEV);
    }

    return 0;
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
    pthread_t ultraID;
    int dutyCycle = 99;                  // 0 ~ 100

    iRet = open_mars_wifi_vehicle_hw_dev();
    if (iRet == -1) {
        printf("Open mars wifi vehicle hw device failed!\n");
        return -1;
    }

    /* Setup the camServo initial state */
    set_mars_pwm_camServo_horizontal_initState(MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE, MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_140K);
    set_mars_pwm_camServo_vertical_initState(MARS_PWM_IOCTL_SET_DUTYRATIO_OPSCODE, MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_269K - 40000);
    
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

    memset(&request, 0, sizeof(struct reqMsg));
    request.camServoHorizontalDutyNS    = MARS_PWM_CAMSERVO_HORIZONTAL_DUTY_TIME_140K;
    request.camServoVerticalDutyNS      = MARS_PWM_CAMSERVO_VERTICAL_DUTY_TIME_269K - 40000;
    
	while (1) {
		iAddrLen = sizeof(struct sockaddr);
		iSocketClient = accept(iSocketServer, (struct sockaddr *)&tSocketClientAddr, &iAddrLen);
		if (-1 != iSocketClient) {
			iClientNum++;
			printf("Get connect from client %d : %s\n",  iClientNum, inet_ntoa(tSocketClientAddr.sin_addr));
			if (!fork()) {

                /* create dutyCycle pthread */
                pthread_create(&dutyCycleID, NULL, &dutyCycleThread, &dutyCycle);

                /* create ultrasonic monitoring pthread */
                pthread_create(&ultraID, NULL, (void *)ultraMonitorThread, NULL);

                /* 子进程的源码 */
				while (1) {

                    /* 接收客户端发来的数据并显示出来 */
					iRecvLen = recv(iSocketClient, &request, sizeof(struct reqMsg), 0);
					if (iRecvLen <= 0) {
						close(iSocketClient);
						return -1;
					}else {
					    
					    switch (request.type) {
                        case REQ_CMD_TYPE_VECHILE_DIRECTION:
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

                        case REQ_CMD_TYPE_CAMSERVO_OPERATION:
                            if (request.camServoType == MARS_PWM_CAMSERVO_TYPE_HORIZONTAL)
                                set_mars_pwm_camServo_horizontal_ops(request.camServoOpsCode, request.camServoHorizontalDutyNS);
                            else if (request.camServoType == MARS_PWM_CAMSERVO_TYPE_VERTICAL)
                                set_mars_pwm_camServo_vertical_ops(request.camServoOpsCode, request.camServoVerticalDutyNS);
                            break;

                        case REQ_CMD_TYPE_LEDS_ONOFF_OPERATION:
                            ioctl(iLedsFd, 0, request.dir);         // 0 = cmd, the second argument of the ioctl
                            //printf("Get Msg From Client %d: REQ_CMD_TYPE_LEDS_ONOFF_OPERATION = %d\n", iClientNum, request.dir);
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

    pthread_join(dutyCycleID, NULL);
    pthread_join(ultraID, NULL);
	close(iSocketServer);
	return 0;
}

