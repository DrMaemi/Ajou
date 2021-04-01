/* control_car.c */
#include "kernel.h"
#include "kernel_id.h"
#include "ecrobot_interface.h"

/* Definitions */
#define MOTOR_STEERING     			NXT_PORT_A
#define MOTOR_LEFT         			NXT_PORT_B
#define MOTOR_RIGHT        			NXT_PORT_C
#define S1                 			NXT_PORT_S1
#define S2                 			NXT_PORT_S2
#define S3                 			NXT_PORT_S3
#define S4                			NXT_PORT_S4
#define NEUTRAL_ZONE                8 // degree

#define FAST_SPEED     				70 // MAX_SPEED
#define SLOW_SPEED     				40
#define FAST_STEER     				50 // STEER_RATIO
#define SLOW_STEER     				30
#define FAST_ACCEL     				1  // ACCEL_RATIO
#define SLOW_ACCEL     				1
#define FAST_BRAKE     				3 // BRAKE_RATIO
#define SLOW_BRAKE     				1

#define PI 							3.141593
#define WHEEL_DIAMETER				5.5 // cm
#define FRONT_WHEEL_GEAR_RATIO		2
#define REAR_WHEEL_GEAR_RATIO		4
#define DISTANCE_BETWEEN_AXES		18 // cm

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareTask(TaskInitialize);
DeclareTask(TaskLCD);
DeclareTask(TaskControl);
DeclareTask(TaskDriveForward);
DeclareTask(TaskDriveBackward);
DeclareTask(TaskTurnAxis);
DeclareTask(TaskTurnLeft);
DeclareTask(TaskTurnRight);
DeclareTask(TaskFastSpeed);
DeclareTask(TaskSlowSpeed);
DeclareTask(TaskFastBrake);
DeclareTask(TaskSlowBrake);
DeclareTask(TaskDoBrake);
DeclareTask(TaskSearchParkingSpace);
DeclareTask(TaskDoParking);
DeclareTask(TaskPlayElise);
DeclareTask(TaskAdjustYield);

DeclareEvent(DriveForward);
DeclareEvent(DriveBackward);
DeclareEvent(TurnAxis);
DeclareEvent(TurnLeft);
DeclareEvent(TurnRight);
DeclareEvent(FastSpeed);
DeclareEvent(SlowSpeed);
DeclareEvent(FastBrake);
DeclareEvent(SlowBrake);
DeclareEvent(DoBrake);
DeclareEvent(SearchParkingSpace);
DeclareEvent(DoParking);
DeclareEvent(PlayElise);

DeclareResource(VELOCITY);
DeclareResource(PARKING);

/* STATIC VARIABLES */
static U8 bt_receive_buf[32];

static int MOTOR_LEFT_VELOCITY;
static int MOTOR_RIGHT_VELOCITY;
static int MAX_SPEED;
static int STEER_RATIO;
static int ACCEL_RATIO;
static int BRAKE_RATIO;
static int STATE;
static int axis;
static int STEERING_LIMIT;
static int l_count_max;
static int r_count_max;

static int ENTER_SEARCH;
static int SONAR_DATA[5];
static int SONAR_CNT;
static int SONAR_MEDIAN_VALUE;
static int SEARCH_START;
static int SEARCH_END;
static int WIDTH;

static int PARKING_CONDITION;
static int PARKING_START;
static int PARKING_STEER;
static int HALF;
float TURNING_RADIUS;
float ARC_LENGTH;
float MOVED;

static int DELAY_CNT;
static int pivot;

static int LEFT_CNT_PREV;
static int RIGHT_CNT_PREV;

/* nxtOSEK hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_sonar_sensor(S4);
	ecrobot_init_bt_slave("5-MASTER");
}

void ecrobot_device_terminate()
{
	nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	ecrobot_term_sonar_sensor(S4);
	ecrobot_term_bt_connection();
}

void user_1ms_isr_type2(void)
{
	StatusType ercd;
	ercd = SignalCounter(SysTimerCnt);
	if (ercd != E_OK) { ShutdownOS(ercd); }
}

/***** SUB-FUNCTIONS *****/
S32 FrictionComp(S32 ratio, S32 offset)
{
	if (ratio > 0)
	{
		return ((100-offset)*ratio/100 + offset);
	}
	else if (ratio < 0)
	{
		return ((100-offset)*ratio/100 - offset);
	}
	else
	{
		return ratio;
	}
}

int abs(int num)
{
	return num >= 0 ? num : -num;
}

/* MergeSort */
void merge(int data[], int p, int q, int r)
{
    int i = p, j = q+1, k = p;
    int tmp[5]; // sizeof(SONAR_DATA[5]);
    while(i<=q && j<=r)
	{
        if(data[i] <= data[j]) tmp[k++] = data[i++];
        else tmp[k++] = data[j++];
    }
    while(i<=q)
	{
		tmp[k++] = data[i++];
	}
    while(j<=r)
	{
		tmp[k++] = data[j++];
	}
    for(int a = p; a<=r; a++)
	{
		data[a] = tmp[a];
	}
}

void mergeSort(int data[], int p, int r)
{
    int q;
    if (p<r)
	{
        q = (p+r)/2;
        mergeSort(data, p , q);
        mergeSort(data, q+1, r);
        merge(data, p, q, r);
    }
}
/* MergeSort */

float getRadian(float angle)
{
	return angle * (PI / 180);
}

int factorial(int num)
{
	int result = 1;
	for (int i = 2; i <= num; i++) { result *= i; }
	return result;
}

int getDegree(float radian)
{
	return radian * (float)180 / PI;
}


float pow(float dub, int num)
{
	float result = 1;
	for (int i = 0; i < num; i++)
	{
		result *= dub;
	}
	return result;
}

float sin(float radian)
{
	float result = 0;
	int k;
	for (int i = 1; i <= 5; i++)
	{
		k = 2*i-1;
		if (i % 2) { result += pow(radian, k)/factorial(k); }
		else { result -= pow(radian, k)/factorial(k); }
	}
	return result;
}

void ResetParking()
{
	ENTER_SEARCH = TRUE;
	SONAR_CNT = 0;
	SONAR_MEDIAN_VALUE = 0;
	SEARCH_START = 0;
	SEARCH_END = 0;
	WIDTH = 0;
	PARKING_CONDITION = FALSE;
	PARKING_START = FALSE;
	PARKING_STEER = FALSE;
	HALF = FALSE;
	MOVED = 0;
	pivot = 0;
	DELAY_CNT = 0;
}
/***** SUB-FUNCTIONS *****/


TASK(TaskInitialize)
{
	/* LEFT, RIGHT STEERING CALCULATION & GET AXIS */
	int LR_DONE = FALSE;
	int L_DONE = FALSE;
	int R_DONE = FALSE;
	int l_count_prev, l_count, l_delta;
	int r_count_prev, r_count, r_delta;

	while (!LR_DONE)
	{
		if (!L_DONE)
		{
			nxt_motor_set_speed(MOTOR_STEERING, -30, 1); 
			systick_wait_ms(25);
			l_count = nxt_motor_get_count(MOTOR_STEERING); //왼쪽으로 25ms 회전하고 getCount로 값을 받는다. 더이상 돌지 않을경우 l_count 은 이전 측정했던 값과 같을 것이다.
			l_delta = l_count - l_count_prev;
			if (l_delta != 0) { l_count_prev = l_count; } //  l_count_prev = l_count 이 만족할때까지 while문을 돈다.
			else // l_delta =0 이면 최대로 회전했다고 판단가능하다.
			{
				l_count_max = l_count; // 이때 값은 l_count_max 에 담는다.
				nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
				L_DONE = TRUE;
			}
		}
		if (L_DONE && !R_DONE) // 왼쪽 먼저 측정하고 오른쪽 측정한다.
		{
			nxt_motor_set_speed(MOTOR_STEERING, 30, 1);
			systick_wait_ms(25);
			r_count = nxt_motor_get_count(MOTOR_STEERING);
			r_delta = r_count - r_count_prev;
			if (r_delta != 0) { r_count_prev = r_count; }
			else
			{
				r_count_max = r_count;
				nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
				R_DONE = TRUE;
			}
		}
		if (L_DONE && R_DONE)
		{
			axis = (l_count_max + r_count_max)/2; // 중심축은 2개 더해서 2로 나눈값이다. 이상적일경우 0이 될것이다.
			STEERING_LIMIT = (r_count_max - l_count_max)/2 - 30; // 좌우 회전 최대치를 결정하는 것으로 -30 한것은 바퀴가 너무 돌아가면 회전이 힘들어서 뺀것이다.
			LR_DONE = TRUE;
		}
	}
	MOTOR_LEFT_VELOCITY = 0;
	MOTOR_RIGHT_VELOCITY = 0;
	MAX_SPEED = SLOW_SPEED;
	STEER_RATIO = SLOW_STEER;
	ACCEL_RATIO = SLOW_ACCEL;
	BRAKE_RATIO = SLOW_BRAKE;
	STATE = 0;
	LEFT_CNT_PREV = 0;
	RIGHT_CNT_PREV = 0;
	TURNING_RADIUS = DISTANCE_BETWEEN_AXES / sin(getRadian((float)STEERING_LIMIT/FRONT_WHEEL_GEAR_RATIO)); // 추가기능-자동주차에서 사용되는 최소회전반경을 계산
	ARC_LENGTH = TURNING_RADIUS * getRadian(90); // 추가기능-자동주차에서 필요 후진 회전 거리 계산
	ResetParking();
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	nxt_motor_set_count(MOTOR_LEFT, 0);
	nxt_motor_set_count(MOTOR_RIGHT, 0);
	TerminateTask();
}

TASK(TaskLCD)
{
	int x = 0;
	int y = 1;
	static SINT bt_status = BT_NO_INIT;
	ecrobot_init_bt_slave("5-MASTER");
	if (ecrobot_get_bt_status() == BT_STREAM && bt_status != BT_STREAM)
    {
		display_goto_xy(0, 0);
		display_string("[BT Connected]");
		display_update();
    }

	for (int i=0; i<8; i++)
  	{
    	display_goto_xy(x, y);
    	display_int(bt_receive_buf[i], 0);
    	display_update();
    	x++; if (x == 8) { x = 0; y++; }
  	}

	display_goto_xy(0, 5);
	display_int(nxt_motor_get_count(MOTOR_STEERING), 3);
	display_goto_xy(0, 6);
	display_int(nxt_motor_get_count(MOTOR_LEFT), 3);
	display_goto_xy(0, 7);
	display_int(nxt_motor_get_count(MOTOR_RIGHT), 3);

	display_goto_xy(11, 1);
	display_int(l_count_max, 0);
	display_goto_xy(11, 2);
	display_int(r_count_max, 0);
	display_goto_xy(11, 3);
	display_int(axis, 0);

	display_goto_xy(11, 6);
	display_int((int)MOVED, 3);
	display_goto_xy(11, 7);
	display_int((int)ARC_LENGTH, 3);

	display_update();
	TerminateTask();
}

TASK(TaskControl) // ACTIVATED PER 25ms
{
	ecrobot_read_bt_packet(bt_receive_buf, 32);

	/****** STEERING ******/	
	if (bt_receive_buf[4] == 3) // TURN LEFT
	{
		SetEvent(TaskTurnLeft ,TurnLeft);
	}
	else if (bt_receive_buf[4] == 4) // TURN RIGHT
	{
		SetEvent(TaskTurnRight, TurnRight);
	}
	else // TURN AXIS
	{
		if (bt_receive_buf[7]) // DURING BRAKE OR PARKING
		{
			nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
		}
		else { SetEvent(TaskTurnAxis, TurnAxis); }
	}

	/****** BRAKE ******/
	if (bt_receive_buf[7] == 1) // DO BRAKE
	{
		SetEvent(TaskDoBrake, DoBrake);
		GetResource(PARKING);
		ResetParking();
		ReleaseResource(PARKING);
		TerminateTask();
	}

	/****** PARKING ******/
	if (bt_receive_buf[7] == 2)
	{
		DELAY_CNT++;
		if (!PARKING_CONDITION)
		{
			if (!(DELAY_CNT % 2))
			{
				SetEvent(TaskSearchParkingSpace, SearchParkingSpace);
			}
		}
		else // PARKING CONDITION
		{
			SetEvent(TaskDoParking, DoParking);
			if (DELAY_CNT >= 10) // ACTIVATED PER 250ms (FOR ELISE)
			{
				SetEvent(TaskPlayElise, PlayElise);
				DELAY_CNT = 0;
			}
		}
		TerminateTask();
	}
	else // WHEN 'B' BUTTON IS NOT PRESSED
	{
		GetResource(PARKING);
		ResetParking();
		ReleaseResource(PARKING);
	}

	/****** DRIVE ******/	
	if (bt_receive_buf[3] == 1) // DRIVE FORWARD
	{
		SetEvent(TaskDriveForward, DriveForward);
	}
	else if (bt_receive_buf[3] == 2) // DRIVE BACKWARD
	{
		SetEvent(TaskDriveBackward, DriveBackward);
	}

	/****** SPEED MODE ******/
	if (bt_receive_buf[5] == 1) // FAST SPEED MODE
	{
		SetEvent(TaskFastSpeed, FastSpeed);
	}
	else if (bt_receive_buf[5] == 2) // SLOW SPEED MODE
	{
		SetEvent(TaskSlowSpeed, SlowSpeed);
	}

	/****** BRAKE MODE ******/
	if (bt_receive_buf[6] == 1) // FAST BRAKE MODE
	{
		SetEvent(TaskFastBrake, FastBrake);
	}
	else if (bt_receive_buf[6] == 2) // SLOW BRAKE MODE
	{
		SetEvent(TaskSlowBrake, SlowBrake);
	}

	TerminateTask();
}

TASK(TaskTurnLeft)
{
	while (1)
	{
		WaitEvent(TurnLeft);
		ClearEvent(TurnLeft);
		if (nxt_motor_get_count(MOTOR_STEERING) > (axis - STEERING_LIMIT))
		{
			nxt_motor_set_speed(MOTOR_STEERING, -STEER_RATIO, 1);
		}
		else
		{
			nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
		}
	}
	TerminateTask();
}

TASK(TaskTurnRight)
{
	while (1)
	{
		WaitEvent(TurnRight);
		ClearEvent(TurnRight);
		if (nxt_motor_get_count(MOTOR_STEERING) < (axis + STEERING_LIMIT))
		{
			nxt_motor_set_speed(MOTOR_STEERING, STEER_RATIO, 1);
		}
		else
		{
			nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
		}
	}
	TerminateTask();
}

TASK(TaskTurnAxis)
{
	while (1)
	{
		WaitEvent(TurnAxis);
		ClearEvent(TurnAxis);
		if (nxt_motor_get_count(MOTOR_STEERING) > (axis + NEUTRAL_ZONE))
		{
			nxt_motor_set_speed(MOTOR_STEERING, -(STEER_RATIO - ACCEL_RATIO), 1);		
		}
		else if (nxt_motor_get_count(MOTOR_STEERING) < (axis - NEUTRAL_ZONE))
		{
			nxt_motor_set_speed(MOTOR_STEERING, (STEER_RATIO - ACCEL_RATIO), 1);
		}
		else
		{
			nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
		}
	}
	TerminateTask();
}

TASK(TaskDriveForward)
{
	while (1)
	{
		WaitEvent(DriveForward);
		ClearEvent(DriveForward);
		GetResource(VELOCITY);
		if (MOTOR_RIGHT_VELOCITY > 0) // During DriveBackward
		{
			STATE = 1;
			MOTOR_LEFT_VELOCITY -= ACCEL_RATIO;
			MOTOR_RIGHT_VELOCITY -= ACCEL_RATIO;
		}
		else
		{
			STATE = 2;
			if (MOTOR_RIGHT_VELOCITY > -MAX_SPEED)
			{
				MOTOR_LEFT_VELOCITY -= ACCEL_RATIO;
				MOTOR_RIGHT_VELOCITY -= ACCEL_RATIO;
			}
			else
			{
				MOTOR_LEFT_VELOCITY += ACCEL_RATIO;
				MOTOR_RIGHT_VELOCITY += ACCEL_RATIO;
			}
		}
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY);
	}
	TerminateTask();
}

TASK(TaskDriveBackward)
{
	while (1)
	{
		WaitEvent(DriveBackward);
		ClearEvent(DriveBackward);
		GetResource(VELOCITY);
		if (MOTOR_RIGHT_VELOCITY < 0) // During DriveForward
		{
			STATE = 3;
			MOTOR_LEFT_VELOCITY += ACCEL_RATIO;
			MOTOR_RIGHT_VELOCITY += ACCEL_RATIO;
		}
		else
		{
			STATE = 4;
			if (MOTOR_RIGHT_VELOCITY < MAX_SPEED)
			{
				MOTOR_LEFT_VELOCITY += ACCEL_RATIO;
				MOTOR_RIGHT_VELOCITY += ACCEL_RATIO;	
			}
			else
			{
				MOTOR_LEFT_VELOCITY -= ACCEL_RATIO;
				MOTOR_RIGHT_VELOCITY -= ACCEL_RATIO;
			}
		}
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY);
	}
	TerminateTask();
}

TASK(TaskDoBrake)
{
	while (1)
	{
		WaitEvent(DoBrake);
		ClearEvent(DoBrake);
		GetResource(VELOCITY);
		if (MOTOR_RIGHT_VELOCITY < -BRAKE_RATIO) // During DriveForward
		{
			MOTOR_LEFT_VELOCITY += BRAKE_RATIO;
			MOTOR_RIGHT_VELOCITY += BRAKE_RATIO;
		}
		else if (MOTOR_RIGHT_VELOCITY > BRAKE_RATIO) // During DriveBackward
		{
			MOTOR_LEFT_VELOCITY -= BRAKE_RATIO;
			MOTOR_RIGHT_VELOCITY -= BRAKE_RATIO;
		}
		else
		{
			MOTOR_LEFT_VELOCITY = 0;
			MOTOR_RIGHT_VELOCITY = 0;
		}
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY);
	}
	TerminateTask();
}

/****** SPEED MODE ******/
TASK(TaskFastSpeed)
{
	while (1)
	{
		WaitEvent(FastSpeed);
		ClearEvent(FastSpeed);
		MAX_SPEED = FAST_SPEED;
		STEER_RATIO = FAST_STEER;
		ACCEL_RATIO = FAST_ACCEL;
	}
	TerminateTask();
}

TASK(TaskSlowSpeed)
{
	while (1)
	{
		WaitEvent(SlowSpeed);
		ClearEvent(SlowSpeed);
		MAX_SPEED = SLOW_SPEED;
		STEER_RATIO = SLOW_STEER;
		ACCEL_RATIO = SLOW_ACCEL;
	}
	TerminateTask();
}
/****** SPEED MODE ******/

/****** BRAKE MODE ******/
TASK(TaskFastBrake)
{
	while (1)
	{
		WaitEvent(FastBrake);
		ClearEvent(FastBrake);
		BRAKE_RATIO = FAST_BRAKE;	
	}
	TerminateTask();
}

TASK(TaskSlowBrake)
{
	while (1)
	{
		WaitEvent(SlowBrake);
		ClearEvent(SlowBrake);
		BRAKE_RATIO = SLOW_BRAKE;	
	}
	TerminateTask();
}
/****** BRAKE MODE ******/

TASK(TaskSearchParkingSpace)
{
	while (1)
	{
		WaitEvent(SearchParkingSpace);
		ClearEvent(SearchParkingSpace);

		GetResource(PARKING); // 자동 주차와 관련된 전역 변수에 세마포를 설정한다.
		GetResource(VELOCITY); // 속력에 대해 세마포를 설정한다.
		MOTOR_LEFT_VELOCITY = -50; // 전진하며 탐색하기 위해 속력을 전진으로 설정
		MOTOR_RIGHT_VELOCITY = -50;
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY); // 속력에 대해 세마포를 해제한다.

		if (ENTER_SEARCH) // B버튼을 눌러 첫 주차 공간 탐색을 수행하는 경우
		{
			SEARCH_START = nxt_motor_get_count(MOTOR_RIGHT); // 현재 후륜 모터의 회전각을 저장
			ENTER_SEARCH = FALSE;
		}

		SONAR_DATA[SONAR_CNT] = ecrobot_get_sonar_sensor(S4); // 초음파 센서 배열에 초음파 측정 값을 저장
		SONAR_CNT++;
		if (SONAR_CNT == 5) // 배열이 모두 채워진 경우
		{
			mergeSort(SONAR_DATA, 0, 4); // 합병 정렬
			SONAR_MEDIAN_VALUE = SONAR_DATA[2]; // 중앙값 추출
			SONAR_CNT = 0;
			if (SONAR_MEDIAN_VALUE >= 40) // 산출된 초음파 측정 값이 40(cm) 이상인 경우
			{
				SEARCH_END = nxt_motor_get_count(MOTOR_RIGHT);
				// 현재 모터의 회전각 측정
				WIDTH = abs(SEARCH_START - SEARCH_END); // 후륜 모터의 회전각 변화량 계산
				if (WIDTH >= 156) { PARKING_CONDITION = TRUE; }
				/* 회전각 변화량이 156도 이상이면 차체가 30cm 이상 전진한 것.
				차체가 30cm 이상 전진하는 동안 산출된 초음파 측정 값이 지속적으로 40 이상인 경우
				가로 30cm, 세로 40cm의 충분한 주차 공간이 있다고 판단. */ 
			}
			else // 중간에 초음파 측정 값이 40 미만이 되는 경우
			{
				SEARCH_START = nxt_motor_get_count(MOTOR_RIGHT);
				// 주차 공간이 없으므로 후륜 모터의 회전각 변화량이 0이 되도록 구현
			}
		}
			
		ReleaseResource(PARKING);
	}
	TerminateTask();
}

TASK(TaskDoParking)
{
	int BACKWARD_CNT_PREV;
	int BACKWARD_START, BACKWARD_END;
	int BACKWARD_DELTA;
	while (1)
	{
		WaitEvent(DoParking);
		ClearEvent(DoParking);

		GetResource(PARKING);
		GetResource(VELOCITY);
		if (!PARKING_START) // 주차 1단계, 차량 정지가 완료되지 않은 경우
		{
			if (MOTOR_RIGHT_VELOCITY < -10)
			{
				MOTOR_LEFT_VELOCITY += 4;
				MOTOR_RIGHT_VELOCITY += 4;
			}
			else if (MOTOR_RIGHT_VELOCITY > 10)
			{
				MOTOR_LEFT_VELOCITY -= 4;
				MOTOR_RIGHT_VELOCITY -= 4;
			}
			else
			{
				MOTOR_LEFT_VELOCITY = 0;
				MOTOR_RIGHT_VELOCITY = 0;
				PARKING_START = TRUE;
			}
			// 차량을 감속하여 정지시킨다.
		}
		else // 주차 1단계, 차량 정지가 완료된 경우
		{
			if (!PARKING_STEER) // 주차 2단계, 스티어링 휠 조작이 완료되지 않은 경우
			{
				if (nxt_motor_get_count(MOTOR_STEERING) > axis - STEERING_LIMIT)
				{
					SetEvent(TaskTurnLeft, TurnLeft);
					// 좌향 최대 각으로 조정
				}
				else
				{
					nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
					BACKWARD_CNT_PREV = nxt_motor_get_count(MOTOR_RIGHT);
					// 적절한 후진 속력을 얻기 위해 단위시간 전 후륜 모터의 회전각 저장
					BACKWARD_START = BACKWARD_CNT_PREV;
					// 후진 회전 거리를 측정하기 위해 주차 시작 시 후륜 모터 회전각 저장
					PARKING_STEER = TRUE;
				}
			}
			else // 주차 2단계, 스티어링 휠 조작이 완료된 경우
			{
				if (!HALF) // 필요한 후진 회전 거리의 절반만큼 후진하지 않은 경우
				{
					BACKWARD_END = nxt_motor_get_count(MOTOR_RIGHT);
					// // 현재 후륜 모터 회전각 측정
					BACKWARD_DELTA = abs(BACKWARD_END-BACKWARD_CNT_PREV);
					if (BACKWARD_DELTA < 7) // 적절한 후진 속력에 도달하지 않은 경우
					{
						MOTOR_LEFT_VELOCITY += 1;
						MOTOR_RIGHT_VELOCITY += 1;
						// 후진 속력을 높힌다
					}
					BACKWARD_CNT_PREV = BACKWARD_END;
					// 다음 회전각 변화량을 측정하기 위해 현재 회전각을 단위시간 전 회전각으로 저장
					BACKWARD_DELTA = abs(BACKWARD_END-BACKWARD_START);
					// 주차 시작 후 후륜 모터의 회전각이 얼마나 변화했는지 측정
					MOVED = (WHEEL_DIAMETER/2)*getRadian(REAR_WHEEL_GEAR_RATIO*BACKWARD_DELTA);
					// 호의 길이 공식 사용, 주차 시작 후 얼마나 후진했는지 측정
					if (MOVED >= ARC_LENGTH/2) { HALF = TRUE; }
				}
				else // 필요한 후진 회전 거리의 절반 이상 후진한 경우
				{
					if (MOTOR_RIGHT_VELOCITY > 10)
					{
						MOTOR_LEFT_VELOCITY -= 1;
						MOTOR_RIGHT_VELOCITY -= 1;
						// 천천히 감속
					}
					else
					{
						MOTOR_LEFT_VELOCITY = 0;
						MOTOR_RIGHT_VELOCITY = 0;
						SetEvent(TaskTurnAxis, TurnAxis);
						// 주차 완료, 스티어링 휠을 정향으로
					}
				}
			}
		}
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY);
		ReleaseResource(PARKING);
	}
	TerminateTask();
}

TASK(TaskPlayElise)
{
	int Elise[24][3] =
	{
		{1319, 250, 20}, {1245, 250, 20}, {1319, 250, 20}, {1245, 250, 20},
		{1319, 250, 20}, {988, 250, 20}, {1175, 250, 20}, {1047, 250, 20},
		{880, 250, 20}, {880, 250, 20}, {0, 250, 0}, {523, 250, 20},
		{659, 250, 20}, {880, 250, 20}, {988, 250, 20}, {988, 250, 20}, 
		{0, 250, 0},	{659, 250, 20}, {831, 250, 20}, {988, 250, 20},
		{1047, 250, 20}, {1047, 250, 20}, {0, 250, 0}, {659, 250, 20}
	};
	while (1)
	{
		WaitEvent(PlayElise);
		ClearEvent(PlayElise);
		ecrobot_sound_tone
				(
					Elise[pivot][0],
					Elise[pivot][1],
					Elise[pivot][2]
				);
		pivot++;
		if (pivot == 24) { pivot = 0; }
	}
	TerminateTask();
}

TASK(TaskAdjustYield)
{
	int LEFT_CNT = nxt_motor_get_count(MOTOR_LEFT);
	int RIGHT_CNT = nxt_motor_get_count(MOTOR_RIGHT);
	int LEFT_CNT_DELTA = abs(LEFT_CNT - LEFT_CNT_PREV);
	int RIGHT_CNT_DELTA = abs(RIGHT_CNT - RIGHT_CNT_PREV);
	int GAP = LEFT_CNT_DELTA - RIGHT_CNT_DELTA;          // GAP>0 이면 왼쪽 모터가 빠름 , GAP<0 이면 오른쪽 모터가 빠름 
	int STEERING_CNT = nxt_motor_get_count(MOTOR_STEERING);
	GetResource(VELOCITY);
	if (MOTOR_RIGHT_VELOCITY != 0)
	{
		if ((axis-NEUTRAL_ZONE) < STEERING_CNT  
			&& STEERING_CNT < (axis+NEUTRAL_ZONE)) // 중심축에 있을경우 실행
		{
			if (GAP > 1) 
			{
				if (STATE == 1 || STATE == 2)  //전진일경우 왼쪽 모터가 더빨라서 왼쪽모터의 속도를 줄인다.
				{
					MOTOR_LEFT_VELOCITY++;
				}
				if (STATE == 3 || STATE == 4) //후진일경우 왼쪽 모터가 더빨라서 왼쪽모터의 속도를 줄인다.
				{
					MOTOR_LEFT_VELOCITY--;
				}
			}
			else if (GAP < -1)
			{
				if (STATE == 1 || STATE == 2) //전진일경우 오른쪽 모터가 더빨라서 왼쪽모터의 속도를 올린다.
				{
					MOTOR_LEFT_VELOCITY--;
				}
				if (STATE == 3 || STATE == 4) //후진일경우 오른쪽 모터가 더빨 라서 왼쪽모터의 속도를 올린다.
				{
					MOTOR_LEFT_VELOCITY++;
				}
			}
		}
	}
	nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
	ReleaseResource(VELOCITY);

	LEFT_CNT_PREV = LEFT_CNT;
	RIGHT_CNT_PREV = RIGHT_CNT;
	TerminateTask();
}
