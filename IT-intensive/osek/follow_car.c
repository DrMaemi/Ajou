/* follow_car.c */
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
#define S4                 			NXT_PORT_S4
#define NEUTRAL_ZONE                8 // degree
#define SLACK						1

#define PI 							3.141593

#define MAX_BUFF_SIZE				3
#define Sonar_Distance 				10
#define HEIGHT_WEIGHT				2.5

/* OSEK declarations */
DeclareCounter(SysTimerCnt);
DeclareTask(TaskInitialize);
DeclareTask(TaskLCD);
DeclareTask(TaskSonarSensor);
DeclareTask(TaskSteerCtrl);
DeclareTask(TaskDistanceCtrl);

DeclareEvent(SonarSensor);
DeclareEvent(SteerCtrl);
DeclareEvent(DistanceCtrl);

DeclareResource(VELOCITY);

/* STATIC VARIABLES */
static int MOTOR_LEFT_VELOCITY;
static int MOTOR_RIGHT_VELOCITY;
static int axis;
static int STEERING_LIMIT;
static int l_count_max;
static int r_count_max;

static int DELAY_CNT;
static int START;

static int sonarBuff_left[MAX_BUFF_SIZE]; //CIRCULAR BUFFER FOR LEFT SONARSENSOR
static int sonarBuff_right[MAX_BUFF_SIZE]; //CIRCULAR BUFFER FOR RIGHT SONARSENSOR
static int sonarBuff_cursor1 = 0; 
static int sonarBuff_cursor2 = 0;
static float sonar_left_avrg;
static float sonar_right_avrg;
static int theta;
static int sonar_error_count_l;
static int sonar_error_count_r;
static float ARCTAN_WEIGHT;
static int sonar_left;
static int sonar_right;
static float DIST_DELTA;
static float DIST_PREV;
static float DIST;


/* nxtOSEK hooks */
void ecrobot_device_initialize()
{
	ecrobot_init_sonar_sensor(S1);
	ecrobot_init_sonar_sensor(S4);
}

void ecrobot_device_terminate()
{
	ecrobot_term_sonar_sensor(S1);
	ecrobot_term_sonar_sensor(S4);
	nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
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

float min (float a, float b)
{
	if (a > b) { return b; }
	else { return a; }
}

int abs(int num)
{
	return num >= 0 ? num : -num;
}

float getRadian(int angle)
{
	return angle * (PI / 180);
}

int getDegree(float radian)
{
	return radian * ((float)180 / PI);
}

int factorial(int num)
{
	int result = 1;
	for (int i = 2; i <= num; i++) { result *= i; }
	return result;
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

float arctan(float x)
{
	if (x == 1) { return getRadian(45); }
	if (x == -1) { return -getRadian(45); }
	if (x > 1)
	{
		return getRadian(45) + arctan((x-1)/(x+1));
	}
	if (x < -1)
	{
		return -(getRadian(45) + arctan((x+1)/(x-1)));
	}
	float result = 0;
	int k;
	for (int i = 1; i <= 5; i++)
	{
		k = 2*i-1;
		if (i % 2) { result += pow(x, k)/k; }
		else { result -= pow(x, k)/k; }
	}
	return result;
}

void goForward(void)
{
	if (MOTOR_RIGHT_VELOCITY > -40)
	{
		MOTOR_LEFT_VELOCITY = -40;
		MOTOR_RIGHT_VELOCITY = -40;
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
	}
}

void doStop(void)
{
	MOTOR_LEFT_VELOCITY = 0;
	MOTOR_RIGHT_VELOCITY = 0;
	nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
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
			nxt_motor_set_speed(MOTOR_STEERING, -22, 1);
			systick_wait_ms(30);
			l_count = nxt_motor_get_count(MOTOR_STEERING); //왼쪽으로 25ms 회전하고 getCount로 값을 받는다. 더이상 돌지 않을경우 l_count 은 이전 측정했던 값과 같을 것이다.
			l_delta = l_count - l_count_prev;
			if (l_delta != 0) { l_count_prev = l_count; } //  l_count_prev = l_count 이 만족할때까지 while문을 돈다.
			else  // l_delta =0 이면 최대로 회전했다고 판단가능하다.
			{
				l_count_max = l_count; // 이때 값은 l_count_max 에 담는다.
				nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
				L_DONE = TRUE;
			}
		}
		if (L_DONE && !R_DONE) // 왼쪽 먼저 측정하고 오른쪽 측정한다.
		{
			nxt_motor_set_speed(MOTOR_STEERING, 22, 1);
			systick_wait_ms(30);
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
	nxt_motor_set_speed(MOTOR_LEFT, 0, 1);
	nxt_motor_set_speed(MOTOR_RIGHT, 0, 1);
	nxt_motor_set_count(MOTOR_LEFT, 0);
	nxt_motor_set_count(MOTOR_RIGHT, 0);
	DIST_PREV = 0;
	DIST = 0;
	START = FALSE;
	sonar_error_count_l = 0;
	sonar_error_count_r = 0;
	ARCTAN_WEIGHT = 0.25;
	TerminateTask();
}

TASK(TaskLCD)
{
	display_goto_xy(0, 2);
	display_int(sonar_error_count_l, 3);
	display_goto_xy(0, 3);
	display_int(sonar_error_count_r, 3);
	display_goto_xy(0, 4);
	display_int(sonar_right_avrg, 3);
	display_goto_xy(0, 5);
	display_int(sonar_left, 3);
	display_goto_xy(0, 6);
	display_int(sonar_right, 3);
	display_goto_xy(0, 7);
	display_int(theta, 3);
	display_update();
	TerminateTask();
}

TASK(TaskControl)
{
	DELAY_CNT++;
	SetEvent(TaskSteerCtrl, SteerCtrl);
	if (DELAY_CNT == 5)
	{
		DELAY_CNT = 0;
		SetEvent(TaskSonarSensor, SonarSensor);
		SetEvent(TaskDistanceCtrl, DistanceCtrl);
	}
	if (DELAY_CNT == 1000) { DELAY_CNT = 0; }
	TerminateTask();
}

TASK(TaskSteerCtrl) 
{
	int STEERED, STEER_THETA;
	int LEFT_SLACK, RIGHT_SLACK;
	while (1)
	{
		WaitEvent(SteerCtrl);
		ClearEvent(SteerCtrl);
		STEERED = nxt_motor_get_count(MOTOR_STEERING);
		STEER_THETA = 2*theta;
		LEFT_SLACK = axis + STEER_THETA - SLACK;
		RIGHT_SLACK = axis + STEER_THETA + SLACK;
		if (STEER_THETA < 0) // TURN LEFT
		{
			if (LEFT_SLACK < STEERED && STEERED < RIGHT_SLACK) // 중립 범위
			{
				nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
			}
			else if (RIGHT_SLACK <= STEERED) // 측정된 각도보다 덜 좌향되어 있는 경우
			{

				if (STEERED > axis - STEERING_LIMIT) // 좌향 최대 가동 범위에 도달하지 않았다면
				{
					nxt_motor_set_speed(MOTOR_STEERING, -40, 1);
				}
				else { nxt_motor_set_speed(MOTOR_STEERING, 0, 1); }
			}
			else if (STEERED <= LEFT_SLACK) // 측정된 각도보다 더 좌향되어 있는 경우
			{
				nxt_motor_set_speed(MOTOR_STEERING, 40, 1);
			}
		}
		else if (STEER_THETA > 0) // TURN RIGHT
		{
			if (LEFT_SLACK < STEERED && STEERED < RIGHT_SLACK) // 중립 범위
			{
				nxt_motor_set_speed(MOTOR_STEERING, 0, 1);
			}
			else if (STEERED <= LEFT_SLACK) // 측정된 각도보다 덜 우향되어 있는 경우
			{
				if (STEERED < axis + STEERING_LIMIT) // 우향 최대 가동 범위에 도달하지 않았다면
				{
					nxt_motor_set_speed(MOTOR_STEERING, 40, 1);
				}
				else { nxt_motor_set_speed(MOTOR_STEERING, 0, 1); }
			}
			else if (RIGHT_SLACK <= STEERED) // 측정된 각도보다 더 우향되어 있는 경우
			{
				nxt_motor_set_speed(MOTOR_STEERING, -40, 1);
			}
		} // THETA == 0
		else { nxt_motor_set_speed(MOTOR_STEERING, 0, 1); }
	}
	TerminateTask();	
}

TASK(TaskDistanceCtrl)
{
	while (1)
	{
		WaitEvent(DistanceCtrl);
		ClearEvent(DistanceCtrl);
		DIST_DELTA = DIST - DIST_PREV;

		GetResource(VELOCITY);
		if (START)
		{
			if (DIST < 26) // 거리가 가까운 경우
			{
				if (MOTOR_RIGHT_VELOCITY < -15) //전진인 경우 속도를 -15로 감소시킨다.
				{
					MOTOR_LEFT_VELOCITY = -15;
					MOTOR_RIGHT_VELOCITY = -15;
				}
				else if (MOTOR_RIGHT_VELOCITY < -10) // -15로 감소한 이후 천천히 감소시킨다.
				{
					MOTOR_LEFT_VELOCITY++;
					MOTOR_RIGHT_VELOCITY++;
				}
				else if (MOTOR_RIGHT_VELOCITY < 0) // 정지
				{
					MOTOR_LEFT_VELOCITY = 0;
					MOTOR_RIGHT_VELOCITY = 0;
				}
				else // 후진인 경우
				{
					if (DIST < 15)   // 거리가 15미만이면 가까운 거리여서 후진 속도를 증가
					{
						MOTOR_LEFT_VELOCITY++;
						MOTOR_RIGHT_VELOCITY++;
					}
					else // 거리가 15 이상인 경우
					{
						if (MOTOR_RIGHT_VELOCITY < 60)  
						{
							if (DIST_DELTA > 0) // (현재 거리 - 이전거리)가 0보다 크다는것은 이전보다 거리가 벌어졌다는 것이어서 속도를 높힌다. 
							{
								MOTOR_LEFT_VELOCITY++;
								MOTOR_RIGHT_VELOCITY++;
							}
							else if (DIST_DELTA < 0) // (현재 거리 - 이전거리)가 0보다 작다는것은 이전보다 거리가 줄어들었다는 것이어서 속도를 낮춘다. 
							{
								MOTOR_LEFT_VELOCITY--;
								MOTOR_RIGHT_VELOCITY--;
							}
						}
					}
				}
			}
			else if (DIST < 31) // 26<DIST<31
			{
				if (DIST_DELTA > 0) // (현재 거리 - 이전거리)>0 이면 이전보다 앞차와의 거리가 늘어난 것이어서 속도를 증가
				{
					MOTOR_LEFT_VELOCITY--;
					MOTOR_RIGHT_VELOCITY--;
				}
				else if (DIST_DELTA < 0) // (현재 거리 - 이전거리)<0 이면 이전보다 앞차와의 거리가 줄어든 것이어서 속도를 감소
				{
					MOTOR_LEFT_VELOCITY++;
					MOTOR_RIGHT_VELOCITY++;
				}
			}
			else //  31 <= DIST
			{
				if (MOTOR_RIGHT_VELOCITY >= 0) //전진인경우 
				{
					MOTOR_LEFT_VELOCITY = -15;
					MOTOR_RIGHT_VELOCITY = -15;
				}
				if (MOTOR_RIGHT_VELOCITY > -60) // 후진인경우
				{
					if (DIST_DELTA >= 0.1)   // 거리가 크게 벌어진거 별로 감속하는 계수, 거리가 많이 벌어지면 감속계수가 커지고 거리가 조금 벌어지면 감속계수가 작아져서 31의 거리를 유지
					{
						MOTOR_LEFT_VELOCITY -= 3;
						MOTOR_RIGHT_VELOCITY -= 3;
					}
					else if (DIST_DELTA >= 0.05)
					{
						MOTOR_LEFT_VELOCITY -= 2;
						MOTOR_RIGHT_VELOCITY -= 2;
					}
					else if (DIST_DELTA >= 0.025)
					{
						MOTOR_LEFT_VELOCITY--;
						MOTOR_RIGHT_VELOCITY--;
					}
					else if (DIST_DELTA < 0) // 거리가 줄어들었을시 속도를 감속시킨다.
					{
						if (MOTOR_RIGHT_VELOCITY < 60)
						{
							MOTOR_LEFT_VELOCITY += 2;
							MOTOR_RIGHT_VELOCITY += 2;
						}
					}
				}
			}
		}
		nxt_motor_set_speed(MOTOR_LEFT, MOTOR_LEFT_VELOCITY, 1);
		nxt_motor_set_speed(MOTOR_RIGHT, MOTOR_RIGHT_VELOCITY, 1);
		ReleaseResource(VELOCITY);
		DIST_PREV = DIST;
	}
	TerminateTask();
}

TASK(TaskSonarSensor) //min-max noise filter(not median), every 50ms
{
	float sum_l, sum_r;
	float LR_SONAR_DIFF;
	float TRIAGOMETRIC_RATIO;
	while (1)
	{
		WaitEvent(SonarSensor);
		ClearEvent(SonarSensor);
		sum_l = 0;
		sum_r = 0;
		sonar_left = ecrobot_get_sonar_sensor(S1);
		sonar_right = ecrobot_get_sonar_sensor(S4);	
		if (sonar_left < 60) // min:5, max:60 
		{
			sonar_error_count_l=0;
			sonarBuff_left[sonarBuff_cursor1]=sonar_left; // 기존 방식의 배열을 선언해서 3개 데이터를 받고 처리하고 다시 3개 데이터를 받고 처리하는 방식이 아니라 circular buffer를 활용해서 데이터가 하나씩 들어오면 즉각 반응할수 있어서 빠른 반응속도를 보여준다.
			sonarBuff_cursor1++;
			if (sonarBuff_cursor1 == MAX_BUFF_SIZE) { sonarBuff_cursor1 = 0; } //circular buffer를 활용했다.
		}
		else { sonar_error_count_l++; }

		if (sonar_right < 60)
		{
			sonar_error_count_r=0;
			sonarBuff_right[sonarBuff_cursor2]=sonar_right;
			sonarBuff_cursor2++;
			if (sonarBuff_cursor2 == MAX_BUFF_SIZE) { sonarBuff_cursor2 = 0; } //circular buffer
		}
		else { sonar_error_count_r++; }
		// 에러카운터가 4번 부터 충분하다고 판단했다.
		if ((sonar_error_count_l > 3) && (sonar_error_count_r > 3)) // 이경우는 앞차는 완전히 놓친 경우
		{
			if (sonar_error_count_l > sonar_error_count_r) // 왼쪽 에러카운트가 많으면 앞차를 놓치기전 앞차가 우회전을 하고 있는 상황
			{
				theta = 30; //  오른쪽으로 꺾어서 앞차를 찾으려고 한다.
				GetResource(VELOCITY);
				goForward();
				ReleaseResource(VELOCITY);
			}
			else if (sonar_error_count_l < sonar_error_count_r) // 오른쪽 에러카운트가 많으면 앞차를 놓치기전 앞차가 좌회전을 하고 있는 상황
				theta = -30;// 왼쪽으로 꺾어서 앞차를 찾으려고 한다.
				GetResource(VELOCITY);
				goForward();
				ReleaseResource(VELOCITY);
			}
			else
			{
				theta = 0;
				GetResource(VELOCITY);
				doStop();
				ReleaseResource(VELOCITY);
				ecrobot_sound_tone(1175, 50, 10);
			}
			continue;
		}
		else if ((sonar_error_count_l > 3) && (sonar_error_count_r <= 3))// 왼쪽만 에러카운트 3이상이면 앞차가 우회전을 한것이고 앞차가 뒷차보다 작은 원의 궤적으로 도는 것이어서 뒷차를 30도 각도로 우회전 시킨다.
		{
			theta = 30;
			GetResource(VELOCITY);
			goForward();
			ReleaseResource(VELOCITY);
			continue;
		}
		else if ((sonar_error_count_l <= 3) && (sonar_error_count_r > 3))// 오른쪽만 에러카운트 3이상이면 앞차가 좌회전을 한것이고 앞차가 뒷차보다 작은 원의 궤적으로 도는 것이어서 뒷차를 30도 각도로 좌회전 시킨다.
		{
			theta = -30;
			GetResource(VELOCITY);
			goForward();
			ReleaseResource(VELOCITY);
			continue;
		}

		for(int i=0;i<MAX_BUFF_SIZE;i++) // 버퍼에 들어있는 값을 평균낸다.
		{
			sum_l=sum_l+sonarBuff_left[i];
			sum_r=sum_r+sonarBuff_right[i];
		}
		sonar_left_avrg = sum_l/MAX_BUFF_SIZE;
		sonar_right_avrg = sum_r/MAX_BUFF_SIZE;

		DIST = min(sonar_left_avrg, sonar_right_avrg);
		START = TRUE;

		LR_SONAR_DIFF = sonar_left_avrg - sonar_right_avrg;
		LR_SONAR_DIFF *= HEIGHT_WEIGHT; // 초음파 값의 차이가 차량 1의 판자를 빗변으로 하는 높이보다 훨씬 작게 측정되어 이를 보정하기 위한 계수를 곱해준다.
		TRIAGOMETRIC_RATIO = (float)LR_SONAR_DIFF/Sonar_Distance; // 삼각비, 높이 / 밑변
		theta = getDegree(ARCTAN_WEIGHT*arctan(TRIAGOMETRIC_RATIO));
	}
	TerminateTask();
}

