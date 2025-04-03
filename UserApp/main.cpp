#include "common_inc.h"
#include "usart.h"


// On-board Screen, can choose from hi2c2 or hi2c0(soft i2c)
// SSD1306 oled(&hi2c0);
// On-board Sensor, used hi2c1
// MPU6050 mpu6050(&hi2c1);
// 5 User-Timers, can choose from htim7/htim10/htim11/htim13/htim14
Timer timerCtrlLoop(&htim7, 2400);
Timer timerUart1Tx(&htim10, 400);
// 2x2-channel PWMs, used htim9 & htim12, each has 2-channel outputs
// PWM pwm(21000, 21000);

// RGB rgb(0);
// Robot instance
DummyRobot dummy(&hcan1);

void ThreadControlLoopUpdate(void* argument);
void ThreadControlLoopFixUpdate(void* argument);
// void ThreadOledUpdate(void* argument);
//
// void ThreadRGBUpdate(void* argument);
void ThreadUartTx(void* argument);

char arr[10];
char arr1[30];

/* Default Entry -------------------------------------------------------*/
void Main(void)
{
    int s;
    // Init all communication staff, including USB-CDC/VCP/UART/CAN etc.
    InitCommunication();

    // Init Robot.
    dummy.Init();

    // Init IMU.
    // do
    // {
    //     mpu6050.Init();
    //     osDelay(100);
    // } while (!mpu6050.testConnection());
    // mpu6050.InitFilter(200, 100, 50);

    // Init OLED 128x80.
    // oled.Init();
    // pwm.Start();

    // Init & Run User Threads.
    const osThreadAttr_t controlLoopTask_attributes = {
        .name = "ControlLoopFixUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityRealtime,
    };
    controlLoopFixUpdateHandle = osThreadNew(ThreadControlLoopFixUpdate, nullptr,
                                             &controlLoopTask_attributes);

    const osThreadAttr_t ControlLoopUpdateTask_attributes = {
        .name = "ControlLoopUpdateTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    ControlLoopUpdateHandle = osThreadNew(ThreadControlLoopUpdate, nullptr,
                                          &ControlLoopUpdateTask_attributes);

    // const osThreadAttr_t oledTask_attributes = {
    //     .name = "OledTask",
    //     .stack_size = 2000,
    //     .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    // };
    // oledTaskHandle = osThreadNew(ThreadOledUpdate, nullptr, &oledTask_attributes);

    // const osThreadAttr_t rgbTask_attributes = {
    //         .name = "RGBTask",
    //         .stack_size = 2000,
    //         .priority = (osPriority_t) osPriorityNormal,   // should >= Normal
    // };
    // rgbTaskHandle = osThreadNew(ThreadRGBUpdate, nullptr, &rgbTask_attributes);

    const osThreadAttr_t uart1TxTask_attributes = {
        .name = "uart1TxTask",
        .stack_size = 2000,
        .priority = (osPriority_t) osPriorityNormal,
    };
    uart1TxTaskHandle = osThreadNew(ThreadUartTx, nullptr,
                                             &uart1TxTask_attributes);

    // Start Timer Callbacks.
    timerCtrlLoop.SetCallback(OnTimer7Callback);
    timerCtrlLoop.Start();

    timerUart1Tx.SetCallback(OnTimer10Callback);
    timerUart1Tx.Start();
    // System started, light switch-led up.
    // Respond(*uart4StreamOutputPtr, "[sys] Heap remain: %d Bytes\n", xPortGetMinimumEverFreeHeapSize());
    // pwm.SetDuty(PWM::CH_A1, 0.5);
}

/* Thread Definitions -----------------------------------------------------*/
osThreadId_t controlLoopFixUpdateHandle;
int i = 0;
// void ThreadControlLoopFixUpdate(void* argument)
// {
//     for (;;)
//     {
//         // Suspended here until got Notification.
//         ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
//         i++;
//         if (dummy.IsEnabled())
//         {
//             if(i%2==1)
//             {
//                 // Send control command to Motors & update Joint states
//                 switch (dummy.commandMode)
//                 {
//                 case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
//                 case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
//                 case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
//                     dummy.MoveJoints(dummy.targetJoints, (i+1)/2);
//                     // dummy.UpdateJointPose6D();
//                     break;
//                 case DummyRobot::COMMAND_MOTOR_TUNING:
//                     dummy.tuningHelper.Tick(10);
//                     // dummy.UpdateJointPose6D();
//                     break;
//                 }
//             }else
//             {
//                 dummy.UpdateAccCurrent(i/2);
//             }
//
//         } else
//         {
//             if(i%2==1)
//             {
//                 dummy.UpdateJointAngles((i+1)/2);
//             }
//
//         }
//         if(i ==12)
//         {
//             i = 0;
//             dummy.UpdateJointPose6D();
//         }
//     }
// }

//不考虑摩擦的激励轨迹
// const float a_matrix[6][5] = {{0.0133768074630065,	-0.0317559903458093,	0.00852983865655058,	-0.169812581363882,	0.179661925590134},
//                                 {0.142016170208041,	-0.00274584740581368,	0.229525150844855,	0.00460432196213471,	-0.373399795609217},
//                                 {-0.170123650644494,	0.280848483586662,	0.0729120440694084,	-0.451594531152250,	0.267957654140674},
//                                 {0.00331188389189863,	-0.235293203783329,	-0.0554689460589809,	0.181263529186930,	0.106186736763482},
//                                 {-0.0334338786968660,	-0.102485982518600,	0.0451664298707876,	0.204110650480638,	-0.113357219135959},
//                                 {-1.14613289722067,	1.08796892010524,	0.00207363483077734,	-0.0629966547085974,	0.119086996993252},
// };
// const float b_matrix[6][5] = {  {-0.0325918098125358,	0.601037718662437	,0.0159093102265894,	-0.637698029961109,	0.266716112330466   },
//                                 {-0.200539410922256	,-0.0139969603231995,	0.648038777698717,	-0.0100969843309848,	-0.335039012840712},
//                                 {-0.162782076298597	,0.247326149404952	,0.105546012020752,	0.157193200509001,	-0.255456212121913   },
//                                 {0.244857340925440	,0.921004837827730	,-0.0423350798944879,	-0.414162685143978,	-0.0606422072643047   },
//                                 {-0.406649384753491	,0.151195538025249	,-0.0422753894583372,	0.608219851184975,	-0.440358985532379   },
//                                 {-0.652338445680203	,0.179969751655420	,0.161755175359640,	0.0165215254562040,	-0.0517905371068745},
// };
// const float q[6] = {0.266025685590343,
// -1.84250450761731,
// -0.0250349768744878,
// 0.916062356372956,
// -0.447480430373331,
// -0.819111452172487};

// //考虑摩擦的激励轨迹
// const float a_matrix[6][5] = {  {-0.372921350992799,	0.800230133500048,	-0.000146425863259248,	-0.00187760998356908,	-0.425284746660420  },
//                                 {-0.111291975732336,	-0.00117461825546690,	0.415094567684443,	-0.299372700022677,	-0.00325527367396206},
//                                 {-0.604968768232842,	0.0117555794310133,	0.00194543836247742,	0.222450362712361,	0.368817387726990},
//                                 {-0.784783432947956,	0.0174397200938395,	0.264859226162591,	0.0469067798461657,	0.455577706845360},
//                                 {0.0925060704698541,	-1.01543141501217,	0.875684376844547,	0.0187973265403423,	0.0284436411574226     },
//                                 {1.16032519732057,	-0.972212830359580,	-0.248217447342346,	0.0303041244362141,	0.0298009559451420}
// };
// const float b_matrix[6][5] = {  {-0.0683560195597733,	0.405066406036169,	0.00209508738982305,	0.0120659516676020,	-0.159265172270488},
//                                 {-0.0151733904104474,	0.00113142947406032	,-0.701153068349810,	0.520925337014319,	0.00653367769089561},
//                                 {0.0762693315833463,	-0.0445682961939393,	-0.000658463038435213,	-0.0839391559494390,	0.0701198547435188},
//                                 {0.0224941424580454,	-0.268637601414481,	0.0635640123980592,	-0.0792014200419091,	0.128178940668875},
//                                 {-0.130084835108773,	0.433597523554898,	-0.117871189164456,	-0.0360801346791758,	-0.0478352211581904},
//                                 {-1.01831885696529,	-0.301530466895896,	0.132071509854076,	0.128795988632125,	0.141996261333272}
// };
// const float q[6] = {  0.168766309125438,
//                       -1.93120200541789,
//                       0.0744922765762772,
//                       -0.134965335012977,
//                       0.0458946393736652,
//                       -1.69414389729076   };

//考虑转子惯量的激励轨迹
const float a_matrix[6][5] = {  {-0.00667689144335028,0.00555279493518187,	0.215679888308004,	-0.222340260644520	,0.00778446884468378},
                                {0.323749409544191,	-1.91192425824356e-05,	-0.0369437114012986	,-0.0198066780245666,	-0.266979900875744},
                                {0.00117922116902068,	0.706470068442750,-0.352383225487775	,-0.118472741283678,	-0.236793322840318},
                                {0.781644860931682	,-0.763753965232784	,0.00134529062002458,	0.0192501849819349,	-0.0384863713008576},
                                {-0.0440609647339360,	0.419671759372827,	0.0515637059725410,	0.154239034189585,	-0.581413534801017},
                                {0.444704941662919,	-0.913825271634299,	0.224701354621913,	0.132344933162587,	0.112074042186880}
};
const float b_matrix[6][5] = {  {0.0620367356779697,	0.0299279697703382,	-0.590667821299827,	0.397526793676656	,0.0120007227948418},
                                {-0.0402002262450320,	5.41431153663920e-06,	0.0459638610991600,	-0.374201496461287	,0.279820760033925},
                                {-0.000372348957972621,	0.00331262141111746,	-0.0793349666790479,	0.0479164810920279	,0.00801721636095397},
                                {0.527629905710756,	-0.372040952319376,	0.0335419648760533,	0.00771092037174998,	0.0169964845625673},
                                {0.0779274845298214,	0.123082808700100,	-0.141238994040016,	0.180056474723353,	-0.124120403740677},
                                {0.407293500701078,	-0.485326517161727	,0.354625778190522,	-0.636071971007770	,0.408754016616378}
};
const float q[6] = { -0.0288177243549505,
                     -1.84474133537403,
                     -0.0184276971538652,
                     0.569961085034361,
                     0.179175460580366,
                     0.327177435284013
};
// //只考虑前三个连杆的参数辨识
// const float a_matrix[6][5] = {  {-0.109734434324185,	0.731292179754783	,0.0192520960207368,	0.000847504207294473,	-0.641657345658630},
//                                 {0.245719565130556,	-0.00234758207085609,	-0.730723709653135	,0.00231939796461203,	0.485032328628823},
//                                 {0.456008400885345,	0.0124032098649597	,-0.0983053820264452,	-0.137896458035836	,-0.232209770688024},
//     //后三行凑数用
//                                 {-0.297971478254164,	-0.0814018343322424,	0.190105787029727,	0.228620524479076,	-0.0393529989223976},
//                                 {-0.251217557274139,	0.423328167780684,	0.349034571763707,	-0.425741381387557,	-0.0954038008826945},
//                                 {-0.0191612369881367,	0.471751559113447,	0.0470406353038591,	-0.0845370049756288,	-0.415093952453540}
// };
// const float b_matrix[6][5] = {  {-0.0728827155202174,	-0.432308299662065,	-0.0283541058034824,	-0.0342440445821491	,0.231907562116678},
//                                 {-0.0782400464504203	,-0.000167266566902302,	0.0985494095260785,	0.00145241311241679,	-0.0445766602887356},
//                                 {0.0337962137246479	,-0.133712419635523,	0.293390806552602,	-0.466973468433938,	0.244270015924869},
//     //后三行凑数用
//                                 {0.354287964785823,	0.657977778919239	,0.0186552227447530,	-0.434144698889561,	0.00207392093993652},
//                                 {-0.281969778851347,	0.651881496765944,	0.104535620466192,	-0.0760356965675179,	-0.206251457961809},
//                                 {-1.06349371232455,	0.200326230071184,	-0.0141376866182501,	0.211108618551471,	-0.0278360324337908}
// };
// const float q[6] = { -0.414865581597213,
//                     -1.83131446920004,
//                     -0.00501723941754209,
//     //后三行凑数用
//                       0.925285254006925,
//                       0.0295343201134666,
//                       -1.46555157348570 };

bool start_counter = false;
int32_t point = 0;
float freq = 400.0f;
float dt = 1/freq;
float period = 10.0f;
float period_int = period*freq;
float all_time = period_int*1;
const float rad2deg = 57.295779513082320876798154814105f;
const float wf = 0.2*3.14159265358979323846f;

float sin_value[5];
float cos_value[5];
float vel[6];
float acc[6];
void ThreadControlLoopFixUpdate(void* argument)
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if(start_counter&& point<=all_time)
        {
            vel[i] = a_matrix[i][0]*cos_value[0]+b_matrix[i][0]*sin_value[0]+a_matrix[i][1]*cos_value[1]+b_matrix[i][1]*sin_value[1]+
                    a_matrix[i][2]*cos_value[2]+b_matrix[i][2]*sin_value[2]+a_matrix[i][3]*cos_value[3]+b_matrix[i][3]*sin_value[3]+
                        a_matrix[i][4]*cos_value[4]+b_matrix[i][4]*sin_value[4];
            vel[i] = vel[i]*rad2deg;
            acc[i] = -a_matrix[i][0]*sin_value[0]*wf*1+b_matrix[i][0]*cos_value[0]*wf*1-a_matrix[i][1]*sin_value[1]*wf*2+b_matrix[i][1]*cos_value[1]*wf*2-
                    a_matrix[i][2]*sin_value[2]*wf*3+b_matrix[i][2]*cos_value[2]*wf*3-a_matrix[i][3]*sin_value[3]*wf*4+b_matrix[i][3]*cos_value[3]*wf*4-
                        a_matrix[i][4]*sin_value[4]*wf*5+b_matrix[i][4]*cos_value[4]*wf*5;
            acc[i] = acc[i]*rad2deg;
        }
        i++;
        if (dummy.IsEnabled())
        {
                // Send control command to Motors & update Joint states
                switch (dummy.commandMode)
                {
                case DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL:
                case DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE:
                case DummyRobot::COMMAND_CONTINUES_TRAJECTORY:
                    dummy.MoveJoints(dummy.targetJoints, i);
                    // dummy.UpdateJointPose6D();
                    break;
                case DummyRobot::COMMAND_MOTOR_TUNING:
                    dummy.tuningHelper.Tick(10);
                    // dummy.UpdateJointPose6D();
                    break;
                }

        } else
        {
            dummy.UpdateJointAngles(i);
        }
        if(i ==6)
        {
            i = 0;
            dummy.UpdateJointPose6D();
        }
    }
}
uint8_t joint_angles[100];


float current_pos[6];
float pos[6];


float p1 = 45.0f,p2 = -45.0f,p3 = 50.0f,p4 = 45.0f,p5 = -40.0f,p6 = 90.0f;

extern int id[6];

osThreadId_t uart1TxTaskHandle;
void ThreadUartTx(void* argument)//500Hz
{
    for (;;)
    {
        // Suspended here until got Notification.
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        memcpy(joint_angles + 0 * sizeof(float), &dummy.currentJoints.a[0] ,sizeof(float));//targetPose6D
        memcpy(joint_angles + 1 * sizeof(float), &dummy.currentJoints.a[1] , sizeof(float));
        memcpy(joint_angles + 2 * sizeof(float), &dummy.currentJoints.a[2] , sizeof(float));
        memcpy(joint_angles + 3 * sizeof(float), &dummy.currentJoints.a[3] , sizeof(float));
        memcpy(joint_angles + 4 * sizeof(float), &dummy.currentJoints.a[4], sizeof(float));
        memcpy(joint_angles + 5 * sizeof(float),  &dummy.currentJoints.a[5], sizeof(float));

        // memcpy(joint_angles + 0 * sizeof(float), &pos[0] ,sizeof(float));//targetPose6D
        // memcpy(joint_angles + 1 * sizeof(float), &pos[1] , sizeof(float));
        // memcpy(joint_angles + 2 * sizeof(float), &pos[2] , sizeof(float));
        // memcpy(joint_angles + 3 * sizeof(float), &pos[3] , sizeof(float));
        // memcpy(joint_angles + 4 * sizeof(float), &pos[4], sizeof(float));
        // memcpy(joint_angles + 5 * sizeof(float),  &pos[5], sizeof(float));

        memcpy(joint_angles + 6 * sizeof(float),  &dummy.current.a[0], sizeof(float));
        memcpy(joint_angles + 7 * sizeof(float),  &dummy.current.a[1], sizeof(float));
        memcpy(joint_angles + 8 * sizeof(float),  &dummy.current.a[2], sizeof(float));
        memcpy(joint_angles + 9 * sizeof(float),  &dummy.current.a[3], sizeof(float));
        memcpy(joint_angles + 10 * sizeof(float), &dummy.current.a[4],  sizeof(float));
        memcpy(joint_angles + 11 * sizeof(float), &dummy.current.a[5],  sizeof(float));

        memcpy(joint_angles + 12 * sizeof(float),  &vel[0], sizeof(float));
        memcpy(joint_angles + 13 * sizeof(float),  &vel[1], sizeof(float));
        memcpy(joint_angles + 14 * sizeof(float),  &vel[2], sizeof(float));
        memcpy(joint_angles + 15 * sizeof(float),  &vel[3], sizeof(float));
        memcpy(joint_angles + 16 * sizeof(float),  &vel[4], sizeof(float));
        memcpy(joint_angles + 17 * sizeof(float),  &vel[5], sizeof(float));

        memcpy(joint_angles + 18 * sizeof(float),  &acc[0], sizeof(float));
        memcpy(joint_angles + 19 * sizeof(float),  &acc[1], sizeof(float));
        memcpy(joint_angles + 20 * sizeof(float),  &acc[2], sizeof(float));
        memcpy(joint_angles + 21 * sizeof(float),  &acc[3], sizeof(float));
        memcpy(joint_angles + 22 * sizeof(float),  &acc[4],sizeof(float));
        memcpy(joint_angles + 23 * sizeof(float),  &acc[5],sizeof(float));

        // memcpy(joint_angles + 12 * sizeof(float),  &dummy.acceleration.a[0], sizeof(float));
        // memcpy(joint_angles + 13 * sizeof(float),  &dummy.acceleration.a[1], sizeof(float));
        // memcpy(joint_angles + 14 * sizeof(float),  &dummy.acceleration.a[2], sizeof(float));
        // memcpy(joint_angles + 15 * sizeof(float),  &dummy.acceleration.a[3], sizeof(float));
        // memcpy(joint_angles + 16 * sizeof(float), &dummy.acceleration.a[4], sizeof(float));
        // memcpy(joint_angles + 17 * sizeof(float), &dummy.acceleration.a[5], sizeof(float));
        //
        // memcpy(joint_angles + 18 * sizeof(float),  &dummy.velocity.a[0], sizeof(float));
        // memcpy(joint_angles + 19 * sizeof(float),  &dummy.velocity.a[1], sizeof(float));
        // memcpy(joint_angles + 20 * sizeof(float),  &dummy.velocity.a[2], sizeof(float));
        // memcpy(joint_angles + 21 * sizeof(float),  &dummy.velocity.a[3], sizeof(float));
        // memcpy(joint_angles + 22 * sizeof(float),  &dummy.velocity.a[4],sizeof(float));
        // memcpy(joint_angles + 23 * sizeof(float),  &dummy.velocity.a[5],sizeof(float));

        joint_angles[4*24]=0x00;
        joint_angles[4*24+1]=0x00;
        joint_angles[4*24+2]=0x80;
        joint_angles[4*24+3]=0x7f;
        HAL_UART_Transmit_DMA(&huart1, (uint8_t*)joint_angles, 100);
        if(start_counter&& point<=all_time)
        {
            for(int i=1;i<6;i++)
            {
                sin_value[i-1] = sinf((float)point*dt*(float)i/period*2*3.14159265358979323846f);
                cos_value[i-1] = cosf((float)point*dt*(float)i/period*2*3.14159265358979323846f);
            }

            if(point == 0)
            {
                for(int i=0;i<6;i++)
                {
                    current_pos[i] = dummy.currentJoints.a[i];
                }
            }

            for(int i=0;i<6;i++)
            {
                if(id[i]==1)
                {
                    pos[i] = a_matrix[i][0]*sin_value[0]/wf/1-b_matrix[i][0]*cos_value[0]/wf/1+a_matrix[i][1]*sin_value[1]/wf/2-b_matrix[i][1]*cos_value[1]/wf/2+
                    a_matrix[i][2]*sin_value[2]/wf/3-b_matrix[i][2]*cos_value[2]/wf/3+a_matrix[i][3]*sin_value[3]/wf/4-b_matrix[i][3]*cos_value[3]/wf/4+
                        a_matrix[i][4]*sin_value[4]/wf/5-b_matrix[i][4]*cos_value[4]/wf/5 +q[i];
                    pos[i] = pos[i]*rad2deg;
                }else
                {
                    pos[i] = current_pos[i];
                }
            }

            point ++;
            dummy.MoveJ_Traj(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5],vel[0],vel[1],vel[2],vel[3],
                vel[4],vel[5],acc[0],acc[1],acc[2],acc[3],acc[4],acc[5]);
        }
        else
        {
            point = 0;
            start_counter = false;
            dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
        }
    }
}

osThreadId_t ControlLoopUpdateHandle;
void ThreadControlLoopUpdate(void* argument)
{
    for (;;)
    {
        dummy.commandHandler.ParseCommand(dummy.commandHandler.Pop(osWaitForever));
    }
}


// osThreadId_t oledTaskHandle;
// void ThreadOledUpdate(void* argument)
// {
//     uint32_t t = micros();
//     char buf[16];
//     char cmdModeNames[4][4] = {"SEQ", "INT", "TRJ", "TUN"};
//
//     for (;;)
//     {
//         // mpu6050.Update(true);
//
//         oled.clearBuffer();
//         oled.setFont(u8g2_font_5x8_tr);
//         // oled.setCursor(0, 10);
//         // oled.printf("IMU:%.3f/%.3f", mpu6050.data.ax, mpu6050.data.ay);
//         oled.setCursor(85, 10);
//         oled.printf("| FPS:%lu", 1000000 / (micros() - t));
//         t = micros();
//
//         oled.drawBox(0, 15, 128, 3);
//         oled.setCursor(0, 30);
//         oled.printf(">%3d|%3d|%3d|%3d|%3d|%3d",
//                     (int) roundf(dummy.currentJoints.a[0]), (int) roundf(dummy.currentJoints.a[1]),
//                     (int) roundf(dummy.currentJoints.a[2]), (int) roundf(dummy.currentJoints.a[3]),
//                     (int) roundf(dummy.currentJoints.a[4]), (int) roundf(dummy.currentJoints.a[5]));
//
//         oled.drawBox(40, 35, 128, 24);
//         oled.setFont(u8g2_font_6x12_tr);
//         oled.setDrawColor(0);
//         oled.setCursor(42, 45);
//         oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.X),
//                     (int) roundf(dummy.currentPose6D.Y), (int) roundf(dummy.currentPose6D.Z));
//         oled.setCursor(42, 56);
//         oled.printf("%4d|%4d|%4d", (int) roundf(dummy.currentPose6D.A),
//                     (int) roundf(dummy.currentPose6D.B), (int) roundf(dummy.currentPose6D.C));
//         oled.setDrawColor(1);
//         oled.setCursor(0, 45);
//         oled.printf("[XYZ]:");
//         oled.setCursor(0, 56);
//         oled.printf("[ABC]:");
//
//         oled.setFont(u8g2_font_10x20_tr);
//         oled.setCursor(0, 78);
//         if (dummy.IsEnabled())
//         {
//             for (int i = 1; i <= 6; i++)
//                 buf[i - 1] = (dummy.jointsStateFlag & (1 << i) ? '*' : '_');
//             buf[6] = 0;
//             oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], buf);
//         } else
//         {
//             oled.printf("[%s] %s", cmdModeNames[dummy.commandMode - 1], "======");
//         }
//
//         oled.sendBuffer();
//     }
// }

// osThreadId_t rgbTaskHandle;
// void ThreadRGBUpdate(void* argument)
// {
//     for (;;) {
//         if (dummy.GetRGBEnabled())
//         {
//             rgb.Run((RGB::Rgb_style_t)dummy.GetRGBMode());
//             osDelay(30);
//         }else
//         {
//             rgb.Run(RGB::ALLOff);
//             osDelay(30);
//         }
//     }
// }
//
// void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
// {
//     if(htim->Instance==TIM2)
//     {
//         HAL_TIM_PWM_Stop_DMA(&htim2, TIM_CHANNEL_4);
//         rgb.Interrupt(1);
//     }
// }





/* Timer Callbacks -------------------------------------------------------*/
void OnTimer7Callback()//2400
{

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    // if(dummy.withplanner)
    // {
    //     dummy.ControlLoop();
    // }

    // Wake & invoke thread IMMEDIATELY.
    vTaskNotifyGiveFromISR(TaskHandle_t(controlLoopFixUpdateHandle), &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




