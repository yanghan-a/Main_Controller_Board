#include "common_inc.h"

extern DummyRobot dummy;

float pi = 3.14159265358979323846f;
extern bool start_counter;
void OnUsbAsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    char uart6_rx_buffer[128];
    sprintf(uart6_rx_buffer, "%s\n", _cmd);
    // HAL_UART_Transmit(&huart6, (uint8_t *) uart6_rx_buffer, _len+1, HAL_MAX_DELAY);
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '1')
    {
        dummy.SetEnable(true);

    }else if (_cmd[0] == '0')
    {
        dummy.SetEnable(false);
    }
    if(_cmd[0] == 't')
    {
        start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_CONTINUES_TRAJECTORY);

    }
    if(_cmd[0] == 'p')
    {
        start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL);

    }
    if(_cmd[0] == '2')
    {
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
        dummy.MoveJ(0, 0, 0, 0, 0, 0);
    }





    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    // if (_cmd[0] == '!')// || !dummy.IsEnabled())
    // {
    //     std::string s(_cmd);
    //     if (s.find("STOP") != std::string::npos)
    //     {
    //         dummy.commandHandler.EmergencyStop();
    //         Respond(_responseChannel, "Stopped ok");
    //     } else if (s.find("START") != std::string::npos)
    //     {
    //         dummy.SetEnable(true);
    //         Respond(_responseChannel, "Started ok");
    //     } else if (s.find("DISABLE") != std::string::npos)
    //     {
    //         dummy.SetEnable(false);
    //         Respond(_responseChannel, "Disabled ok");
    //     }
    // } else if (_cmd[0] == '#')
    // {
    //     std::string s(_cmd);
    //     if (s.find("GETJPOS") != std::string::npos)
    //     {
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentJoints.a[0], dummy.currentJoints.a[1],
    //                 dummy.currentJoints.a[2], dummy.currentJoints.a[3],
    //                 dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
    //     } else if (s.find("GETLPOS") != std::string::npos)
    //     {
    //         dummy.UpdateJointPose6D();
    //         Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
    //                 dummy.currentPose6D.X, dummy.currentPose6D.Y,
    //                 dummy.currentPose6D.Z, dummy.currentPose6D.A,
    //                 dummy.currentPose6D.B, dummy.currentPose6D.C);
    //     } else if (s.find("CMDMODE") != std::string::npos)
    //     {
    //         uint32_t mode;
    //         sscanf(_cmd, "#CMDMODE %lu", &mode);
    //         dummy.SetCommandMode(mode);
    //         Respond(_responseChannel, "Set command mode to [%lu]", mode);
    //     } else
    //         Respond(_responseChannel, "ok");
    // } else if (_cmd[0] == '>' || _cmd[0] == '@')
    // {
    //     uint32_t freeSize = dummy.commandHandler.Push(_cmd);
    //     Respond(_responseChannel, "%d", freeSize);
    // }else if(_cmd[0] == 'c')
    // {
    //     float joints[6];
    //     sscanf(_cmd, "c %f,%f,%f,%f,%f,%f", &joints[0], &joints[1], &joints[2], &joints[3], &joints[4], &joints[5]);
    //     dummy.ikCalculate(joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);
    //     Respond(_responseChannel, "ok");
    // }

/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}
static float deg2rad(float deg)
{
    return deg*pi/180.0f;
}
int id[6];
void OnUart1AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    // int a = 0;
    // char uart1_rx_buffer[128];
    // sprintf(uart1_rx_buffer, "%s\n", _cmd);
    // HAL_UART_Transmit(&huart6, (uint8_t *) uart6_rx_buffer, _len+1, HAL_MAX_DELAY);
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '1')
    {
        dummy.SetEnable(true);

    }else if (_cmd[0] == '0')
    {
        dummy.SetEnable(false);
    }
    if(_cmd[0] == 't')
    {
        start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_CONTINUES_TRAJECTORY);

    }
    if(_cmd[0] == 'p')
    {
        sscanf((char*) _cmd, "p %d %d %d %d %d %d", &id[0],&id[1],&id[2],&id[3],&id[4],&id[5]);

        start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_SEQUENTIAL);

    }
    if(_cmd[0] == 'i')
    {
        // start_counter = true;
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);

    }
    if(_cmd[0] == '2')
    {
        dummy.SetCommandMode(DummyRobot::COMMAND_TARGET_POINT_INTERRUPTABLE);
        dummy.MoveJ(0, 0, 0, 0, 0, 0);
    }

    if(_cmd[0] == 'a' && _cmd[1] == 'p')
    {
        dummy.motorJ[ALL]->ApplyPositionAsHome();
    }
    if(_cmd[0] == 'd' && _cmd[1] == 'y')
    {
        float j1 = 0.0f, j2 = -165.0f, j3 = 90.0f, j4 = 0.0f, j5 = 0.0f, j6 = 0.0f;
        float p = -40.0f;
        float v = 0.0f;
        float a = deg2rad(20*pi*pi);

        float j[6] ={deg2rad(j1+p), deg2rad(j2-p), deg2rad(j3+p), deg2rad(j4+p), deg2rad(j5+p), deg2rad(j6+p)};
        float vel[6] = {v, -v, v, v, v, v};
        float acc[6] = {a, -a, a, a, a, a};
        dummy.DynamicCalculation(j[0], j[1], j[2], j[3], j[4], j[5], vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], acc[0], acc[1], acc[2], acc[3], acc[4], acc[5]);
    }


}

void OnUart4AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
{
    /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
    if (_cmd[0] == '!' || !dummy.IsEnabled())
    {
        std::string s(_cmd);
        if (s.find("STOP") != std::string::npos)
        {
            dummy.commandHandler.EmergencyStop();
            Respond(_responseChannel, "Stopped ok");
        } else if (s.find("START") != std::string::npos)
        {
            dummy.SetEnable(true);
            Respond(_responseChannel, "Started ok");
        } else if (s.find("DISABLE") != std::string::npos)
        {
            dummy.SetEnable(false);
            Respond(_responseChannel, "Disabled ok");
        }
    } else if (_cmd[0] == '#')
    {
        std::string s(_cmd);
        if (s.find("GETJPOS") != std::string::npos)
        {
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentJoints.a[0], dummy.currentJoints.a[1],
                    dummy.currentJoints.a[2], dummy.currentJoints.a[3],
                    dummy.currentJoints.a[4], dummy.currentJoints.a[5]);
        } else if (s.find("GETLPOS") != std::string::npos)
        {
            dummy.UpdateJointPose6D();
            Respond(_responseChannel, "ok %.2f %.2f %.2f %.2f %.2f %.2f",
                    dummy.currentPose6D.X, dummy.currentPose6D.Y,
                    dummy.currentPose6D.Z, dummy.currentPose6D.A,
                    dummy.currentPose6D.B, dummy.currentPose6D.C);
        } else if (s.find("CMDMODE") != std::string::npos)
        {
            uint32_t mode;
            sscanf(_cmd, "#CMDMODE %lu", &mode);
            dummy.SetCommandMode(mode);
            Respond(_responseChannel, "Set command mode to [%lu]", mode);
        } else
            Respond(_responseChannel, "ok");
    } else if (_cmd[0] == '>' || _cmd[0] == '@')
    {
        uint32_t freeSize = dummy.commandHandler.Push(_cmd);
        Respond(_responseChannel, "%d", freeSize);
    }
/*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
}


// void OnUart5AsciiCmd(const char* _cmd, size_t _len, StreamSink &_responseChannel)
// {
//     /*---------------------------- ↓ Add Your CMDs Here ↓ -----------------------------*/
//
// /*---------------------------- ↑ Add Your CMDs Here ↑ -----------------------------*/
// }