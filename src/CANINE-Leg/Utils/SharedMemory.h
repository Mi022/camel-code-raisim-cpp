//
// Created by camel on 22. 9. 13.
//

#ifndef RAISIM_SHAREDMEMORY_H
#define RAISIM_SHAREDMEMORY_H

#define CMD_dT              0.001
#define CONTROL_dT          0.005
#define VISUAL_dT           0.01
#define MAX_COMMAND_DATA    10
#define MAX_CUSTOM_DATA     20
#define PI                  3.141592
#define R2D                 57.2957802
#define D2R                 0.0174533

enum CONTROL_STATE
{
    STATE_CONTROL_STOP,
    STATE_READY,
    STATE_HOME_READY,
    STATE_HOME_CONTROL,
    STATE_PD_CONTROL
};

enum VISUAL_STATE
{
    STATE_VISUAL_STOP,
    STATE_OPEN_RAISIM,
    STATE_UPDATE_VISUAL
};

enum COMMAND
{
    NO_ACT,
    CAN_ON,
    VISUAL_ON,
    MOTOR_ON,
    MOTOR_OFF,
    HOME,
    PD_CMD,
    CUSTOM_1,
    CUSTOM_2
};

typedef struct _UI_COMMAND_
{
    int userCommand;
    char userParamChar[MAX_COMMAND_DATA];
    int userParamInt[MAX_COMMAND_DATA];
    double userParamDouble[MAX_COMMAND_DATA];
} UI_COMMAND, *pUI_COMMAND;

typedef struct _SHM_
{
    bool newCommand;
    bool canStatus;
    bool motorStatus;
    int motorErrorStatus;
    int motorId;
    int controlState;
    int visualState;
    int motorTemp;
    double localTime;
    double motorPosition;
    double motorVelocity;
    double motorTorque;
    double motorDesiredTorque;
    double motorVoltage;
}SHM, *pSHM;

typedef struct _CUSTOM_DATA_ {
    double       customVariableDouble[MAX_CUSTOM_DATA];
    int         customVariableInt[MAX_CUSTOM_DATA];
} CUSTOM_DATA, *pCUSTOM_DATA;



#endif //RAISIM_SHAREDMEMORY_H
