#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <term.h>
#include <termios.h>
#include <libgen.h>
#include "LinuxDARwIn.h"
#include "MotionManager.h"
#include "MotionStatus.h"
#include <python2.7/Python.h>

using namespace Robot;

LinuxCM730 linux_cm730("/dev/ttyUSB0");
CM730 cm730(&linux_cm730);

void change_current_dir()
{
    char exepath[1024] = {0};
    if(readlink("/proc/self/exe", exepath, sizeof(exepath)) != -1)
        chdir(dirname(exepath));
}

void sighandler(int sig)
{
    struct termios term;
    tcgetattr( STDIN_FILENO, &term );
    term.c_lflag |= ICANON | ECHO;
    tcsetattr( STDIN_FILENO, TCSANOW, &term );

    exit(0);
}

void action_play_motion(int motion_id);

void initMotionManager(char * ini_file_path){
    signal(SIGABRT, &sighandler);
    signal(SIGTERM, &sighandler);
    signal(SIGQUIT, &sighandler);
    signal(SIGINT, &sighandler);

    change_current_dir();

    minIni* ini = new minIni(ini_file_path); // ini_file_path should always be an absolute path

    if(MotionManager::GetInstance()->Initialize(&cm730) == false)
    {
        printf("Fail to initialize Motion Manager!\n");
        exit(1);
    }

    MotionManager::GetInstance()->AddModule((MotionModule*)Walking::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Action::GetInstance());
    MotionManager::GetInstance()->AddModule((MotionModule*)Head::GetInstance());

    LinuxMotionTimer *motion_timer = new LinuxMotionTimer(MotionManager::GetInstance());
    motion_timer->Start();

    MotionManager::GetInstance()->LoadINISettings(ini);
    Walking::GetInstance()->LoadINISettings(ini);
    Action::GetInstance()->LoadFile((char *)"/home/juarez/Darwin-tools/Data/motion_4096.bin");

    MotionManager::GetInstance()->SetEnable(true);

    action_play_motion(1); // maybe this should be removed later
    printf("Motion Manager initialized...\n");

}

void motion_load_ini_settings(char * ini_file_path){
    minIni * ini = new minIni(ini_file_path);
    MotionManager::GetInstance()->LoadINISettings(ini);
}

void walk_load_ini_settings(char * ini_file_path){
    minIni * ini = new minIni(ini_file_path);
    Walking::GetInstance()->LoadINISettings(ini);
}

void action_play_motion(int motion_id){
    // We need to enable the MotionManager instance to get a hold of the motors
    Action::GetInstance()->m_Joint.SetEnableBody(true, true);
    Action::GetInstance()->Start(motion_id);
    while(Action::GetInstance()->IsRunning()) usleep(8*1000);

    // After reproducing the motion we disable the MotionManager so we can
    // communicate directly to the motors via the cm730 object
    //MotionManager::GetInstance()->SetEnable(false);
}

// Walking functions
void walk_set_walk_velocities(float x, float y, float a){
    Walking::GetInstance()->X_MOVE_AMPLITUDE = x;
    Walking::GetInstance()->Y_MOVE_AMPLITUDE = y;
    Walking::GetInstance()->A_MOVE_AMPLITUDE = a;
}

void walk_start(){
    Walking::GetInstance()->m_Joint.SetEnableBodyWithoutHead(true, true);

    Walking::GetInstance()->Start();
}

bool walk_is_running(){
    // This function has a strange behavior, if this is called during a loop it throws
    // an overflow error after some time. I'm not sure why right now
    return Walking::GetInstance()->IsRunning();
}

void walk_stop(){
    Walking::GetInstance()->Stop();
    while(Walking::GetInstance()->IsRunning()) usleep(8*1000);
}

void walk_print_params(){
	printf("X_OFFSET: %lf\n", Walking::GetInstance()->X_OFFSET);
	printf("Y_OFFSET: %lf\n", Walking::GetInstance()->Y_OFFSET);
	printf("Z_OFFSET: %lf\n", Walking::GetInstance()->Z_OFFSET);
    printf("R_OFFSET: %lf\n", Walking::GetInstance()->R_OFFSET);
	printf("P_OFFSET: %lf\n", Walking::GetInstance()->P_OFFSET);
    printf("A_OFFSET: %lf\n", Walking::GetInstance()->A_OFFSET);
    printf("HIP_PITCH_OFFSET: %lf\n", Walking::GetInstance()->HIP_PITCH_OFFSET);
	printf("PERIOD_TIME: %lf\n", Walking::GetInstance()->PERIOD_TIME);
	printf("DSP_RATIO: %lf\n", Walking::GetInstance()->DSP_RATIO);
	printf("STEP_FB_RATIO: %lf\n", Walking::GetInstance()->STEP_FB_RATIO);
	printf("Z_MOVE_AMPLITUDE: %lf\n", Walking::GetInstance()->Z_MOVE_AMPLITUDE);
    printf("Y_SWAP_AMPLITUDE: %lf\n", Walking::GetInstance()->Y_SWAP_AMPLITUDE);
    printf("Z_SWAP_AMPLITUDE: %lf\n", Walking::GetInstance()->Z_SWAP_AMPLITUDE);
    printf("PELVIS_OFFSET: %lf\n", Walking::GetInstance()->PELVIS_OFFSET);
    printf("ARM_SWING_GAIN: %lf\n", Walking::GetInstance()->ARM_SWING_GAIN);
	printf("BALANCE_KNEE_GAIN: %lf\n", Walking::GetInstance()->BALANCE_KNEE_GAIN);
	printf("BALANCE_ANKLE_PITCH_GAIN: %lf\n", Walking::GetInstance()->BALANCE_ANKLE_PITCH_GAIN);
	printf("BALANCE_HIP_ROLL_GAIN: %lf\n", Walking::GetInstance()->BALANCE_HIP_ROLL_GAIN);
	printf("BALANCE_ANKLE_ROLL_GAIN: %lf", Walking::GetInstance()->BALANCE_ANKLE_ROLL_GAIN);
}

void walk_set_X_offset(double X){
    Walking::GetInstance()->X_OFFSET = X;
}

void walk_set_Y_offset(double Y){
    Walking::GetInstance()->Y_OFFSET = Y;
}

void walk_set_Z_offset(double Z){
    Walking::GetInstance()->Z_OFFSET = Z;
}

void walk_set_Hip_Pitch_offset(double hip_pitch_offset){
    Walking::GetInstance()->HIP_PITCH_OFFSET = hip_pitch_offset;
}

void walk_set_period_time(double ptime){
    Walking::GetInstance()->PERIOD_TIME = ptime;
}

void head_move_by_angle(double pan, double tilt){
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->MoveByAngle(pan, tilt);
}

void head_move_to_home(){
    Head::GetInstance()->m_Joint.SetEnableHeadOnly(true, true);
    Head::GetInstance()->MoveToHome();
}

double head_get_tilt(){
    double tilt;
    tilt = Head::GetInstance()->GetTiltAngle();
    return tilt;
}

double head_get_pan(){
    double pan;
    pan = Head::GetInstance()->GetPanAngle();
    return pan;
}

int joint_read(int id){
    int value, error;

    cm730.ReadWord(id, 0x24, &value, &error);
    
    if (error == 0)
        return value;
    else
        return -1;
}

void joint_set_value(int id, int value){
    int error;
    cm730.WriteWord(id, 0x1E, value, &error);
    if (error != 0)
        fprintf(stderr, "Couldn't write to motor %d\n", id);
}

void joint_enable_torque(int id){
    int error;
    cm730.WriteByte(id, 0x18, 1, &error);
    if (error != 0)
        fprintf(stderr, "Couldn't write to motor %d\n", id);
}

void joint_disable_torque(int id){
    int error;
    cm730.WriteByte(id, 0x18, 0, &error);
    if (error != 0)
        fprintf(stderr, "Couldn't write to motor %d\n", id);
}

int get_button(){
    return MotionStatus::BUTTON;
}

// PYTHON WRAPPING

static PyObject * initMotionManager_wrapper(PyObject * self, PyObject * args)
{
    char * ini_file_path;

    if (!PyArg_ParseTuple(args, "s", &ini_file_path)){
        printf("Failed to parse arugment.\n");
        return NULL;
    }

    initMotionManager(ini_file_path);
    Py_RETURN_NONE;
}

static PyObject * motion_load_ini_settings_wrapper(PyObject * self, PyObject * args)
{
    char * ini_file_path;

    if (!PyArg_ParseTuple(args, "s", &ini_file_path)){
        printf("Failed to parse arugment.\n");
        return NULL;
    }

    motion_load_ini_settings(ini_file_path);
    Py_RETURN_NONE;
}

static PyObject * walk_load_ini_settings_wrapper(PyObject * self, PyObject * args)
{
    char * ini_file_path;

    if (!PyArg_ParseTuple(args, "s", &ini_file_path)){
        printf("Failed to parse arugment.\n");
        return NULL;
    }

    walk_load_ini_settings(ini_file_path);
    Py_RETURN_NONE;
}

static PyObject * action_play_motion_wrapper(PyObject * self, PyObject * args){
    int motion_id;

    if (!PyArg_ParseTuple(args, "i", &motion_id)){
        printf("Failed to parse argument.\n");
        return NULL;
    }
    // Call actual function
    action_play_motion(motion_id);

    Py_RETURN_NONE;
}

static PyObject * walk_set_walk_velocities_wrapper(PyObject * self, PyObject * args){
    float x_val;
    float y_val;
    float a_val;

    if (!PyArg_ParseTuple(args, "fff", &x_val, &y_val, &a_val)){
        return NULL;
    }

    // Call the actual function
    walk_set_walk_velocities(x_val, y_val, a_val);

    Py_RETURN_NONE;
}

static PyObject * walk_start_wrapper(PyObject * self){
    walk_start();
    Py_RETURN_NONE;
}

static PyObject * walk_stop_wrapper(PyObject * self){
    walk_stop();
    Py_RETURN_NONE;
}

static PyObject * walk_is_running_wrapper(PyObject * self){
    bool isRunning;
    isRunning = walk_is_running();

    if (isRunning == true)
        return Py_True;
    else
        return Py_False;
}

static PyObject * walk_set_X_offset_wrapper(PyObject * self, PyObject * args){
    double x;

    if (!PyArg_ParseTuple(args, "d", &x))
        return NULL;

    walk_set_X_offset(x);

    Py_RETURN_NONE;
}

static PyObject * walk_set_Y_offset_wrapper(PyObject * self, PyObject * args){
    double y;

    if (!PyArg_ParseTuple(args, "d", &y))
        return NULL;

    walk_set_Y_offset(y);

    Py_RETURN_NONE;
}

static PyObject * walk_set_Z_offset_wrapper(PyObject * self, PyObject * args){
    double z;

    if (!PyArg_ParseTuple(args, "d", &z))
        return NULL;

    walk_set_Z_offset(z);

    Py_RETURN_NONE;
}

static PyObject * walk_set_Hip_Pitch_offset_wrapper(PyObject * self, PyObject * args){
    double hip_pitch_offset;

    if (!PyArg_ParseTuple(args, "d", &hip_pitch_offset))
        return NULL;

    walk_set_Hip_Pitch_offset(hip_pitch_offset);

    Py_RETURN_NONE;
}

static PyObject * walk_set_period_time_wrapper(PyObject * self, PyObject * args){
    double ptime;

    if (!PyArg_ParseTuple(args, "d", &ptime))
        return NULL;

    printf("ptime: %lf\n", ptime);
    walk_set_period_time(ptime);

    Py_RETURN_NONE;
}

static PyObject * joint_read_wrapper(PyObject * self, PyObject * args){
    int id, value;

    if (!PyArg_ParseTuple(args, "i", &id))
        return NULL;
    
    value = joint_read(id);

    return Py_BuildValue("i", value);
}

static PyObject * joint_enable_torque_wrapper(PyObject * self, PyObject * args){
    int id;

    if (!PyArg_ParseTuple(args, "i", &id))
        return NULL;

    joint_enable_torque(id);

    Py_RETURN_NONE;
}

static PyObject * joint_disable_torque_wrapper(PyObject * self, PyObject * args){
    int id;

    if (!PyArg_ParseTuple(args, "i", &id))
        return NULL;

    joint_disable_torque(id);

    Py_RETURN_NONE;
}

static PyObject * joint_set_value_wrapper(PyObject * self, PyObject * args){
    int id, value;

    if (!PyArg_ParseTuple(args, "ii", &id, &value))
        return NULL;

    joint_set_value(id, value);

    Py_RETURN_NONE;
}

static PyObject * head_move_by_angle_wrapper(PyObject * self, PyObject * args){
    double pan, tilt;

    if (!PyArg_ParseTuple(args, "dd", &pan, &tilt))
        return NULL;

    head_move_by_angle(pan, tilt);

    Py_RETURN_NONE;
}

static PyObject * head_move_to_home_wrapper(PyObject * self){
    head_move_to_home();

    Py_RETURN_NONE;
}

static PyObject * head_get_pan_wrapper(PyObject * self){
    double value;

    value = head_get_pan();

    return Py_BuildValue("d", value);
}

static PyObject * head_get_tilt_wrapper(PyObject * self){
    double value;

    value = head_get_tilt();

    return Py_BuildValue("d", value);
}

static PyObject * walk_print_params_wrapper(PyObject * self){
    walk_print_params();

    Py_RETURN_NONE;
}

static PyObject * get_button_wrapper(PyObject * self){
    int button;

    button = get_button();

    return Py_BuildValue("i", button);
}

static PyMethodDef darwin_motions_methods[] = {
    { "initMotionManager", (PyCFunction) initMotionManager_wrapper, METH_VARARGS, "Initialize Darwin Framework and Motion Manager."},
    { "motionLoadINI", (PyCFunction) motion_load_ini_settings_wrapper, METH_VARARGS, "Load a new INI file for Motion Manager"},
    { "walkLoadINI", (PyCFunction) walk_load_ini_settings_wrapper, METH_VARARGS, "Load a new INI file for Walking Manager"},
    { "playMotion", (PyCFunction) action_play_motion_wrapper, METH_VARARGS, "Play a recorded motion from the Action Editor."},
    { "getButton", (PyCFunction) get_button_wrapper, METH_NOARGS, "Get button press states."},
    { "walkStart", (PyCFunction) walk_start_wrapper, METH_NOARGS, "Start walking."},
    { "walkStop", (PyCFunction) walk_stop_wrapper, METH_NOARGS, "Stop walking."},
    { "walkIsRunning", (PyCFunction) walk_is_running_wrapper, METH_NOARGS, "Return status of walking."},
    { "walkSetVelocities", (PyCFunction) walk_set_walk_velocities_wrapper, METH_VARARGS, "Set X, Y and A amplitudes for Walking."},
    { "walkSetXOffset", (PyCFunction) walk_set_X_offset_wrapper, METH_VARARGS, "Set X Offset parameter."},
    { "walkSetYOffset", (PyCFunction) walk_set_Y_offset_wrapper, METH_VARARGS, "Set Y Offset parameter."},
    { "walkSetZOffset", (PyCFunction) walk_set_Z_offset_wrapper, METH_VARARGS, "Set Z Offset parameter."},
    { "walkSetHipPitchOffset", (PyCFunction) walk_set_Hip_Pitch_offset, METH_VARARGS, "Set Hip Pitch Offset parameter."},
    { "walkSetPeriodTime", (PyCFunction) walk_set_period_time_wrapper, METH_VARARGS, "Set Period Time Offset parameter."},
    { "walkPrintParams", (PyCFunction) walk_print_params_wrapper, METH_NOARGS, "Print Walking Parameters"},
    { "jointRead", (PyCFunction) joint_read_wrapper, METH_VARARGS, "Read current position of joint ID." },
    { "jointEnableTorque", (PyCFunction) joint_enable_torque_wrapper, METH_VARARGS, "Enable torque for joint of specified ID." },
    { "jointDisableTorque", (PyCFunction) joint_disable_torque_wrapper, METH_VARARGS, "Disable torque for joint of specified ID." },
    { "jointSetValue", (PyCFunction) joint_set_value_wrapper, METH_VARARGS, "Set value for joint of specified ID." },
    { "headMoveByAngle", (PyCFunction) head_move_by_angle_wrapper, METH_VARARGS, "Moves Head by Pan and Tilt amount from current position." },
    { "headMoveToHome", (PyCFunction) head_move_to_home_wrapper, METH_NOARGS, "Moves Head to default position." },
    { "headGetPan", (PyCFunction) head_get_pan_wrapper, METH_NOARGS, "Get Pan Angle" },
    { "headGetTilt", (PyCFunction) head_get_tilt_wrapper, METH_NOARGS, "Get Tilt Angle" },
    { NULL }
};

PyMODINIT_FUNC initdarwin_motions() {
    Py_InitModule3("darwin_motions", darwin_motions_methods, "meh");
}
